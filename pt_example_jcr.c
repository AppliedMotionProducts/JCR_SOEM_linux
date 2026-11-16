/******************************************************************************
 * pt_example_jcr.c - Profile Torque (CiA 402 mode 4) example for the
 *                    JCR joint module, using the SOEM EtherCAT master.
 *
 * What it does: brings the drive to OPERATIONAL, enables it, waits for the
 * brake to open, commands TARGET_TORQUE_PCT, holds for HOLD_S, commands zero
 * and waits for the torque to reach zero, disables and exits.
 *
 * Profile Torque uses the drive's INTERNAL torque ramp. The master writes
 * the target torque and the drive ramps to it using the "torque slope"
 * (object 0x6087). No cyclic real-time setpoint streaming is required, so
 * a 10 ms cycle and a vanilla (non-PREEMPT_RT) kernel are enough.
 *
 * Build:  gcc pt_example_jcr.c -o pt_example_jcr -I <SOEM>/include \
 *             -I <SOEM>/build/include -I <SOEM>/osal -I <SOEM>/osal/linux \
 *             -I <SOEM>/oshw/linux -L <SOEM>/build -lsoem -lpthread -lrt
 * Run:    sudo ./pt_example_jcr <network-interface>
 *
 * SAFETY: torque mode has NO position limit - a free shaft keeps speeding up
 * until it reaches SPEED_LIMIT. Keep the joint clear and an emergency stop
 * in reach. Ctrl+C commands zero torque and disables the drive.
 ******************************************************************************/

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include "soem/soem.h"

/* ---- Settings: change these ----------------------------------------------- */
#define CYCLE_NS           10000000 /* 10 ms (profile modes don't need faster)          */
#define TARGET_TORQUE_PCT  5.0      /* peak torque, % of rated torque                   */
#define TORQUE_SLOPE_PCTPS 200.0    /* ramp rate, % of rated torque per second (0x6087) */
#define HOLD_S             2.0      /* time held at the peak                            */
#define BRAKE_S            0.5      /* wait after enable so the brake can open          */
#define STOP_TIMEOUT_S     5.0      /* timeout waiting for zero torque after stop       */
#define SPEED_LIMIT        5000     /* 0x2A47 speed limit in torque mode, counts/s      */
#define COUNTS_PER_REV     10000.0  /* command counts per OUTPUT revolution (100:1 unit) */
#define GEAR_RATIO         100.0    /* reducer ratio of your JCR: print only            */

/* ---- Derived (do not edit) -------------------------------------------------- */
#define NSEC_PER_SEC       1000000000
#define PEAK               ((int16)(TARGET_TORQUE_PCT  * 10))      /* unit: 0.1 % of rated */
#define SLOPE              ((uint32)(TORQUE_SLOPE_PCTPS * 10))     /* 0.1 %/s             */
#define BRAKE_CYCLES       ((long)(BRAKE_S         * 1e9 / CYCLE_NS))
#define HOLD_CYCLES        ((long)(HOLD_S          * 1e9 / CYCLE_NS))
#define TIMEOUT_CYCLES     ((long)(STOP_TIMEOUT_S  * 1e9 / CYCLE_NS))

/* ---- Process data. pack(1) makes the structs match the wire format exactly -- */
#pragma pack(push,1)
typedef struct {                   /* output PDO, 5 bytes */
   uint16 controlword;             /* 0x6040 */
   int8   mode;                    /* 0x6060 */
   int16  target_torque;           /* 0x6071, 0.1 % of rated torque */
} out_t;
typedef struct {                   /* the first 13 of the JCR's 35 input bytes */
   uint16 error;                   /* 0x603F */
   uint16 statusword;              /* 0x6041 -- bit 10 = target reached        */
   int8   mode_shown;              /* 0x6061                                   */
   int32  position;                /* 0x6064                                   */
   int32  follow_err;              /* 0x60F4                                   */
} in_t;
#pragma pack(pop)
#define JCR_INPUT_BYTES 35

static ecx_contextt ctx;
static uint8 IOmap[4096];
static out_t *out;
static in_t  *in;
static OSAL_THREAD_HANDLE thread;
static int   expectedWKC;
static long  brake_cnt, phase_cnt;
static int64 toff;
static volatile int run = 1;
static volatile int in_op, enabled, done;
static volatile int stop;    /* 0 ok, 1 bus lost, 2 drive fault, 3 start-up */
static volatile int phase;   /* 0 brake wait, 1 ramp up / cruise, 2 hold, 3 ramp down, 4 done */
static void on_sigint(int s){ (void)s; run = 0; }

/* ---- DC phase lock (optional at 10 ms) ------------------------------------ */
static void add_time_ns(ec_timet *ts, int64 ns){
   ec_timet a;
   a.tv_nsec = ns % NSEC_PER_SEC;
   a.tv_sec  = (ns - a.tv_nsec) / NSEC_PER_SEC;
   osal_timespecadd(ts, &a, ts);
}
static void ec_sync(int64 reftime, int64 cycle, int64 *offset){
   static int64 integral;
   int64 delta = reftime % cycle;
   if (delta > cycle / 2) delta -= cycle;
   integral += delta;
   *offset = -(delta / 100) - (integral / 20000);
}

static void sdo(uint16 slave, uint16 index, uint8 sub, int size, void *value){
   if (ecx_SDOwrite(&ctx, slave, index, sub, FALSE, size, value, EC_TIMEOUTRXM) <= 0)
      printf("SDO write 0x%04X:%d FAILED\n", index, sub);
}

/* ---- Drive setup: SOEM calls this during PRE-OP -> SAFE-OP ---------------- */
static int setup(ecx_contextt *c, uint16 s){
   (void)c;
   uint8 u8; uint16 u16; uint32 u32; int8 mode = 4;

   /* Limits. */
   u32 = SPEED_LIMIT; sdo(s, 0x2A47, 0, 4, &u32);   /* speed limit in torque mode */
   u16 = 1000;        sdo(s, 0x6072, 0, 2, &u16);   /* max torque                 */
   u16 = 1000;        sdo(s, 0x6073, 0, 2, &u16);   /* max current                */
   u16 = 3000;        sdo(s, 0x60E0, 0, 2, &u16);   /* positive torque limit      */
   u16 = 3000;        sdo(s, 0x60E1, 0, 2, &u16);   /* negative torque limit      */
   u32 = 1000000;     sdo(s, 0x607F, 0, 4, &u32);   /* max velocity               */

   /* Torque slope (0x6087): the drive ramps torque toward target at this rate. */
   u32 = SLOPE;       sdo(s, 0x6087, 0, 4, &u32);

   /* Output PDO remap: Controlword + Mode + Target Torque.                      */
   u8 = 0;            sdo(s, 0x1C12, 0, 1, &u8);
   u8 = 0;            sdo(s, 0x1600, 0, 1, &u8);
   u32 = 0x60400010;  sdo(s, 0x1600, 1, 4, &u32);
   u32 = 0x60600008;  sdo(s, 0x1600, 2, 4, &u32);
   u32 = 0x60710010;  sdo(s, 0x1600, 3, 4, &u32);
   u8 = 3;            sdo(s, 0x1600, 0, 1, &u8);
   u16 = 0x1600;      sdo(s, 0x1C12, 1, 2, &u16);
   u8 = 1;            sdo(s, 0x1C12, 0, 1, &u8);

   sdo(s, 0x6060, 0, 1, &mode);                     /* mode 4 = profile torque   */
   return 1;
}

/* ---- CiA 402 enable ladder ------------------------------------------------ */
static void ladder(uint16 sw){
   if      ((sw & 0x004F) == 0x0040) out->controlword = 0x06;
   else if ((sw & 0x006F) == 0x0021) out->controlword = 0x07;
   else if ((sw & 0x006F) == 0x0023) out->controlword = 0x0F;
   else if ((sw & 0x006F) == 0x0027){
      enabled = 1; brake_cnt = 0;
   }
   else if  (sw & 0x0008)            out->controlword = 0x80;
}

/* ---- Phase runner: just sets the target torque each phase ---------------- */
static void profile_runner(uint16 sw){
   if (brake_cnt < BRAKE_CYCLES){ brake_cnt++; return; }

   switch (phase){
      case 0:                                /* command peak torque                */
         out->target_torque = PEAK;
         phase = 1; phase_cnt = 0;
         break;
      case 1:                                /* ramp up                            */
         if (sw & 0x0400){ phase = 2; phase_cnt = 0; }   /* target reached         */
         break;
      case 2:                                /* hold at peak                       */
         if (++phase_cnt >= HOLD_CYCLES){
            out->target_torque = 0;
            phase = 3; phase_cnt = 0;
         }
         break;
      case 3:                                /* ramp down to zero                  */
         if (sw & 0x0400){ phase = 4; done = 1; }
         else if (++phase_cnt >= TIMEOUT_CYCLES){ stop = 3; run = 0; }
         break;
   }
}

/* ---- The cyclic thread: 10 ms is plenty for profile mode ------------------ */
OSAL_THREAD_FUNC_RT cyclic(void *arg){
   (void)arg;
   int miss = 0;
   ec_timet t; osal_get_monotonic_time(&t);
   t.tv_nsec = (t.tv_nsec / CYCLE_NS + 1) * CYCLE_NS;
   if (t.tv_nsec >= NSEC_PER_SEC){ t.tv_sec++; t.tv_nsec -= NSEC_PER_SEC; }
   ecx_send_processdata(&ctx);

   while (run){
      add_time_ns(&t, CYCLE_NS + toff);
      osal_monotonic_sleep(&t);
      int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      if (ctx.slavelist[0].hasdc) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

      uint16 sw = in->statusword;
      if (in_op){
         if (wkc < expectedWKC){
            if (++miss >= 8){ stop = 1; run = 0; }
         } else miss = 0;

         if (!enabled){
            ladder(sw);
         } else if (sw & 0x0008){
            out->target_torque = 0;
            stop = 2; run = 0;
         } else {
            out->controlword = 0x0F;
            profile_runner(sw);
         }
      }
      ecx_send_processdata(&ctx);
   }
   out->target_torque = 0;
   ecx_send_processdata(&ctx);
}

int main(int argc, char *argv[]){
   long i;
   printf("PT (Profile Torque) example for the JCR joint module (%d ms cycle)\n",
          CYCLE_NS / 1000000);
   if (argc != 2){ printf("Usage: sudo %s <network-interface>\n", argv[0]); return 1; }
   signal(SIGINT, on_sigint);
   mlockall(MCL_CURRENT | MCL_FUTURE);

   if (!ecx_init(&ctx, argv[1])){ printf("Cannot open %s - run with sudo\n", argv[1]); return 1; }
   if (ecx_config_init(&ctx) != 1){ printf("Expected exactly one drive\n"); ecx_close(&ctx); return 1; }
   printf("Found: %s\n", ctx.slavelist[1].name);

   ctx.slavelist[1].PO2SOconfig = setup;
   ecx_configdc(&ctx);
   ecx_dcsync0(&ctx, 1, TRUE, CYCLE_NS, 0);
   osal_usleep(2000);

   ecx_config_map_group(&ctx, IOmap, 0);
   ecx_statecheck(&ctx, 1, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
   ecx_readstate(&ctx);
   out = (out_t *)ctx.slavelist[1].outputs;
   in  = (in_t  *)ctx.slavelist[1].inputs;
   if (ctx.slavelist[1].state != EC_STATE_SAFE_OP ||
       ctx.slavelist[1].Obytes != sizeof(out_t) || ctx.slavelist[1].Ibytes != JCR_INPUT_BYTES){
      printf("SAFE-OP failed or unexpected PDO sizes (%d out / %d in, state 0x%02x)\n",
             ctx.slavelist[1].Obytes, ctx.slavelist[1].Ibytes, ctx.slavelist[1].state);
      stop = 3; goto cleanup;
   }

   out->controlword = 0; out->mode = 4; out->target_torque = 0;
   ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;

   osal_thread_create_rt(&thread, 128000, (void *)&cyclic, NULL);
   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
   if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL){
      printf("OPERATIONAL not reached (AL status 0x%04x)\n", ctx.slavelist[1].ALstatuscode);
      stop = 3; goto cleanup;
   }
   printf("Drive is OPERATIONAL\n");

   in_op = 1;
   for (i = 0; i < 2000 && !enabled && run; i++) osal_usleep(1000);
   if (!enabled){
      printf("Drive did not enable (statusword 0x%04x, error 0x%04x)\n", in->statusword, in->error);
      stop = 3; goto cleanup;
   }
   printf("Enabled. Brake wait %.1f s, then %.1f %% torque for %.1f s.\n",
          BRAKE_S, TARGET_TORQUE_PCT, HOLD_S);
   printf("Worst-case travel if the shaft is free: %.2f output rev\n",
          (double)SPEED_LIMIT / COUNTS_PER_REV * HOLD_S);
   printf("Speed limit: %.2f output rev/s = %.0f rpm at the motor (%.0f:1 reducer)\n",
          (double)SPEED_LIMIT / COUNTS_PER_REV,
          (double)SPEED_LIMIT / COUNTS_PER_REV * GEAR_RATIO * 60.0, GEAR_RATIO);

   while (run && !done){
      printf("phase=%d  target=%4d  position=%10d  statusword=0x%04x\n",
             phase, out->target_torque, in->position, in->statusword);
      osal_usleep(200000);
   }
   if      (stop == 1) printf("STOPPED: lost contact with the drive\n");
   else if (stop == 2) printf("STOPPED: drive fault, error 0x%04x\n", in->error);
   else if (stop == 3) printf("STOPPED: did not reach zero torque in time\n");
   else if (done)      printf("Profile complete\n");
   else                printf("Stopped by Ctrl+C\n");

cleanup:
   run = 0; osal_usleep(50000);
   if (out){
      out->controlword = 0; out->target_torque = 0;
      ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   }
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 1, EC_STATE_INIT, EC_TIMEOUTSTATE);
   ecx_dcsync0(&ctx, 1, FALSE, 0, 0);
   ecx_close(&ctx);
   printf("Done\n");
   return stop;
}
