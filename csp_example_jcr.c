/******************************************************************************
 * csp_example_jcr.c - Cyclic Synchronous Position (CiA 402 mode 8) example for
 *                     the JCR joint module, using the SOEM EtherCAT master.
 *
 * What it does: brings the drive to OPERATIONAL, enables it, waits for the
 * brake to open, moves MOVE_REVS one way, waits PAUSE_S, moves the same
 * distance back, waits until the shaft has settled, disables the drive and
 * exits.
 *
 * Use a cycle time of 1 ms or more: shorter CSP cycles have not been validated
 * on this drive.
 *
 * Build:  gcc csp_example_jcr.c -o csp_example_jcr -I <SOEM>/include \
 *             -I <SOEM>/build/include -I <SOEM>/osal -I <SOEM>/osal/linux \
 *             -I <SOEM>/oshw/linux -L <SOEM>/build -lsoem -lpthread -lrt
 * Run:    sudo ./csp_example_jcr <network-interface>
 *
 * SAFETY: the joint MOVES and there are no software position limits. Check the
 * move printed at start against the joint's mechanical travel. Keep an
 * emergency stop in reach. Ctrl+C holds the current position and disables
 * the drive.
 ******************************************************************************/

/* Expose clock_gettime / mlockall even when built with a strict -std= flag. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include "soem/soem.h"

/* ---- Settings: change these ----------------------------------------------- */
#define CYCLE_NS       1000000  /* bus cycle = Sync0 period (1 ms; whole ms only)       */
#define MOVE_REVS      2.0      /* output revolutions one way, then the same back       */
#define SPEED_RPS      0.5      /* peak speed, output rev/s                             */
#define ACCEL_RPS2     1.0      /* acceleration, output rev/s^2                         */
#define PAUSE_S        2.0      /* wait at the far end                                  */
#define BRAKE_S        0.5      /* wait after enabling so the brake can open            */
#define SETTLE_TOL     30       /* "home" = within this many counts of the start        */
#define SETTLE_S       4.0      /* give up waiting to settle after this long            */
#define COUNTS_PER_REV 10000.0  /* command counts per OUTPUT revolution (measured on a 100:1 unit) */
#define GEAR_RATIO     100.0    /* reducer ratio of your JCR: 100 or 80. Used only to print the motor
                                   speed. Do NOT scale the counts with it: the drive already does.  */

/* ---- Derived (do not edit) -------------------------------------------------- */
#if CYCLE_NS % 1000000 != 0
#error "CYCLE_NS must be a whole number of milliseconds for this sample"
#endif
#define NSEC_PER_SEC   1000000000
#define CYCLE_S        (CYCLE_NS / 1e9)
#define MOVE           (MOVE_REVS  * COUNTS_PER_REV)          /* counts            */
#define SPEED          (SPEED_RPS  * COUNTS_PER_REV)          /* counts/s          */
#define ACCEL          (ACCEL_RPS2 * COUNTS_PER_REV)          /* counts/s^2        */
#define PAUSE_CYCLES   ((long)(PAUSE_S  * 1e9 / CYCLE_NS))
#define BRAKE_CYCLES   ((long)(BRAKE_S  * 1e9 / CYCLE_NS))
#define SETTLE_CYCLES  ((long)(SETTLE_S * 1e9 / CYCLE_NS))

/* ---- Process data. pack(1) makes the structs match the wire format exactly -- */
#pragma pack(push,1)
typedef struct {                   /* output PDO, 7 bytes */
   uint16 controlword;             /* 0x6040 */
   int8   mode;                    /* 0x6060 */
   int32  target_position;         /* 0x607A, counts */
} out_t;
typedef struct {                   /* the first 13 of the JCR's 35 input bytes */
   uint16 error;                   /* 0x603F error code     */
   uint16 statusword;              /* 0x6041                */
   int8   mode_shown;              /* 0x6061                */
   int32  position;                /* 0x6064 actual position, counts */
   int32  follow_err;              /* 0x60F4                */
} in_t;
#pragma pack(pop)
#define JCR_INPUT_BYTES 35         /* size of the JCR's factory input image */

static ecx_contextt ctx;            /* everything SOEM knows about the bus        */
static uint8 IOmap[4096];           /* process data memory                        */
static out_t *out;                  /* what we send to the drive                  */
static in_t  *in;                   /* what the drive reports                     */
static OSAL_THREAD_HANDLE thread;
static int   expectedWKC;           /* working counter of a healthy cycle         */
static long  brake_cnt;             /* cycles spent waiting for the brake         */
static int32 base;                  /* position when the drive was enabled        */
static int64 toff;                  /* phase-lock correction, ns                  */
static volatile int run = 1;        /* Ctrl+C or any stop sets this to 0          */
static volatile int in_op, enabled, done;
static volatile int stop;           /* 0 ok, 1 bus lost, 2 drive fault, 3 start-up problem */
static void on_sigint(int s){ (void)s; run = 0; }

/* ---- Staying in step with the drive's clock (Distributed Clocks) ---------- */
static void add_time_ns(ec_timet *ts, int64 ns){
   ec_timet a;
   a.tv_nsec = ns % NSEC_PER_SEC;
   a.tv_sec  = (ns - a.tv_nsec) / NSEC_PER_SEC;
   osal_timespecadd(ts, &a, ts);
}
/* PI controller: reads where our frame landed inside the drive's cycle and
   trims the next period, so we follow the drive's clock instead of drifting.
   Here the frames are steered to land HALF A CYCLE after the Sync0 edge (the
   setting this CSP sample was validated with). */
static void ec_sync(int64 reftime, int64 cycle, int64 *offset){
   static int64 integral;
   int64 delta = (reftime % cycle) - cycle / 2;
   integral += delta;
   *offset = -(delta / 100) - (integral / 20000);
}

/* Write one object in the drive (SDO = configuration channel). */
static void sdo(uint16 slave, uint16 index, uint8 sub, int size, void *value){
   if (ecx_SDOwrite(&ctx, slave, index, sub, FALSE, size, value, EC_TIMEOUTRXM) <= 0)
      printf("SDO write 0x%04X:%d FAILED\n", index, sub);
}

/* ---- Drive setup: SOEM calls this while the drive goes PRE-OP -> SAFE-OP ---- */
static int setup(ecx_contextt *c, uint16 s){
   (void)c;
   uint8 u8; uint16 u16; uint32 u32; int8 e = -3, mode = 8;

   /* Limits. They are 0 after power-up and silently block motion, so set them. */
   u32 = 100000;      sdo(s, 0x2A47, 0, 4, &u32);   /* torque-mode speed limit; set high so it can never be the limiter */
   u16 = 1000;        sdo(s, 0x6072, 0, 2, &u16);   /* max torque,  0.1 % of rated  */
   u16 = 1000;        sdo(s, 0x6073, 0, 2, &u16);   /* max current, 0.1 % of rated  */
   u16 = 3000;        sdo(s, 0x60E0, 0, 2, &u16);   /* positive torque limit        */
   u16 = 3000;        sdo(s, 0x60E1, 0, 2, &u16);   /* negative torque limit        */
   u32 = 1000000;     sdo(s, 0x607F, 0, 4, &u32);   /* max velocity                 */

   /* Output PDO (what we send): controlword, mode of operation, target position. */
   u8 = 0;            sdo(s, 0x1C12, 0, 1, &u8);    /* unassign the old mapping     */
   u8 = 0;            sdo(s, 0x1600, 0, 1, &u8);    /* clear it                     */
   u32 = 0x60400010;  sdo(s, 0x1600, 1, 4, &u32);   /* 0x6040, 16 bit: controlword  */
   u32 = 0x60600008;  sdo(s, 0x1600, 2, 4, &u32);   /* 0x6060,  8 bit: mode         */
   u32 = 0x607A0020;  sdo(s, 0x1600, 3, 4, &u32);   /* 0x607A, 32 bit: target position */
   u8 = 3;            sdo(s, 0x1600, 0, 1, &u8);    /* three entries                */
   u16 = 0x1600;      sdo(s, 0x1C12, 1, 2, &u16);   /* assign it                    */
   u8 = 1;            sdo(s, 0x1C12, 0, 1, &u8);
   /* The input PDO stays at the drive's factory layout. */

   u8 = CYCLE_NS / 1000000;  sdo(s, 0x60C2, 1, 1, &u8);   /* interpolation period = cycle time ...  */
   sdo(s, 0x60C2, 2, 1, &e);                              /* ... in units of 10^-3 s (whole ms)     */
   sdo(s, 0x6060, 0, 1, &mode);                           /* mode 8 = cyclic synchronous position   */
   return 1;
}

/* ---- Target position for this cycle (absolute counts) ----------------------
   Called once per bus cycle after the drive is enabled. Waits for the brake,
   moves out, waits, moves back, then waits until the shaft is really home.
   Sets "done" at the end.                                                    */
static int32 position_cmd(void){
   static double pos, vel;         /* distance from the start (counts) and speed (counts/s) */
   static int  leg;                /* 0 move out, 1 wait, 2 move back, 3 settle at home     */
   static long pause_cnt, settle;

   if (brake_cnt < BRAKE_CYCLES){ brake_cnt++; return base; }

   if (leg == 1){ if (++pause_cnt >= PAUSE_CYCLES) leg = 2; }
   else if (leg == 3){             /* the shaft trails the target: disable only once it arrived */
      if (abs(in->position - base) <= SETTLE_TOL || ++settle >= SETTLE_CYCLES) done = 1;
   } else {                        /* trapezoid: speed up, cruise, slow down to stop on the goal */
      double goal = (leg == 0) ? MOVE : 0.0;
      double dist = goal - pos, dir = (dist >= 0) ? 1.0 : -1.0;
      double left = dir * dist;                         /* distance still to go          */
      if (left <= vel * vel / (2.0 * ACCEL)) vel -= ACCEL * CYCLE_S;   /* time to brake   */
      else if (vel < SPEED)                  vel += ACCEL * CYCLE_S;
      if (vel < 0.0)   vel = 0.0;
      if (vel > SPEED) vel = SPEED;
      if (vel * CYCLE_S >= left){ pos = goal; vel = 0.0; leg = (leg == 0) ? 1 : 3; }
      else pos += dir * vel * CYCLE_S;
   }
   return base + (int32)pos;
}

/* CiA 402 enable sequence: look at the drive's statusword, answer with the
   next controlword. Once it reports "operation enabled", we are ready to move. */
static void ladder(uint16 sw){
   if      ((sw & 0x004F) == 0x0040) out->controlword = 0x06;   /* switch on disabled -> shutdown   */
   else if ((sw & 0x006F) == 0x0021) out->controlword = 0x07;   /* ready to switch on -> switch on  */
   else if ((sw & 0x006F) == 0x0023) out->controlword = 0x0F;   /* switched on -> enable operation  */
   else if ((sw & 0x006F) == 0x0027){                           /* operation enabled: ready to move */
      enabled = 1; base = in->position; brake_cnt = 0;
   }
   else if  (sw & 0x0008)            out->controlword = 0x80;   /* fault -> reset                   */
}

/* ---- The real-time loop: runs every CYCLE_NS, locked to the drive's clock --- */
OSAL_THREAD_FUNC_RT cyclic(void *arg){
   (void)arg;
   int miss = 0;
   ec_timet t; osal_get_monotonic_time(&t);
   t.tv_nsec = (t.tv_nsec / CYCLE_NS + 1) * CYCLE_NS;       /* start on a cycle boundary */
   ecx_send_processdata(&ctx);

   while (run){
      add_time_ns(&t, CYCLE_NS + toff);                      /* next cycle + phase correction */
      osal_monotonic_sleep(&t);
      int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);   /* the drive's answer arrives */
      if (ctx.slavelist[0].hasdc) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

      uint16 sw = in->statusword;
      if (in_op){
         if (wkc < expectedWKC){                             /* 8 bad cycles in a row = bus lost */
            if (++miss >= 8){ stop = 1; run = 0; }
         } else miss = 0;

         if (!enabled){
            ladder(sw);                                      /* walk the drive up to "enabled" */
            out->target_position = in->position;             /* follow the shaft: no jump at enable */
         } else if (sw & 0x0008){                            /* drive fault while running */
            out->target_position = in->position;               /* hold where we are */
            stop = 2; run = 0;
         } else {
            out->controlword = 0x0F;
            out->target_position = position_cmd();
         }
      }
      ecx_send_processdata(&ctx);
   }
   out->target_position = in->position;                     /* last frame: hold the position */
   ecx_send_processdata(&ctx);
}

int main(int argc, char *argv[]){
   long i;
   printf("CSP example for the JCR joint module (%d us cycle)\n", CYCLE_NS / 1000);
   if (argc != 2){ printf("Usage: sudo %s <network-interface>\n", argv[0]); return 1; }
   signal(SIGINT, on_sigint);                     /* Ctrl+C sets run = 0          */
   mlockall(MCL_CURRENT | MCL_FUTURE);            /* keep the program out of swap */

   /* 1. Open the network card and find the drive (it starts in PRE-OP). */
   if (!ecx_init(&ctx, argv[1])){ printf("Cannot open %s - run with sudo\n", argv[1]); return 1; }
   if (ecx_config_init(&ctx) != 1){ printf("Expected exactly one drive\n"); ecx_close(&ctx); return 1; }
   printf("Found: %s\n", ctx.slavelist[1].name);

   /* 2. Distributed clocks: start the drive's Sync0 pulse BEFORE going to SAFE-OP. */
   ctx.slavelist[1].PO2SOconfig = setup;          /* SOEM calls setup() in step 3 */
   ecx_configdc(&ctx);
   ecx_dcsync0(&ctx, 1, TRUE, CYCLE_NS, 0);
   osal_usleep(2000);

   /* 3. Map the process data (this runs setup()) and check the drive reached SAFE-OP. */
   ecx_config_map_group(&ctx, IOmap, 0);
   ecx_statecheck(&ctx, 1, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
   ecx_readstate(&ctx);                           /* read the real state, not a cached one */
   out = (out_t *)ctx.slavelist[1].outputs;
   in  = (in_t  *)ctx.slavelist[1].inputs;
   if (ctx.slavelist[1].state != EC_STATE_SAFE_OP ||
       ctx.slavelist[1].Obytes != sizeof(out_t) || ctx.slavelist[1].Ibytes != JCR_INPUT_BYTES){
      printf("SAFE-OP failed or unexpected PDO sizes (%d out / %d in, state 0x%02x)\n",
             ctx.slavelist[1].Obytes, ctx.slavelist[1].Ibytes, ctx.slavelist[1].state);
      stop = 3; goto cleanup;
   }

   /* 4. Start the real-time thread, then ask for OPERATIONAL. The thread must
         already be sending frames: the drive only accepts OP while data arrives. */
   ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);   /* get real input data first */
   out->controlword = 0; out->mode = 8; out->target_position = in->position;
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

   /* 5. Let the thread enable the drive, then wait while the profile runs. */
   in_op = 1;
   for (i = 0; i < 2000 && !enabled && run; i++) osal_usleep(1000);
   if (!enabled){
      printf("Drive did not enable (statusword 0x%04x, error 0x%04x)\n", in->statusword, in->error);
      stop = 3; goto cleanup;
   }
   printf("Enabled. Brake wait %.1f s, then +%.2f output rev (%.1f deg), wait %.1f s, and back.\n",
          BRAKE_S, MOVE_REVS, MOVE_REVS * 360.0, PAUSE_S);
   printf("Peak speed %.2f output rev/s = %.0f rpm at the motor (%.0f:1 reducer)\n",
          SPEED_RPS, SPEED_RPS * GEAR_RATIO * 60.0, GEAR_RATIO);
   while (run && !done){
      printf("target=%10d  actual=%10d  lag=%5d  statusword=0x%04x\n",
             out->target_position, in->position, out->target_position - in->position, in->statusword);
      osal_usleep(100000);
   }
   if      (stop == 1) printf("STOPPED: lost contact with the drive\n");
   else if (stop == 2) printf("STOPPED: drive fault, error 0x%04x\n", in->error);
   else if (done)      printf("Move complete\n");
   else                printf("Stopped by Ctrl+C\n");

cleanup:
   /* 6. Shut down in this order: command zero and disable, leave OPERATIONAL,
         and only then stop Sync0 (the drive needs Sync0 until it has left OP). */
   run = 0; osal_usleep(50000);                   /* let the thread send its last frame */
   if (out){
      out->controlword = 0;
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
