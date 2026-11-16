# JCR SOEM — EtherCAT Samples (Linux)

Open-source EtherCAT master sample programs for the **Applied Motion Products [JCR Series](https://www.applied-motion.com/s/jcr-series) robotic joint modules**, built on [SOEM 2.x](https://github.com/OpenEtherCATsociety/SOEM) and real-time Linux (PREEMPT_RT).

Each sample brings a single JCR joint to OPERATIONAL, enables it, runs a mode-appropriate motion profile, and shuts down cleanly. These are drop-in reference programs tuned to the JCR's factory PDO layout, brake behavior, and harmonic reducer.

<p align="center">
  <img width="250" alt="JCR joint module" src="https://github.com/user-attachments/assets/1ac016fd-6b45-4d6f-9e1b-147980fe7816" />
</p>

<p align="center">
  <img alt="JCR" src="https://img.shields.io/badge/-JCR%20Joint%20Module-003366?style=for-the-badge&logoColor=white" />
  <img alt="EtherCAT" src="https://img.shields.io/badge/-EtherCAT-7F2A2A?style=for-the-badge&logoColor=white" />
  <img alt="SOEM" src="https://img.shields.io/badge/-SOEM%202.x-A8B9CC?style=for-the-badge&logoColor=black" />
  <img alt="PREEMPT_RT" src="https://img.shields.io/badge/-Linux%20PREEMPT__RT-FCC624?style=for-the-badge&logo=linux&logoColor=black" />
  <img alt="CiA 402" src="https://img.shields.io/badge/-CiA%20402-0A66C2?style=for-the-badge&logoColor=white" />
  <img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?style=for-the-badge" />
</p>

---

## 🎯 Modes covered

| Mode | Code | File | Cycle | Status |
|------|------|------|-------|--------|
| Cyclic Synchronous Position (CSP) | 8 | [`csp_example_jcr.c`](csp_example_jcr.c) | 1 ms | ✅ tested |
| Cyclic Synchronous Velocity (CSV) | 9 | [`csv_example_jcr.c`](csv_example_jcr.c) | 500 µs | ✅ tested |
| Cyclic Synchronous Torque (CST) | 10 | [`cst_example_jcr.c`](cst_example_jcr.c) | 500 µs | ✅ tested |
| Profile Position (PP) | 1 | [`pp_example_jcr.c`](pp_example_jcr.c) | 10 ms | ✅ tested |
| Profile Velocity (PV) | 3 | [`pv_example_jcr.c`](pv_example_jcr.c) | 10 ms | ✅ tested |
| Profile Torque (PT) | 4 | [`pt_example_jcr.c`](pt_example_jcr.c) | 10 ms | ✅ tested |

> **Note:** CSP is validated at 1 ms. Shorter CSP cycles have not been tested on this drive — CSV/CST run safely at 500 µs.

---

## 🦾 Hardware

| Component | Details |
|---|---|
| **Drive** | Applied Motion Products **JCR Series** robotic joint module — [product page](https://www.applied-motion.com/s/jcr-series) |
| **Reducer** | Harmonic (100:1 or 80:1 depending on model) |
| **Bus** | EtherCAT (native) |
| **Protocol** | CoE — CiA 402 drive profile |
| **PDO input image** | 35 bytes (factory default; the sample reads the leading 13 bytes) |

> The constants `COUNTS_PER_REV = 10000` were measured on a **100:1 JCR**. Verify against your unit's datasheet before running — the drive already scales the gear ratio internally, so **do not multiply the commanded counts by the ratio yourself**.

---

## 🔧 Requirements

- Linux with the **PREEMPT_RT** kernel (Ubuntu 24.04 + Ubuntu Pro real-time kernel used for validation)
- **SOEM 2.x** built from source
- Direct Ethernet connection to the JCR (Intel-based NIC recommended)
- Root privileges (raw socket access)

## 🛠️ Build

```bash
gcc csp_example_jcr.c -o csp_example_jcr \
    -I <SOEM>/include -I <SOEM>/build/include \
    -I <SOEM>/osal -I <SOEM>/osal/linux -I <SOEM>/oshw/linux \
    -L <SOEM>/build -lsoem -lpthread -lrt
```

Substitute `csv_example_jcr.c` or `cst_example_jcr.c` for the other modes.

## ▶️ Run

```bash
sudo ./csp_example_jcr <network-interface>     # e.g. enp0s31f5
```

Each sample prints:
- the move/profile it is about to run (in output revolutions **and** degrees)
- motor-side speed in RPM given the configured gear ratio (printout only — does not affect the commanded counts)
- live `target / actual / statusword` once enabled

---

## 🔑 Key implementation notes

- **Startup parameters:** the drive ships with torque/velocity limit objects at 0 (`0x6072`, `0x6073`, `0x60E0`, `0x60E1`, `0x607F`, `0x2A47`). Each zero silently clamps motion while the drive reports healthy status. The samples write the full chain during the PRE-OP → SAFE-OP hook.
- **PDO remap:** `0x1600` is reassigned in PRE-OP to `Controlword + Modes of operation + mode-specific target` (position for CSP, velocity for CSV, torque for CST). The factory input PDO (35 bytes) is kept as-is.
- **Distributed Clocks:** `Sync0` is armed before SAFE-OP, with a short settle delay. The phase-lock PI controller tracks the drive's DC clock every cycle.
  - CSP steers frames to **½ cycle after Sync0** (validated with this sample)
  - CSV and CST steer frames to the **Sync0 edge** (standard SOEM pattern)
- **Interpolation period (`0x60C2`)** is set only in CSP — CSV and CST do not interpolate position.
- **Brake wait:** after reaching OPERATIONAL + enabled, the samples hold `target = current` for `BRAKE_S` (default 0.5 s) so the holding brake can release before any motion command is applied.
- **Bus-lost detection:** 8 consecutive bad working counters → controlled stop, hold position, exit non-zero.
- **Fault handling:** drive fault bit set mid-run → zero the command, exit non-zero (surface the drive error code).
- **Shutdown order:** command zero → disable → leave OP → stop Sync0. Sync0 must remain active until the drive has left OP, otherwise the drive trips its own sync-loss monitor.
- **Gear ratio:** printed in the console for operator context (motor-side RPM). **Do NOT scale the command counts by the ratio** — the drive already does.

---

## ⚠️ Safety

These programs move motor shafts. Read the safety block in each file before running:

- **CSP (`csp_example_jcr.c`)**: executes a trapezoidal move (default: ±2 output revolutions). No software position limits — verify against the joint's mechanical travel before running.
- **CSV (`csv_example_jcr.c`)**: rotates the joint continuously at the commanded velocity. Not suitable for travel-limited axes without adjusting the profile.
- **CST (`cst_example_jcr.c`)**: commands motor torque. A free shaft **accelerates until it reaches the velocity limit** (`0x2A47`, default 5,000 counts/s ≈ 0.5 output rev/s on a 100:1 JCR). Verify the limit matches your mechanical envelope.

All three perform a controlled zero-command shutdown on **Ctrl+C**.

---

## 📚 References

- [Applied Motion JCR Series](https://www.applied-motion.com/s/jcr-series) — product page
- [SOEM — Simple Open EtherCAT Master](https://github.com/OpenEtherCATsociety/SOEM)
- [CiA 402 drive profile (ETG)](https://www.ethercat.org/en/downloads.html)
- [AMP_EtherCAT_SOEM_Samples](https://github.com/AppliedMotionProducts/AMP_EtherCAT_SOEM_Samples) — public sister repo for MDX+ EC servo drives

---

## 📝 License

Apache License 2.0 — see [LICENSE](LICENSE). Free to use, modify, and redistribute (including commercially); keep the copyright notice preserved on redistribution.
