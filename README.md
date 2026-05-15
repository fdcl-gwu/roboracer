# RoboRacer

Raceline-following controllers for the FDCL RoboRacer 1/10-scale autonomous car. The car is localised by a Vicon motion-capture system and commanded via SiK radio over serial. Three independent controllers share the same hardware interface, raceline format, dashboard, simulator, and lap counter — only the steering law differs.

---

## Pick a controller

| Controller | Type | When to use | Detailed setup |
|---|---|---|---|
| **PID** | Reactive (no horizon, no model) | Baseline. Quickest to run. CLI-tunable gains — no recompile. | [README_PID.md](README_PID.md) |
| **MPPI** | Stochastic sampling MPC (300 rollouts) | Production controller. Currently the best-behaved on hardware. | [README_MPPI.md](README_MPPI.md) |
| **MPC** | Deterministic NMPC (acados, SQP-RTI) | Smoothest predicted steering trace; needs a one-time acados build. | [README_MPC.md](README_MPC.md) |

All three implement the same kinematic-bicycle model and read the same raceline CSV (`raceline.csv` for the oval, `figure_eight.csv` for the figure-8 test track).

---

## Quick start (simulator)

No hardware required. Verify the controller in the bundled simulator first:

```bash
# Cheapest — no extra deps
python3 pid_racing.py  --simulation --raceline figure_eight.csv --laps 3

# MPPI — numpy only, no compilation
python3 mppi_racing.py --simulation --raceline figure_eight.csv --laps 3

# MPC — first run triggers a ~30 s acados compile
python3 mpc_racing.py  --simulation --raceline figure_eight.csv --laps 3
```

All three open the same live matplotlib dashboard: CTE, heading error, velocity error, track view, controller-specific diagnostic panel, and steering indicator.

---

## Repository layout

```
roboracer/
├── README.md                ← this file
├── README_PID.md            ← PID controller usage and tuning
├── README_MPPI.md           ← MPPI controller usage and tuning
├── README_MPC.md            ← MPC controller usage and tuning, acados setup
│
├── pid_racing.py            ← PID baseline (Stanley-style)
├── mppi_racing.py           ← MPPI controller (numpy)
├── mpc_racing.py            ← Nonlinear MPC controller (acados + CasADi)
│
├── raceline.csv             ← Oval track raceline
├── figure_eight.csv         ← Figure-8 track raceline
├── gen_figure_eight.py      ← Regenerates figure_eight.csv
│
├── serial_test.py           ← Serial-link sanity check
├── sik_test.py              ← SiK radio sanity check
├── waypoint_nav.py          ← Standalone waypoint follower (legacy)
│
├── notes/notes.tex          ← Mathematical overview (model, all three controllers, results)
├── vicon/                   ← Vicon tracker library (C++/pybind11 + VRPN)
└── acados/                  ← acados submodule (only needed for mpc_racing.py)
```

---

## Documentation

- Per-controller usage, setup, and tuning: see the three `README_*.md` files above.
- Mathematical derivation of the vehicle model, both predictive controllers, and the PID law: [notes/notes.tex](notes/notes.tex) (compile with `pdflatex notes.tex`).
- Hardware bring-up procedure (Vicon, SiK radio, serial protocol): covered in `README_MPPI.md` and `README_PID.md` — same for all three controllers.

---

## Hardware overview

- **Vehicle:** 1/10 scale RC car, kinematic bicycle parameters L = 0.324 m, δ_max = 0.44 rad, v_max = 15 m/s.
- **Localisation:** Vicon motion-capture system (subject name `UGV`, server `192.168.11.2`).
- **Communication:** SiK radio over USB serial at 230400 baud; custom CRC-16 framed protocol.
- **Control rate:** 40 Hz (`time.sleep(0.025)` per loop) — all three controllers match this period.

---

## Emergency stop

Every controller's `finally` block sends a zero-throttle, centred-steering packet on normal exit, `Ctrl-C`, or unhandled exception. Keep a finger on `Ctrl-C` for the first runs.
