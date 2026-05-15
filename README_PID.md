# RoboRacer PID Racing

Stanley-style PID baseline for the RoboRacer platform.
Mirrors `mppi_racing.py` and `mpc_racing.py` in structure — same Vicon interface, same serial protocol, same lap counting, same live dashboard, same `--simulation` mode — but the predictive solver is replaced by a fixed-form, single-step PD/PID law on the current cross-track and heading errors.

This is the **baseline controller**. It exists to provide a fair, reactive-only point of comparison against MPPI and MPC. There is no horizon, no internal model, and no optimisation.

---

## How it differs from MPPI and MPC

| | `pid_racing.py` | `mppi_racing.py` | `mpc_racing.py` |
|---|---|---|---|
| Algorithm | Stanley-style PID on current errors | Stochastic sampling (300 rollouts) | Deterministic NMPC, SQP-RTI |
| Horizon | None — reacts to current error only | 20 steps of forward Euler | 20 steps of ERK4 |
| Internal model | None | Kinematic bicycle (vectorised) | Kinematic bicycle (CasADi → C) |
| Startup cost | None | None | ~30 s C-code compilation on first run |
| External dependencies | numpy, pyserial, matplotlib, vicon_tracker | (same) | + CasADi + acados |
| Tunable gains | 4 PD/PID gains (CLI-tunable, no recompile) | 4 cost weights + temperature (source) | 5 cost weights (source; recompile required) |

---

## Hardware setup (Vicon + car)

Identical to the MPPI / MPC procedure. Skip this section entirely if you only intend to run in `--simulation` mode.

### 1. Power on and connect to the Vicon system

- Connect to NETGEAR74 with associated password (ask Maneesh or Mark)
- Turn on the three Vicon boxes
- Log in to the Vicon PC and open Vicon Tracker 4.4
- Wait for Vicon camera rings to turn blue

### 2. Create or verify the Vicon subject

- Make sure the car is inside the Vicon field with the tracking markers attached and oriented correctly
- Ensure that the car shows up on the Vicon tracking screen labeled as UGV
    - If it is not called UGV, you will need to change the associated label before running the test scripts

### 3. Connect the car

- Plug the car's battery in and ensure that the SiK radios are connected
- The light will stay solid blue if the radios are connected

### 4. Verify Vicon tracking before driving

```bash
python3 -c "
import vicon_tracker, time
v = vicon_tracker.vicon()
v.open('UGV@192.168.11.2')
for _ in range(10):
    x_v, R_vm = v.loop()
    print('pos:', x_v)
    time.sleep(0.1)
v.close()
"
```

---

## Software dependencies

Strictly a subset of MPPI's. **No acados, no CasADi, no compilation step.**

```bash
sudo apt-get update
sudo apt-get install -y python3-dev python3-pip
pip3 install numpy pyserial matplotlib
```

The `vicon_tracker` Python module is only required for live hardware runs (see the MPPI README for the install procedure). It is **not** imported in `--simulation` mode.

---

## Usage

```
python3 pid_racing.py [options]
```

| Argument | Default | Description |
|---|---|---|
| `--raceline PATH` | `raceline.csv` | Path to raceline CSV |
| `--port PORT` | `/dev/ttyUSB0` | Serial port to car |
| `--laps N` | `3` | Laps before stopping; `0` = run forever |
| `--yaw-correction F` | `0.0` | Yaw offset added to Vicon heading (rad) |
| `--speed-gain F` | `20.0` | Feedforward throttle gain (`throttle_ff = gain × v_ref`) |
| `--speed-kp F` | `5.0` | Proportional gain on speed error |
| `--max-throttle N` | `200` | Maximum throttle command (hard cap) |
| `--subject NAME` | `UGV` | Vicon subject name |
| `--server IP` | `192.168.11.2` | Vicon server IP |
| `--simulation` | (flag) | Run without Vicon or radio — simulate the bicycle model in-process |
| `--sim-v0 F` | `0.0` | Initial speed (m/s) when running with `--simulation` |
| `--k-heading F` | `0.6` | Heading-error P gain (rad/rad) |
| `--k-cte F` | `0.6` | Cross-track P gain (rad/m) |
| `--k-cte-d F` | `0.05` | Cross-track D gain (rad·s/m) |
| `--k-cte-i F` | `0.0` | Cross-track I gain (rad/(m·s)) |
| `--i-max F` | `0.10` | Anti-windup cap on integral contribution (rad) |

The four `--k-*` gains and `--i-max` are CLI-tunable specifically because PID is the controller you'll iterate on most quickly — no recompile, no warm-start to invalidate.

### Examples

```bash
# Simulator sweep — try a tighter CTE gain without editing source
python3 pid_racing.py --simulation --raceline figure_eight.csv \
    --k-heading 0.6 --k-cte 0.9 --k-cte-d 0.05 --laps 3

# Live run at low speed
python3 pid_racing.py --laps 1 --max-throttle 80
```

---

## Simulation mode

Setting `--simulation` skips the Vicon connection and serial port, and instead integrates the kinematic bicycle model in the main loop using the PID's own output as the applied control. The live dashboard (CTE, heading error, velocity error, track view, **per-term steering breakdown**, steering indicator) is identical to the hardware view.

The steering breakdown panel — green `P_heading`, blue `P_cte`, red `D_cte`, magenta `I_cte`, black `δ_total` — is the most useful diagnostic for tuning PID. The size and sign of each term tells you immediately which loop is doing the work and which one is fighting.

```bash
python3 pid_racing.py --simulation --raceline figure_eight.csv --laps 3
```

---

## Control law

```
δ = − ( K_h · ψ_e  +  K_e · e  +  K_ė · ė  +  K_∫ · ∫e dt )
```

with sign convention (all gains positive):

- `ψ_e = state.psi − ref_psi`     positive → car pointing too far CCW → steer CW
- `e   = signed CTE`               positive → car is left of track     → steer right
- `ė   = (e − e_prev) / dt`        positive → diverging leftward       → steer right

The **signed CTE** is the dot product of the car-to-reference offset vector with the raceline's local left-normal:

```
e = −sin(ref_psi) · (x − ref_x) + cos(ref_psi) · (y − ref_y)
```

Positive `e` means the car is to the left of the track at that arc position. This is the critical difference from MPPI's and MPC's unsigned `‖p − p_ref‖`: PID needs the sign to know *which way* to correct.

### Anti-windup

- The integral is clamped so that the I-term cannot contribute more than `I_MAX` to the steering command.
- The integral is **frozen** while the total steering output is saturated at `±DELTA_MAX`. This is the "clamping" anti-windup scheme — keeps the integrator from accumulating during a sustained corner that has already saturated the steering.

---

## First-time tuning (live hardware)

Tune the gains in this order so each step isolates a single closed-loop effect.

**Step 1 — cap the throttle**

Start with `--max-throttle 80` (40 % of the hardware maximum of 200). This keeps the car slow enough to catch mistakes.

**Step 2 — calibrate the speed controller**

Identical to MPPI/MPC — `speed_gain × v_ref + speed_kp × (v_ref − v_est)`. Find the throttle that holds a steady low speed on a straight, divide by that speed, set `--speed-gain` accordingly.

**Step 3 — tune yaw correction**

Same as MPPI/MPC — `--yaw-correction` compensates for the Vicon-to-car yaw offset. Adjust by ±0.05 rad if the car consistently steers off-line from the very first iteration.

**Step 4 — tune `K_h` alone**

Set `--k-cte 0`, `--k-cte-d 0`, `--k-cte-i 0`. Drive a straight section and increase `K_h` from a low starting value (~0.3) until the heading error settles below ~0.1 rad without oscillation. Watch the green `P_heading` line on the dashboard — it should be smooth and bounded.

**Step 5 — add `K_e`**

Re-enable cross-track tracking with `--k-cte 0.5` (or thereabouts). Expect to re-tune `K_h` slightly: the two terms partially compensate, so the well-tuned `K_h` from step 4 may need to drop by 10–20 %.

**Step 6 — add `K_ė` if oscillatory**

If the closed loop is now oscillatory but not divergent, add `K_ė ≈ 0.05`. Above roughly `K_ė ≈ K_e / 2` the derivative term mostly amplifies sensor noise at 40 Hz — back off if the steering indicator chatters even when the car is on the line.

**Step 7 — add `K_∫` only if persistent one-sided drift**

The default is `0`. On a symmetric track like the figure-8 the steady-state offset of a well-tuned PD is negligible. Add `K_∫ ≈ 0.005` only if you see a consistent same-side error after many laps — typically the sign of chassis misalignment rather than a controller bug.

**Step 8 — raise the speed**

Once the controller tracks cleanly at low speed, raise `--max-throttle` in steps of 20. The effective `K_e` gain scales as `1/v` under Stanley's analysis, so you may need to lower `K_e` proportionally as you go faster (e.g. halve it when doubling speed).

---

## Lap counting note

Identical to MPPI/MPC: the lap counter detects when the closest raceline index wraps from the final 10 % back to the first 10 %. Start the car in the first 90 % of the raceline to avoid an incorrect first lap.

---

## Emergency stop

The script always sends a zero-throttle, centred-steering packet in its `finally` block — on normal exit, `Ctrl-C`, or any unhandled exception. Keep a finger on `Ctrl-C` during the first run.
