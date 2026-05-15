# RoboRacer MPC Racing

Nonlinear MPC controller for the RoboRacer platform using [acados](https://github.com/acados/acados) and CasADi.
Mirrors `mppi_racing.py` in structure — same Vicon interface, same serial protocol, same lap counting — with the MPPI solver replaced by a deterministic SQP-RTI solver.

---

## How it differs from MPPI

| | `mppi_racing.py` | `mpc_racing.py` |
|---|---|---|
| Algorithm | Stochastic sampling (300 rollouts) | Deterministic NMPC, SQP-RTI |
| Dynamics | Forward Euler + velocity clipping | Continuous ODE integrated with ERK4 |
| Velocity bound | Clipped in state update | Inequality constraint `0 ≤ v ≤ 15 m/s` |
| Warm start | Explicit `working_sequence` array; first iter seeded from raceline curvature | acados warm-starts internally; first iter seeded from raceline curvature |
| Steering effort penalty | Rate `(δ_t − δ_{t−1})²` (no fight against sustained corners) | Absolute magnitude `δ²` (LINEAR_LS limitation) |
| Startup cost | None | ~30 s C-code compilation on first run |
| `--rollouts` | Present | Not applicable, removed |
| `--mpc-dt` | Not present | Prediction step size (s) |

---

## Hardware setup (Vicon + car)

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

Run a quick sanity check to confirm position and heading data are being received before sending any commands to the car:

```bash
python3 -c "
import vicon_tracker, time
v = vicon_tracker.vicon()
v.open('UGV@192.168.10.1')
for _ in range(10):
    x_v, R_vm = v.loop()
    print('pos:', x_v)
    time.sleep(0.1)
v.close()
"
```

---

## Additional dependencies

`mpc_racing.py` needs two libraries beyond what `mppi_racing.py` requires:

- **CasADi** — symbolic math for defining the bicycle model ODE
- **acados** — fast NMPC solver; builds a C library that is called from Python

### 1. Install CasADi

```bash
pip3 install casadi
```

### 2. Build and install acados

acados must be built from source. It pulls in HPIPM (QP solver) as a submodule.

```bash
# System dependencies
sudo apt-get install -y cmake g++ libblas-dev liblapack-dev

# Clone acados
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# Build
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j$(nproc)
cd ../..
```

After building, acados installs headers and shared libraries under `acados/lib/` and `acados/include/`. The Python template needs to know this path.

### 3. Install the acados Python template

```bash
pip3 install acados/interfaces/acados_template
```

### 4. Set the required environment variable

`acados_template` needs `ACADOS_SOURCE_DIR` to find the compiled C library at code-generation time. Add this to your shell profile (e.g. `~/.bashrc`):

```bash
export ACADOS_SOURCE_DIR=/absolute/path/to/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
```

Then reload:

```bash
source ~/.bashrc
```

Verify:

```bash
python3 -c "from acados_template import AcadosOcp; print('acados OK')"
```

---

## Generated files

On the **first run**, `mpc_racing.py` calls acados to generate C code, compile it into a shared library, and store the result. This takes roughly 30 seconds and produces two artefacts in the working directory:

```
roboracer_mpc.json       # serialised OCP definition
c_generated_code/        # generated C solver code and compiled .so
```

On subsequent runs the same compiled solver is reused and startup is fast. If you change any MPC parameters (horizon, dt, cost weights) you must delete `c_generated_code/` and `roboracer_mpc.json` so the solver is regenerated:

```bash
rm -rf c_generated_code/ roboracer_mpc.json
```

---

## Usage

```
python3 mpc_racing.py [options]
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
| `--horizon N` | `20` | MPC horizon steps |
| `--mpc-dt F` | `0.025` | Prediction step size in seconds (horizon time = N × dt) — must match the main loop period |
| `--subject NAME` | `UGV` | Vicon subject name |
| `--server IP` | `192.168.11.2` | Vicon server IP |
| `--simulation` | (flag) | Run without Vicon or radio — simulate the bicycle model in-process |
| `--sim-v0 F` | `0.0` | Initial speed (m/s) when running with `--simulation` |

### Example

```bash
# First run — allow time for solver compilation
python3 mpc_racing.py --laps 1 --max-throttle 80

# Subsequent runs with a different raceline (same horizon/dt — no recompile needed)
python3 mpc_racing.py --raceline fast_line.csv --laps 3 --max-throttle 120
```

---

## First-time tuning

**Step 1 — cap the throttle**

Start with `--max-throttle 80` (40 % of the hardware maximum of 200). This keeps the car slow enough to catch mistakes while you verify tracking behaviour.

**Step 2 — run one lap**

```bash
python3 mpc_racing.py --laps 1 --max-throttle 80
```

Watch whether the car follows the raceline. If it consistently undershoots corners, increase `--speed-kp`. If it oscillates, reduce it.

**Step 3 — calibrate the speed controller**

The throttle command is `speed_gain × v_ref + speed_kp × (v_ref − v_est)`. At steady state the proportional term goes to zero, so `speed_gain` is the dominant knob. Find the throttle value that holds a constant low speed on a straight, divide it by that speed, and use the result as `--speed-gain`.

**Step 4 — tune yaw correction**

`--yaw-correction` compensates for a fixed offset between the Vicon frame and the car's forward axis. If the car consistently steers left or right of the raceline from the very start, adjust this value in small increments (±0.05 rad). The default of `0.3` was determined empirically on this platform.

**Step 5 — increase speed**

Once the car tracks cleanly at low speed, raise `--max-throttle` in steps of 20 and re-run. The raceline `v_ref` values top out at 4 m/s; the controller will naturally approach that speed as the ceiling is lifted.

### MPC-specific parameters

**`--horizon` and `--mpc-dt`**

The total prediction horizon is `N × dt` seconds. The default (20 × 0.025 s = 0.5 s) means the solver looks half a second ahead. At 4 m/s this covers ~2 m of track — enough for the corners on `raceline.csv` and `figure_eight.csv`. For faster speeds or tighter corners, increase `--horizon`. `--mpc-dt` should **stay equal to the main loop period** (`time.sleep(0.025)`); breaking that match introduces a systematic understeer because the model predicts inputs are applied for longer than they actually are.

Note that changing either of these requires deleting the cached solver (see [Generated files](#generated-files)).

**Cost weights** (`W_CTE`, `W_HEADING`, `W_SPEED`, `W_STEER`, `W_ACCEL` in source)

These are not currently exposed as CLI arguments because changing them requires recompiling the solver. Edit them directly at the top of `mpc_racing.py`, then delete `c_generated_code/` and `roboracer_mpc.json` before running.

| Weight | Default | Effect of increasing |
|---|---|---|
| `W_CTE` | 23.0 | Tighter lateral tracking; may increase steering oscillation |
| `W_HEADING` | 20.0 | Faster heading correction; may cause overshoot |
| `W_SPEED` | 0.5 | Closer speed tracking; interacts with throttle P-controller |
| `W_STEER` | 1.0 | Smoother steering. **LINEAR_LS penalises absolute magnitude `δ²`, not rate** (unlike MPPI). Too large will widen the racing line through corners |
| `W_ACCEL` | 0.1 | Penalises aggressive acceleration commands |

Other implementation details worth knowing about (no tuning required, but useful when debugging):

- **Velocity-based lookahead.** The MPC's reference projection along the raceline uses `max(state.v, 0.8 m/s)` rather than `v_ref`. Same rationale as MPPI: prevents the reference from jumping several metres ahead of a stationary car at startup.
- **Reference-heading unwrapping.** The raceline's `psi_ref` values live in `[−π, π]` from `atan2`, but the dynamics model integrates `ψ` without wrapping. Each stage's reference heading is unwrapped to stay near the previous stage's value, eliminating the huge spurious residual that would otherwise appear at the figure-8's `±π` crossing.
- **Raceline-curvature warm-start.** On the first iteration before acados has any previous solve to inherit from, the primal trajectory is seeded with `δ_k = atan2(L · Δψ_k, v · dt)` for each stage and the state guesses are walked along the raceline waypoints with a continuous ψ. Without this, the very first SQP-RTI step can return nonsense.

**Solver status warnings**

The SQP-RTI solver returns a status code each step. Status `0` is a clean solve; status `2` (max iterations) is printed as a warning but is normal during hard cornering — the warm-started solution is still used. Persistent status `2` or other non-zero codes indicate the solver is struggling and the horizon or weights should be adjusted.

**Real-time performance**

On a Jetson, a 20-step horizon with ERK4 integration and HPIPM as the QP solver should solve in well under 10 ms, leaving headroom within the 25 ms control loop. If solve times exceed ~20 ms (printed if you add timing instrumentation), reduce `--horizon` first.

---

## Emergency stop

Identical to `mppi_racing.py`: the `finally` block always sends a zero-throttle, centred-steering packet before closing the serial port, regardless of how the script exits.
