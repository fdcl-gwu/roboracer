# RoboRacer MPPI Racing

Runs MPPI on a physical RoboRacer car using Vicon for localisation and a pre-computed raceline CSV.

---

## Full setup procedure (start from scratch)

### 1. Install system dependencies

```bash
sudo apt-get update
sudo apt-get install -y cmake g++ python3-dev python3-pip libeigen3-dev git
pip3 install pybind11 numpy pyserial
```

### 2. Build and install the Vicon tracker library

The library is a C++/pybind11 wrapper around VRPN. VRPN is bundled inside the repo.

```bash
# Clone the mac branch (Ubuntu-compatible despite the branch name)
git clone -b mac https://github.com/ManeeshW/vicon.git
cd vicon

# Build the C++ static library
mkdir build && cd build
cmake ..
make
cd ..

# Build and install the Python extension module
pip3 install .
cd ..
```

After this, `import vicon_tracker` will work from any Python environment.

### 3. Clone this repository

```bash
git clone <this-repo-url>
cd roboracer
```

### 4. Power on and connect to the Vicon system

- Connect to NETGEAR74 with associated password (ask Maneesh or Mark)
- Turn on the three Vicon boxes
- Log in to the Vicon PC and open Vicon Tracker 4.4
- Wait for Vicon camera rings to turn blue


### 5. Create or verify the Vicon subject

- Make sure the car is inside the Vicon field with the tracking markers attached and oriented correctly
- Ensure that the car shows up on the Vicon tracking screen labeled as UGV
    - If it is not called UGV, you will need to change the associated label before running the test scripts

### 6. Connect the car

- Plug the car's battery in and ensure that the SiK radios are connected
- The light will stay solid blue if the radios are connected

### 7. Verify Vicon tracking before driving

Run the Vicon test script (or `test_vicon.py` from the vicon repo) to confirm position and heading data are being received before sending any commands to the car:

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

### 8. Run the controller

```bash
python3 mppi_racing.py --laps 1 --max-throttle 80
```

See [First-time tuning](#first-time-tuning) before increasing speed.

---

## Usage

```
python3 mppi_racing.py [options]
```

| Argument | Default | Description |
|---|---|---|
| `--raceline PATH` | `raceline.csv` | Path to raceline CSV |
| `--port PORT` | `/dev/ttyUSB0` | Serial port to car |
| `--laps N` | `1` | Laps before stopping; `0` = run forever |
| `--yaw-correction F` | `0.3` | Yaw offset added to Vicon heading (rad) |
| `--speed-gain F` | `20.0` | Feedforward throttle gain (`throttle_ff = gain × v_ref`) |
| `--speed-kp F` | `5.0` | Proportional gain on speed error |
| `--max-throttle N` | `200` | Maximum throttle command (hard cap) |
| `--rollouts N` | `300` | MPPI rollout count |
| `--horizon N` | `20` | MPPI horizon steps |
| `--subject NAME` | `UGV` | Vicon subject name |
| `--server IP` | `192.168.10.1` | Vicon server IP |

---

## First-time tuning

**Step 1 — cap the throttle**

Start with `--max-throttle 80` (40 % of the hardware maximum of 200). This keeps the car slow enough to catch mistakes while you verify tracking behaviour.

**Step 2 — run one lap**

```bash
python3 mppi_racing.py --laps 1 --max-throttle 80
```

Watch whether the car follows the raceline. If it consistently undershoots corners, increase `--speed-kp`. If it oscillates, reduce it.

**Step 3 — calibrate the speed controller**

The throttle command is `speed_gain × v_ref + speed_kp × (v_ref − v_est)`. At steady state the proportional term goes to zero, so `speed_gain` is the dominant knob. Find the throttle value that holds a constant low speed on a straight, divide it by that speed, and use the result as `--speed-gain`.

**Step 4 — tune yaw correction**

`--yaw-correction` compensates for a fixed offset between the Vicon frame and the car's forward axis. If the car consistently steers left or right of the raceline from the very start, adjust this value in small increments (±0.05 rad). The default of `0.3` was determined empirically on this platform.

**Step 5 — increase speed**

Once the car tracks cleanly at low speed, raise `--max-throttle` in steps of 20 and re-run. The raceline `v_ref` values top out at 4 m/s; the controller will naturally approach that speed as the ceiling is lifted.

**Step 6 — MPPI performance**

On a Jetson, 300 rollouts × 20 horizon should comfortably fit within the 25 ms control loop. If the loop falls behind (visible as jerky steering), reduce `--rollouts` first (e.g. `--rollouts 150`), then `--horizon`.

---

## Lap counting note

The lap counter detects when the closest raceline index wraps from the final 10 % of the track back to the first 10 %. Start the car in the first 90 % of the raceline to avoid an incorrect lap count on the first wrap-around.

---

## Emergency stop

The script always sends a zero-throttle, centred-steering packet in its `finally` block — on normal exit, `Ctrl-C`, or any unhandled exception. Keep a finger on `Ctrl-C` during the first run.
