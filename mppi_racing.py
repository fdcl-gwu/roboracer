#!/usr/bin/env python3
"""
mppi_racing.py — MPPI raceline follower for the RoboRacer physical platform.

Reads position and heading from a Vicon tracker, runs MPPI over a raceline CSV,
and sends steering + throttle commands over serial.

Usage:
    python mppi_racing.py [--raceline PATH] [--port PORT] [--laps N] [options]

Key options:
    --raceline PATH       Path to raceline CSV  [raceline.csv]
    --port PORT           Serial port           [/dev/ttyUSB0]
    --laps N              Laps before stopping; 0 = run forever  [3]
    --yaw-correction F    Yaw offset added to Vicon heading (rad) [0.3]
    --speed-gain F        Feedforward gain: throttle_ff = speed_gain * v_ref  [20.0]
    --speed-kp F          Proportional gain on speed error  [5.0]
    --max-throttle N      Maximum throttle command  [200]
    --rollouts N          MPPI rollout count  [300]
    --horizon N           MPPI horizon steps  [20]
    --subject NAME        Vicon subject name  [UGV]
    --server IP           Vicon server IP  [192.168.10.1]
"""

import argparse
import csv
import math
import struct
import time
from typing import Tuple

import numpy as np
import serial
import vicon_tracker


# ── Serial / firmware protocol ────────────────────────────────────────────────
STX         = 0xFE
PAYLOAD_LEN = 0x04
BAUD_RATE   = 230400

STEER_MAX    = 880
STEER_MIN    = 120
STEER_CENTER = 512
STEER_TRIM   = 20
STEER_GAIN   = 1200  # servo units per radian of steering angle


# ── Vehicle constants (measured on FDCL RoboRacer) ───────────────────────────
WHEELBASE = 0.3240  # metres, kinematic bicycle model

MAX_SPEED  = 15.0   # m/s  (hard ceiling inside MPPI)
MAX_ACCEL  =  6.15  # m/s²
MAX_DECEL  = -6.15  # m/s²
DELTA_MAX  =  0.44  # rad, max front-wheel steering angle


# ── MPPI hyper-parameters ─────────────────────────────────────────────────────
MPPI_DT          = 0.05   # prediction step size (s)
MPPI_TEMPERATURE = 0.5    # lower = greedier exploitation of best rollouts
MPPI_NOISE       = np.array([0.15, 1.5])  # perturbation std for [delta, a]

# Cost weights
W_CTE     = 5.0   # cross-track error
W_HEADING = 2.0   # heading error
W_SPEED   = 1.0   # speed tracking
W_STEER   = 5.0   # steering effort (smoothness)


# ── Velocity estimator EMA factor ────────────────────────────────────────────
VEL_ALPHA = 0.3   # blend fraction for new measurement; lower = smoother


# ── CRC-16 / CCITT ───────────────────────────────────────────────────────────
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(seq: int, throttle: int, steering: int) -> bytes:
    seq      &= 0xFF
    throttle  = max(0,   min(throttle, 2047))
    steering  = max(0,   min(steering, 1023))
    payload   = struct.pack('<BBHh', PAYLOAD_LEN, seq, throttle, steering)
    crc       = crc16_ccitt(payload)
    return bytes([STX]) + payload + struct.pack('<H', crc)


# ── Raceline data structures ──────────────────────────────────────────────────
class VehicleState:
    def __init__(self, x: float, y: float, psi: float, v: float):
        self.x   = x
        self.y   = y
        self.psi = psi
        self.v   = v


class ControlInput:
    def __init__(self, delta: float, a: float):
        self.delta = delta
        self.a     = a

    def clamp(self) -> "ControlInput":
        return ControlInput(
            delta=float(np.clip(self.delta, -DELTA_MAX, DELTA_MAX)),
            a=float(np.clip(self.a, MAX_DECEL, MAX_ACCEL)),
        )


class Raceline:
    def __init__(self, points: np.ndarray, psis: np.ndarray,
                 arc_lengths: np.ndarray, total_length: float):
        self.points       = points        # (N, 3): [x, y, v_ref]
        self.psis         = psis          # (N,)
        self.arc_lengths  = arc_lengths   # (N,) cumulative arc distance
        self.total_length = total_length  # full loop length (m)

    def __len__(self) -> int:
        return len(self.points)

    def index_at_arc_length(self, length: float) -> int:
        length_wrapped = length % self.total_length
        return int(np.searchsorted(self.arc_lengths, length_wrapped, side="right") - 1)


# ── Raceline utilities ────────────────────────────────────────────────────────
def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_raceline(path: str) -> Raceline:
    pts, psi, arcs = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row["x"]), float(row["y"]), float(row["v_ref"])])
            psi.append(float(row["psi"]))
            arcs.append(float(row["s"]))
    points      = np.array(pts,  dtype=float)
    psis        = np.array(psi,  dtype=float)
    arc_lengths = np.array(arcs, dtype=float)
    last_seg    = float(np.hypot(*(points[-1, :2] - points[0, :2])))
    total_length = arc_lengths[-1] + last_seg
    return Raceline(points=points, psis=psis,
                    arc_lengths=arc_lengths, total_length=total_length)


def find_closest_raceline_point(state: VehicleState, raceline: Raceline) -> int:
    best_index, best_dist = 0, float("inf")
    for i in range(len(raceline)):
        d = math.dist((state.x, state.y), raceline.points[i, :2])
        if d < best_dist:
            best_dist  = d
            best_index = i
    return best_index


# ── MPPI solver ───────────────────────────────────────────────────────────────
def mppi_step(
    state: VehicleState,
    raceline: Raceline,
    closest_index: int,
    working_sequence: np.ndarray,   # (horizon, 2) warm-start from previous step
    n_rollouts: int,
    horizon: int,
) -> Tuple[ControlInput, np.ndarray]:
    """Return the optimal first-step control and the updated warm-start sequence."""

    ref_vel_at_closest = raceline.points[closest_index, 2]

    # Step 1 & 2: sample perturbations around the warm-start sequence
    perturbations = np.random.randn(n_rollouts, horizon, 2) * MPPI_NOISE
    episodes = working_sequence[np.newaxis] + perturbations        # (K, H, 2)
    episodes[:, :, 0] = np.clip(episodes[:, :, 0], -DELTA_MAX, DELTA_MAX)
    episodes[:, :, 1] = np.clip(episodes[:, :, 1],  MAX_DECEL, MAX_ACCEL)

    # Step 3: roll out all episodes from the current state
    rollout_states = np.full((n_rollouts, 4),
                             [state.x, state.y, state.psi, state.v])  # (K, 4)
    costs = np.zeros(n_rollouts)

    for k in range(horizon):
        delta = episodes[:, k, 0]
        a     = episodes[:, k, 1]

        # Kinematic bicycle model (vectorised over all rollouts)
        rollout_states[:, 0] += rollout_states[:, 3] * np.cos(rollout_states[:, 2]) * MPPI_DT
        rollout_states[:, 1] += rollout_states[:, 3] * np.sin(rollout_states[:, 2]) * MPPI_DT
        rollout_states[:, 2] += rollout_states[:, 3] / WHEELBASE * np.tan(delta) * MPPI_DT
        rollout_states[:, 2]  = (rollout_states[:, 2] + math.pi) % (2 * math.pi) - math.pi
        rollout_states[:, 3]  = np.clip(rollout_states[:, 3] + a * MPPI_DT, 0.0, MAX_SPEED)

        # Reference point at the predicted arc position of this horizon step
        next_arc  = raceline.arc_lengths[closest_index] + (k + 1) * ref_vel_at_closest * MPPI_DT
        ref_index = raceline.index_at_arc_length(next_arc)
        ref_pos   = raceline.points[ref_index, :2]
        ref_psi   = raceline.psis[ref_index]
        ref_vel   = raceline.points[ref_index, 2]

        cte     = np.hypot(rollout_states[:, 0] - ref_pos[0],
                           rollout_states[:, 1] - ref_pos[1])
        psi_err = np.abs((rollout_states[:, 2] - ref_psi + math.pi) % (2 * math.pi) - math.pi)
        vel_err = np.abs(rollout_states[:, 3] - ref_vel)

        costs += W_CTE * cte + W_HEADING * psi_err + W_SPEED * vel_err + W_STEER * delta**2

    # Steps 4 & 5: importance-weighted mixture
    costs -= costs.min()
    weights = np.exp(-costs / MPPI_TEMPERATURE)
    weights /= weights.sum()

    new_sequence = np.einsum("k,kij->ij", weights, episodes)  # (H, 2)

    # Step 6: extract first control, shift sequence for warm-start next iteration
    control = ControlInput(delta=float(new_sequence[0, 0]),
                           a=float(new_sequence[0, 1]))
    new_sequence = np.roll(new_sequence, -1, axis=0)
    new_sequence[-1] = 0.0

    return control, new_sequence


# ── Control conversion ────────────────────────────────────────────────────────
def delta_to_steering(delta: float) -> int:
    """Convert MPPI steering angle (rad) → servo command."""
    raw = STEER_CENTER + STEER_GAIN * delta + STEER_TRIM
    return int(np.clip(raw, STEER_MIN, STEER_MAX))


def compute_throttle(v_ref: float, v_est: float,
                     speed_gain: float, speed_kp: float,
                     max_throttle: int) -> int:
    """Feedforward + proportional speed controller → throttle command."""
    throttle = speed_gain * v_ref + speed_kp * (v_ref - v_est)
    return int(np.clip(throttle, 0, max_throttle))


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="MPPI raceline follower for the RoboRacer physical platform.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--raceline",        default="raceline.csv",   help="Path to raceline CSV")
    p.add_argument("--port",            default="/dev/ttyUSB0",   help="Serial port")
    p.add_argument("--laps",            type=int,   default=3,    help="Laps to complete; 0 = run forever")
    p.add_argument("--yaw-correction",  type=float, default=0.3,  help="Yaw offset added to Vicon heading (rad)")
    p.add_argument("--speed-gain",      type=float, default=20.0, help="Feedforward throttle gain (throttle_ff = gain * v_ref)")
    p.add_argument("--speed-kp",        type=float, default=5.0,  help="Proportional gain on speed error")
    p.add_argument("--max-throttle",    type=int,   default=200,  help="Maximum throttle command [0–2047]")
    p.add_argument("--rollouts",        type=int,   default=300,  help="MPPI rollout count")
    p.add_argument("--horizon",         type=int,   default=20,   help="MPPI horizon steps")
    p.add_argument("--subject",         default="UGV",            help="Vicon subject name")
    p.add_argument("--server",          default="192.168.10.1",   help="Vicon server IP")
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()

    raceline = load_raceline(args.raceline)
    n_pts = len(raceline)
    print(f"Loaded raceline: {n_pts} points, {raceline.total_length:.2f} m total")

    object_name = f"{args.subject}@{args.server}"

    vicon = None
    ser   = None
    seq   = 0

    try:
        vicon = vicon_tracker.vicon()
        vicon.open(object_name)
        print(f"Vicon connected: {object_name}")

        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.1)
        time.sleep(0.1)
        ser.reset_input_buffer()
        print(f"Serial open: {args.port} @ {BAUD_RATE}")

        working_sequence = np.zeros((args.horizon, 2))

        # Velocity estimation state
        prev_x, prev_y, prev_t = None, None, None
        v_est = 0.0

        # Lap counting
        # near_end is set when prev_closest_index was in the final 10% of the track.
        # prev_closest_index is initialised to -1 so that near_end is never armed on
        # the very first frame, regardless of where the car starts.
        # NOTE: if the car starts in the final 10% of the raceline, the first wrap will
        # be counted as a completed lap even though only a partial lap was run.
        # To avoid this, start the car in the first 90% of the raceline.
        laps_completed    = 0
        near_end          = False
        prev_closest_index = -1

        lap_target_str = str(args.laps) if args.laps > 0 else "unlimited"
        print(f"Running MPPI. Target laps: {lap_target_str}. Press Ctrl-C to abort.")

        while True:
            t_now    = time.time()
            x_v, R_vm = vicon.loop()
            x, y, _  = x_v

            # Estimate speed from Vicon position differences (EMA-smoothed)
            if prev_x is not None:
                dt = t_now - prev_t
                if dt > 0.0:
                    raw_v = math.hypot(x - prev_x, y - prev_y) / dt
                    v_est = VEL_ALPHA * raw_v + (1.0 - VEL_ALPHA) * v_est
            prev_x, prev_y, prev_t = x, y, t_now

            yaw   = np.arctan2(R_vm[1, 0], R_vm[0, 0]) + args.yaw_correction
            state = VehicleState(x=x, y=y, psi=yaw, v=v_est)

            closest_index = find_closest_raceline_point(state, raceline)

            # ── Lap counter ───────────────────────────────────────────────────
            if prev_closest_index != -1 and prev_closest_index > int(n_pts * 0.9):
                near_end = True
            if near_end and closest_index < int(n_pts * 0.1):
                laps_completed += 1
                near_end = False
                print(f"Lap {laps_completed} complete.")
                if args.laps > 0 and laps_completed >= args.laps:
                    print("Target lap count reached.")
                    break
            prev_closest_index = closest_index

            # ── MPPI step ─────────────────────────────────────────────────────
            control, working_sequence = mppi_step(
                state, raceline, closest_index, working_sequence,
                args.rollouts, args.horizon,
            )

            v_ref    = float(raceline.points[closest_index, 2])
            steering = delta_to_steering(control.delta)
            throttle = compute_throttle(v_ref, v_est,
                                        args.speed_gain, args.speed_kp,
                                        args.max_throttle)

            print(
                f"x={x:.3f} y={y:.3f} yaw={yaw:.3f} "
                f"v_est={v_est:.2f} v_ref={v_ref:.2f} "
                f"lap={laps_completed} idx={closest_index} "
                f"delta={control.delta:.3f} throttle={throttle} steer={steering}"
            )

            pkt = build_packet(seq, throttle, steering)
            ser.write(pkt)
            seq += 1

            time.sleep(0.025)  # ~40 Hz command rate

    except KeyboardInterrupt:
        print("\nKeyboard interrupt.")
    except Exception as e:
        print(f"Exception: {e}")
        raise
    finally:
        print("Sending stop command...")
        if ser is not None and ser.is_open:
            stop_pkt = build_packet(seq, 0, STEER_CENTER)
            ser.write(stop_pkt)
            time.sleep(0.05)
            ser.close()
        if vicon is not None:
            vicon.close()
        print("Stopped. Port and Vicon connection closed.")


if __name__ == "__main__":
    main()
