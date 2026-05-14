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
    --yaw-correction F    Yaw offset added to Vicon heading (rad) [0.0]
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
 
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
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
MPPI_DT          = 0.025  # prediction step size (s) — must match the main loop period
MPPI_TEMPERATURE = 20.0   # must be same order-of-magnitude as the spread of rollout costs;
MPPI_NOISE       = np.array([0.2, 1.5])  # perturbation std for [delta, a]
 
# Cost weights
W_CTE     = 25.0   # cross-track error
W_HEADING = 15.0   # heading error
W_SPEED   = 0.0   # speed tracking
W_STEER   = 0.0005   # steering rate (penalises delta change, not magnitude)
 
MPPI_MIN_LOOKAHEAD_VEL = 0.8  # m/s — minimum arc speed for reference point spread
 
# Coordinate offset applied to raceline x to align with Vicon frame.
# Also applied to car/trajectory display so the map stays consistent.
RACELINE_X_OFFSET = 0.0
 
 
# ── Velocity estimator EMA factor ────────────────────────────────────────────
VEL_ALPHA = 0.3   # blend fraction for new measurement; lower = smoother
 
 
# ── Live plot settings ────────────────────────────────────────────────────────
PLOT_WINDOW = 200  # rolling sample count for time-series axes
 
 
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

    def indices_at_arc_lengths(self, lengths: np.ndarray) -> np.ndarray:
        """Vectorised version: lengths shape (K,) → indices shape (K,)."""
        wrapped = lengths % self.total_length
        return np.searchsorted(self.arc_lengths, wrapped, side="right") - 1
 
 
# ── Raceline utilities ────────────────────────────────────────────────────────
def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
 
 
def load_raceline(path: str) -> Raceline:
    pts, psi, arcs = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row["x"]) + RACELINE_X_OFFSET, float(row["y"]), float(row["v_ref"])])
            psi.append(float(row["psi"]))
            arcs.append(float(row["s"]))
    points      = np.array(pts,  dtype=float)
    psis        = np.array(psi,  dtype=float)
    arc_lengths = np.array(arcs, dtype=float)
    last_seg    = float(np.hypot(*(points[-1, :2] - points[0, :2])))
    total_length = arc_lengths[-1] + last_seg
    return Raceline(points=points, psis=psis,
                    arc_lengths=arc_lengths, total_length=total_length)
 
 
def find_closest_raceline_point(state: VehicleState, raceline: Raceline,
                                prev_index: int = -1, window: int = 100) -> int:
    n = len(raceline)
    if prev_index < 0:
        # First call: full search to initialise
        indices = range(n)
    else:
        # Windowed search: only consider points near the last known index.
        # This prevents the crossing point of a figure-8 (or any other
        # location where two track segments are physically close) from
        # pulling the tracker onto the wrong branch.
        indices = ((prev_index + d) % n for d in range(-window, window + 1))
 
    best_index, best_dist = 0, float("inf")
    for i in indices:
        dist = math.dist((state.x, state.y), raceline.points[i, :2])
        if dist < best_dist:
            best_dist  = dist
            best_index = i
    return best_index
 
 
def initialize_sequence_from_raceline(
    closest_index: int,
    raceline: Raceline,
    v: float,
    horizon: int,
) -> np.ndarray:
    """Seed the MPPI warm-start using raceline curvature instead of zeros."""
    seq = np.zeros((horizon, 2))
    lookahead_vel = max(v, MPPI_MIN_LOOKAHEAD_VEL)
    s0 = raceline.arc_lengths[closest_index]
    for k in range(horizon):
        idx_a = raceline.index_at_arc_length(s0 + k * lookahead_vel * MPPI_DT)
        idx_b = raceline.index_at_arc_length(s0 + (k + 1) * lookahead_vel * MPPI_DT)
        dpsi = normalize_angle(raceline.psis[idx_b] - raceline.psis[idx_a])
        delta = math.atan2(WHEELBASE * dpsi, MPPI_DT * lookahead_vel)
        seq[k, 0] = float(np.clip(delta, -DELTA_MAX, DELTA_MAX))
    return seq
 
 
# ── MPPI solver ───────────────────────────────────────────────────────────────
def _rollout_sequence(state: VehicleState, sequence: np.ndarray,
                      horizon: int) -> np.ndarray:
    """Simulate the weighted-average control sequence forward from the current state."""
    xs = [state.x]
    ys = [state.y]
    px, py, ppsi, pv = state.x, state.y, state.psi, state.v
    for k in range(horizon):
        d  = float(np.clip(sequence[k, 0], -DELTA_MAX, DELTA_MAX))
        a  = float(np.clip(sequence[k, 1],  MAX_DECEL,  MAX_ACCEL))
        px   += pv * math.cos(ppsi) * MPPI_DT
        py   += pv * math.sin(ppsi) * MPPI_DT
        ppsi  = normalize_angle(ppsi + pv / WHEELBASE * math.tan(d) * MPPI_DT)
        pv    = float(np.clip(pv + a * MPPI_DT, 0.0, MAX_SPEED))
        xs.append(px)
        ys.append(py)
    return np.column_stack([xs, ys])
 
 
def mppi_step(
    state: VehicleState,
    raceline: Raceline,
    closest_index: int,
    working_sequence: np.ndarray,   # (horizon, 2) warm-start from previous step
    n_rollouts: int,
    horizon: int,
    last_delta: float = 0.0,
) -> Tuple[ControlInput, np.ndarray, np.ndarray, float]:
    """Return the optimal first-step control, updated warm-start sequence, planned (x,y) trajectory, and minimum rollout cost."""
 
    ref_vel_at_closest = raceline.points[closest_index, 2]
    lookahead_vel = max(state.v, MPPI_MIN_LOOKAHEAD_VEL)
 
    # Step 1 & 2: sample perturbations around the warm-start sequence
    perturbations = np.random.randn(n_rollouts, horizon, 2) * MPPI_NOISE
    episodes = working_sequence[np.newaxis] + perturbations        # (K, H, 2)
    episodes[:, :, 0] = np.clip(episodes[:, :, 0], -DELTA_MAX, DELTA_MAX)
    episodes[:, :, 1] = np.clip(episodes[:, :, 1],  MAX_DECEL, MAX_ACCEL)
 
    # Step 3: roll out all episodes from the current state
    rollout_states = np.full((n_rollouts, 4),
                             [state.x, state.y, state.psi, state.v])  # (K, 4)
    costs = np.zeros(n_rollouts)
    prev_delta = np.full(n_rollouts, last_delta)

    for k in range(horizon):
        delta = episodes[:, k, 0]
        a     = episodes[:, k, 1]

        # Kinematic bicycle model (vectorised over all rollouts)
        rollout_states[:, 0] += rollout_states[:, 3] * np.cos(rollout_states[:, 2]) * MPPI_DT
        rollout_states[:, 1] += rollout_states[:, 3] * np.sin(rollout_states[:, 2]) * MPPI_DT
        rollout_states[:, 2] += rollout_states[:, 3] / WHEELBASE * np.tan(delta) * MPPI_DT
        rollout_states[:, 2]  = (rollout_states[:, 2] + math.pi) % (2 * math.pi) - math.pi
        rollout_states[:, 3]  = np.clip(rollout_states[:, 3] + a * MPPI_DT, 0.0, MAX_SPEED)

        next_arc  = raceline.arc_lengths[closest_index] + (k + 1) * lookahead_vel * MPPI_DT
        ref_index = raceline.index_at_arc_length(next_arc)
        ref_pos   = raceline.points[ref_index, :2]
        ref_psi   = raceline.psis[ref_index]
        ref_vel   = raceline.points[ref_index, 2]

        cte       = np.hypot(rollout_states[:, 0] - ref_pos[0],
                             rollout_states[:, 1] - ref_pos[1])
        psi_err   = np.abs((rollout_states[:, 2] - ref_psi + math.pi) % (2 * math.pi) - math.pi)
        vel_err   = np.abs(rollout_states[:, 3] - ref_vel)
        delta_dot = delta - prev_delta

        costs += W_CTE * cte + W_HEADING * psi_err + W_SPEED * vel_err + W_STEER * delta_dot**2
        prev_delta = delta
 
    # Steps 4 & 5: importance-weighted mixture
    min_cost = float(costs.min())
    costs -= min_cost
    weights = np.exp(-costs / MPPI_TEMPERATURE)
    weights /= weights.sum()
 
    new_sequence = np.einsum("k,kij->ij", weights, episodes)  # (H, 2)
 
    # Planned trajectory from the optimal weighted-average sequence (before warm-start shift)
    planned_traj = _rollout_sequence(state, new_sequence, horizon)
 
    # Step 6: extract first control, shift sequence for warm-start next iteration
    control = ControlInput(delta=float(new_sequence[0, 0]),
                           a=float(new_sequence[0, 1]))
    new_sequence = np.roll(new_sequence, -1, axis=0)
    new_sequence[-1] = 0.0
 
    return control, new_sequence, planned_traj, min_cost
 
 
# ── Control conversion ────────────────────────────────────────────────────────
def delta_to_steering(delta: float) -> int:
    """Convert MPPI steering angle (rad) → servo command."""
    raw = STEER_CENTER - STEER_GAIN * delta + STEER_TRIM
    return int(np.clip(raw, STEER_MIN, STEER_MAX))
 
 
def compute_throttle(v_ref: float, v_est: float,
                     speed_gain: float, speed_kp: float,
                     max_throttle: int) -> int:
    """Feedforward + proportional speed controller → throttle command."""
    throttle = speed_gain * v_ref + speed_kp * (v_ref - v_est)
    return int(np.clip(throttle, 0, max_throttle))
 
 
# ── Live visualization ────────────────────────────────────────────────────────
class LivePlot:
    def __init__(self, raceline: Raceline):
        plt.ion()
        self.fig = plt.figure("MPPI Racing", figsize=(16, 9))
        gs = gridspec.GridSpec(
            3, 3, figure=self.fig,
            height_ratios=[1, 2.5, 0.35],
            hspace=0.50, wspace=0.35,
        )
 
        self.ax_cte   = self.fig.add_subplot(gs[0, 0])
        self.ax_head  = self.fig.add_subplot(gs[0, 1])
        self.ax_vel   = self.fig.add_subplot(gs[0, 2])
        self.ax_map   = self.fig.add_subplot(gs[1, :2])
        self.ax_cost  = self.fig.add_subplot(gs[1:, 2])
        self.ax_steer = self.fig.add_subplot(gs[2, :2])
 
        self.t_buf    = []
        self.cte_buf  = []
        self.head_buf = []
        self.vel_buf  = []
        self.cost_buf = []
        self.t0 = None
 
        # CTE
        self.ax_cte.set_title("CTE (m)", fontsize=9)
        self.ax_cte.set_xlabel("t (s)", fontsize=8)
        self.ax_cte.tick_params(labelsize=7)
        self.l_cte, = self.ax_cte.plot([], [], "r-", lw=1)
 
        # Heading error
        self.ax_head.set_title("Heading Error (rad)", fontsize=9)
        self.ax_head.set_xlabel("t (s)", fontsize=8)
        self.ax_head.tick_params(labelsize=7)
        self.l_head, = self.ax_head.plot([], [], "g-", lw=1)
 
        # Velocity error
        self.ax_vel.set_title("Velocity Error (m/s)", fontsize=9)
        self.ax_vel.set_xlabel("t (s)", fontsize=8)
        self.ax_vel.tick_params(labelsize=7)
        self.l_vel, = self.ax_vel.plot([], [], "b-", lw=1)
 
        # Map
        self.ax_map.set_title("Track View", fontsize=9)
        self.ax_map.set_aspect("equal", adjustable="datalim")
        self.ax_map.plot(
            raceline.points[:, 0], raceline.points[:, 1],
            "k--", lw=1, alpha=0.45, label="raceline",
        )
        self.l_traj, = self.ax_map.plot([], [], "b-",  lw=2, alpha=0.75, label="planned")
        self.l_car,  = self.ax_map.plot([], [], "ro",  ms=8,             label="car")
        self.ax_map.legend(loc="upper right", fontsize=7)
        self.ax_map.tick_params(labelsize=7)
 
        # Cost
        self.ax_cost.set_title("MPPI Min Cost", fontsize=9)
        self.ax_cost.set_xlabel("t (s)", fontsize=8)
        self.ax_cost.tick_params(labelsize=7)
        self.l_cost, = self.ax_cost.plot([], [], "m-", lw=1)
 
        # Steering indicator — a square marker sliding on a track bar
        self.ax_steer.set_title("Steering Input", fontsize=9)
        self.ax_steer.set_xlim(-DELTA_MAX * 1.15, DELTA_MAX * 1.15)
        self.ax_steer.set_ylim(-0.5, 0.5)
        self.ax_steer.set_yticks([])
        self.ax_steer.set_xlabel("δ (rad)   ◄ left · right ►", fontsize=8)
        self.ax_steer.tick_params(labelsize=7)
        self.ax_steer.axhspan(-0.08, 0.08, color="lightsteelblue", alpha=0.4)
        self.ax_steer.axvline(0,          color="gray", lw=0.8, ls="--")
        self.ax_steer.axvline(-DELTA_MAX, color="red",  lw=0.8, ls=":")
        self.ax_steer.axvline( DELTA_MAX, color="red",  lw=0.8, ls=":")
        self.l_steer, = self.ax_steer.plot([0], [0], "bs", ms=18, zorder=5)
 
        self.fig.canvas.draw()
        plt.pause(0.001)
 
    def update(
        self,
        t_now: float,
        cte: float,
        head_err: float,
        vel_err: float,
        cost: float,
        state: VehicleState,
        planned_traj: np.ndarray,
        delta: float,
    ) -> None:
        if self.t0 is None:
            self.t0 = t_now
        t = t_now - self.t0
 
        self.t_buf.append(t)
        self.cte_buf.append(abs(cte))
        self.head_buf.append(abs(head_err))
        self.vel_buf.append(vel_err)
        self.cost_buf.append(cost)
 
        if len(self.t_buf) > PLOT_WINDOW:
            self.t_buf    = self.t_buf[-PLOT_WINDOW:]
            self.cte_buf  = self.cte_buf[-PLOT_WINDOW:]
            self.head_buf = self.head_buf[-PLOT_WINDOW:]
            self.vel_buf  = self.vel_buf[-PLOT_WINDOW:]
            self.cost_buf = self.cost_buf[-PLOT_WINDOW:]
 
        ta = self.t_buf
 
        self.l_cte.set_data(ta, self.cte_buf)
        self.ax_cte.relim(); self.ax_cte.autoscale_view()
 
        self.l_head.set_data(ta, self.head_buf)
        self.ax_head.relim(); self.ax_head.autoscale_view()
 
        self.l_vel.set_data(ta, self.vel_buf)
        self.ax_vel.relim(); self.ax_vel.autoscale_view()
 
        self.l_cost.set_data(ta, self.cost_buf)
        self.ax_cost.relim(); self.ax_cost.autoscale_view()
 
        self.l_car.set_data([state.x], [state.y])
        self.l_traj.set_data(planned_traj[:, 0], planned_traj[:, 1])
 
        self.l_steer.set_data([delta], [0])
 
        self.fig.canvas.flush_events()
        self.fig.canvas.draw_idle()
 
 
# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="MPPI raceline follower for the RoboRacer physical platform.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--raceline",        default="raceline.csv",   help="Path to raceline CSV")
    p.add_argument("--port",            default="/dev/ttyUSB0",   help="Serial port")
    p.add_argument("--laps",            type=int,   default=3,    help="Laps to complete; 0 = run forever")
    p.add_argument("--yaw-correction",  type=float, default=0.0,  help="Yaw offset added to Vicon heading (rad)")
    p.add_argument("--speed-gain",      type=float, default=20.0, help="Feedforward throttle gain (throttle_ff = gain * v_ref)")
    p.add_argument("--speed-kp",        type=float, default=5.0,  help="Proportional gain on speed error")
    p.add_argument("--max-throttle",    type=int,   default=200,  help="Maximum throttle command [0–2047]")
    p.add_argument("--rollouts",        type=int,   default=300,  help="MPPI rollout count")
    p.add_argument("--horizon",         type=int,   default=20,   help="MPPI horizon steps")
    p.add_argument("--subject",         default="UGV",            help="Vicon subject name")
    p.add_argument("--server",          default="192.168.11.2",   help="Vicon server IP")
    p.add_argument("--simulation",      action="store_true",      help="Run in simulation mode (no Vicon or radio required)")
    p.add_argument("--sim-v0",          type=float, default=0.0,  help="Initial speed in simulation (m/s)")
    return p.parse_args()
 
 
# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()
 
    raceline = load_raceline(args.raceline)
    n_pts = len(raceline)
    print(f"Loaded raceline: {n_pts} points, {raceline.total_length:.2f} m total")
 
    live = LivePlot(raceline)
 
    # ── Hardware handles (real mode only) ─────────────────────────────────────
    vicon = None
    ser   = None
    seq   = 0
 
    # ── Simulation state ──────────────────────────────────────────────────────
    # Initialised at the first raceline point (already in the Vicon/shifted frame).
    sim_x   = float(raceline.points[0, 0])
    sim_y   = float(raceline.points[0, 1])
    sim_psi = float(raceline.psis[0])
    sim_v   = args.sim_v0
    sim_delta = 0.0
    sim_a     = 0.0
    prev_sim_t: float = None  # type: ignore[assignment]
 
    try:
        if not args.simulation:
            object_name = f"{args.subject}@{args.server}"
            vicon = vicon_tracker.vicon()
            vicon.open(object_name)
            print(f"Vicon connected: {object_name}")
 
            ser = serial.Serial(args.port, BAUD_RATE, timeout=0.1)
            time.sleep(0.1)
            ser.reset_input_buffer()
            print(f"Serial open: {args.port} @ {BAUD_RATE}")
        else:
            print("Simulation mode — no Vicon or radio connection.")
 
        working_sequence = np.zeros((args.horizon, 2))
        last_delta = 0.0
 
        # Velocity estimation (real mode only)
        prev_x, prev_y, prev_t = None, None, None
        v_est = 0.0
 
        laps_completed     = 0
        near_end           = False
        prev_closest_index = -1
        last_plot_t        = 0.0
 
        lap_target_str = str(args.laps) if args.laps > 0 else "unlimited"
        mode_str = "SIMULATION" if args.simulation else "MPPI"
        print(f"Running {mode_str}. Target laps: {lap_target_str}. Press Ctrl-C to abort.")
 
        while True:
            t_now = time.time()
 
            # ── State acquisition ─────────────────────────────────────────────
            if args.simulation:
                if prev_sim_t is not None:
                    dt_sim = t_now - prev_sim_t
                    sim_x   += sim_v * math.cos(sim_psi) * dt_sim
                    sim_y   += sim_v * math.sin(sim_psi) * dt_sim
                    sim_psi  = normalize_angle(
                        sim_psi + sim_v / WHEELBASE * math.tan(sim_delta) * dt_sim
                    )
                    sim_v = float(np.clip(sim_v + sim_a * dt_sim, 0.0, MAX_SPEED))
                prev_sim_t = t_now
                v_est = sim_v
                state = VehicleState(x=sim_x, y=sim_y, psi=sim_psi, v=sim_v)
            else:
                x_v, R_vm = vicon.loop()
                x, y, _   = x_v
                if prev_x is not None:
                    dt = t_now - prev_t
                    if dt > 0.0:
                        raw_v = math.hypot(x - prev_x, y - prev_y) / dt
                        v_est = VEL_ALPHA * raw_v + (1.0 - VEL_ALPHA) * v_est
                prev_x, prev_y, prev_t = x, y, t_now
                yaw   = np.arctan2(R_vm[1, 0], R_vm[0, 0]) + args.yaw_correction
                state = VehicleState(x=x, y=y, psi=yaw, v=v_est)
 
            closest_index = find_closest_raceline_point(state, raceline, prev_closest_index)
 
            if prev_closest_index == -1:
                working_sequence = initialize_sequence_from_raceline(
                    closest_index, raceline, state.v, args.horizon
                )
 
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
            control, working_sequence, planned_traj, mppi_cost = mppi_step(
                state, raceline, closest_index, working_sequence,
                args.rollouts, args.horizon, last_delta,
            )
            last_delta = control.delta
 
            v_ref       = float(raceline.points[closest_index, 2])
            cte         = math.hypot(
                state.x - raceline.points[closest_index, 0],
                state.y - raceline.points[closest_index, 1],
            )
            heading_err = normalize_angle(state.psi - raceline.psis[closest_index])
            vel_err     = v_est - v_ref
 
            # ── Command dispatch ──────────────────────────────────────────────
            if args.simulation:
                sim_delta = float(np.clip(control.delta, -DELTA_MAX, DELTA_MAX))
                sim_a     = float(np.clip(control.a,     MAX_DECEL,  MAX_ACCEL))
                print(
                    f"[SIM] x={sim_x:.3f} y={sim_y:.3f} psi={sim_psi:.3f} "
                    f"v={sim_v:.2f} v_ref={v_ref:.2f} "
                    f"lap={laps_completed} idx={closest_index} "
                    f"delta={control.delta:.3f} a={control.a:.3f}"
                )
            else:
                steering = delta_to_steering(control.delta)
                throttle = compute_throttle(v_ref, v_est,
                                            args.speed_gain, args.speed_kp,
                                            args.max_throttle)
                print(
                    f"x={state.x:.3f} y={state.y:.3f} yaw={state.psi:.3f} "
                    f"v_est={v_est:.2f} v_ref={v_ref:.2f} "
                    f"lap={laps_completed} idx={closest_index} "
                    f"delta={control.delta:.3f} throttle={throttle} steer={steering}"
                )
                pkt = build_packet(seq, throttle, steering)
                ser.write(pkt)
                seq += 1
 
            time.sleep(0.025)  # ~40 Hz
 
            if t_now - last_plot_t >= 0.1:  # update visualization at ~10 Hz
                live.update(t_now, cte, heading_err, vel_err, mppi_cost,
                            state, planned_traj, control.delta)
                last_plot_t = t_now
 
    except KeyboardInterrupt:
        print("\nKeyboard interrupt.")
    except Exception as e:
        print(f"Exception: {e}")
        raise
    finally:
        if not args.simulation:
            print("Sending stop command...")
            if ser is not None and ser.is_open:
                stop_pkt = build_packet(seq, 0, STEER_CENTER)
                ser.write(stop_pkt)
                time.sleep(0.05)
                ser.close()
            if vicon is not None:
                vicon.close()
        plt.ioff()
        plt.close("all")
        print("Stopped.")
 
 
if __name__ == "__main__":
    main()
 