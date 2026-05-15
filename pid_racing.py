#!/usr/bin/env python3
"""
pid_racing.py — PID raceline follower for the RoboRacer physical platform.

Baseline controller for comparison against MPPI (mppi_racing.py).

Steering law:
    δ = −( K_HEADING·ψ_e  +  K_CTE·e  +  K_CTE_D·ė  +  K_CTE_I·∫e dt )

Sign convention (all gains positive):
    ψ_e  = state.psi − ref_psi     positive → car pointing too far CCW → steer CW (−δ)
    e    = signed CTE               positive → car is left of track     → steer right (−δ)
    ė    = de/dt                    positive → diverging leftward        → steer right (−δ)

Analytical starting point — Stanley controller:
    K_HEADING ≈ 1.0            unit gain: heading error maps directly to steering angle
    K_CTE     ≈ k/v  (k≈2)    at v=4.5 m/s → 0.44 rad/m
    K_CTE_D   ≈ 0.05           light damping; too large causes jitter at ~40 Hz
    K_CTE_I   ≈ 0.0            start at zero; add only to correct steady-state drift

Usage:
    python pid_racing.py [--raceline PATH] [--port PORT] [--laps N] [options]

Key options:
    --raceline PATH       Path to raceline CSV  [raceline.csv]
    --port PORT           Serial port           [/dev/ttyUSB0]
    --laps N              Laps before stopping; 0 = run forever  [3]
    --yaw-correction F    Yaw offset added to Vicon heading (rad)  [0.0]
    --speed-gain F        Feedforward gain: throttle_ff = speed_gain * v_ref  [20.0]
    --speed-kp F          Proportional gain on speed error  [5.0]
    --max-throttle N      Maximum throttle command  [200]
    --subject NAME        Vicon subject name  [UGV]
    --server IP           Vicon server IP  [192.168.11.2]
    --k-heading F         Heading-error P gain  [1.0]
    --k-cte F             Cross-track P gain    [0.44]
    --k-cte-d F           Cross-track D gain    [0.05]
    --k-cte-i F           Cross-track I gain    [0.0]
    --i-max F             Integral anti-windup cap (rad)  [0.10]
"""

import argparse
import csv
import math
import struct
import time
from typing import List, Tuple

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

MAX_SPEED  = 15.0   # m/s
MAX_ACCEL  =  6.15  # m/s²
MAX_DECEL  = -6.15  # m/s²
DELTA_MAX  =  0.44  # rad, max front-wheel steering angle


# ── PID steering gains ────────────────────────────────────────────────────────
# Derivation notes (figure-8 track, R≈1.5 m, v=4.5 m/s):
#
#   Required corner steering: δ = arctan(L/R) = arctan(0.324/1.5) ≈ 0.213 rad
#   Required yaw rate:         ψ̇ = v/R = 3.0 rad/s
#
#   K_HEADING = 1.0  →  0.213 rad heading error produces 0.213 rad correction.
#                        At the onset of a corner the heading error builds to
#                        ~ψ̇·τ where τ≈dt=0.025 s → ψ̇·τ ≈ 0.075 rad.
#                        K_HEADING·0.075 = 0.075 rad  (light, adds to K_CTE term)
#
#   K_CTE = 0.44    →  Stanley gain k=2 at v=4.5 m/s: arctan(2·e/4.5) ≈ (2/4.5)·e
#                       At e=0.3 m: correction ≈ 0.13 rad  (meaningful but not saturating)
#
#   K_CTE_D = 0.05  →  At ė=1 m/s: D term = 0.05 rad  (damping without jitter at 40 Hz)
#                       Rule of thumb: K_CTE_D ≈ K_CTE · (loop_period / 4)
#                       = 0.44 · (0.025/4) ≈ 0.003  →  start conservatively at 0.05
#
#   K_CTE_I = 0.0   →  Start at zero.  Add only to fix persistent one-sided drift.

K_HEADING = 0.6    # rad / rad
K_CTE     = 0.6   # rad / m
K_CTE_D   = 0.05   # rad·s / m
K_CTE_I   = 0.0    # rad / (m·s)
I_MAX     = 0.10   # rad  — maximum integral steering contribution (anti-windup cap)


# ── Speed controller (simulation longitudinal axis) ───────────────────────────
# Real hardware uses compute_throttle (feedforward + P).
# Simulation uses a P controller on speed error for the acceleration input.
# K_SPEED_SIM of 3.0 m/s² per m/s reaches v_ref from rest in ~v_ref/3 seconds,
# e.g. 4.5 m/s in 1.5 s — aggressive enough to catch up, gentle enough not to overshoot.
K_SPEED_SIM = 3.0   # (m/s²) / (m/s)


# ── Misc ──────────────────────────────────────────────────────────────────────
PREVIEW_HORIZON   = 20      # steps to forward-simulate for the map preview arc
PREVIEW_DT        = 0.025   # s — matches the main loop period
RACELINE_X_OFFSET = 0.0
VEL_ALPHA         = 0.3     # EMA blend for velocity estimate; lower = smoother
PLOT_WINDOW       = 200     # rolling sample count for time-series axes


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


class Raceline:
    def __init__(self, points: np.ndarray, psis: np.ndarray,
                 arc_lengths: np.ndarray, total_length: float):
        self.points       = points        # (N, 3): [x, y, v_ref]
        self.psis         = psis          # (N,)
        self.arc_lengths  = arc_lengths   # (N,) cumulative arc distance
        self.total_length = total_length

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
                                prev_index: int = -1, window: int = 30) -> int:
    n = len(raceline)
    if prev_index < 0:
        indices = range(n)
    else:
        # Windowed search prevents the figure-8 crossing from pulling onto wrong branch.
        indices = ((prev_index + d) % n for d in range(-window, window + 1))

    best_index, best_dist = 0, float("inf")
    for i in indices:
        dist = math.dist((state.x, state.y), raceline.points[i, :2])
        if dist < best_dist:
            best_dist  = dist
            best_index = i
    return best_index


# ── PID controller ────────────────────────────────────────────────────────────
def pid_step(
    state: VehicleState,
    raceline: Raceline,
    closest_index: int,
    pid_state: List[float],   # [integral (m·s), prev_signed_cte (m)]
    dt: float,
) -> Tuple[ControlInput, List[float], np.ndarray, Tuple[float, float, float, float]]:
    """
    One PID steering step.  Analogous to mppi_step() in mppi_racing.py.

    pid_state   [integral, prev_signed_cte] — mutable controller memory.
    Returns     (control, new_pid_state, preview_traj, (p_h, p_cte, d_cte, i_cte)).
    """
    ref_x   = raceline.points[closest_index, 0]
    ref_y   = raceline.points[closest_index, 1]
    ref_psi = raceline.psis[closest_index]
    ref_vel = raceline.points[closest_index, 2]

    # Signed CTE: dot product of car-offset vector with the left-normal of the track.
    # Positive → car is to the LEFT of the track direction.
    dx = state.x - ref_x
    dy = state.y - ref_y
    signed_cte = -math.sin(ref_psi) * dx + math.cos(ref_psi) * dy

    # Heading error: positive → car pointing too far CCW relative to track.
    psi_err = normalize_angle(state.psi - ref_psi)

    integral, prev_signed_cte = pid_state
    dt = min(max(dt, 1e-6), 0.2)   # guard against first-step spike and stale frames

    cte_dot = (signed_cte - prev_signed_cte) / dt

    # Tentatively advance the integral (clamped to keep the I-term within I_MAX).
    i_limit  = I_MAX / max(K_CTE_I, 1e-9)
    integral_new = float(np.clip(integral + signed_cte * dt, -i_limit, i_limit))

    # Steering contributions (negative sign: left-of-track / CCW → steer right / −δ).
    p_h   = -K_HEADING * psi_err
    p_cte = -K_CTE     * signed_cte
    d_cte = -K_CTE_D   * cte_dot
    i_cte = -K_CTE_I   * integral_new

    raw_delta = p_h + p_cte + d_cte + i_cte

    # Clamping anti-windup: freeze integral accumulation when output is saturated.
    if abs(raw_delta) >= DELTA_MAX:
        i_cte     = -K_CTE_I * integral   # revert to old integral value
        raw_delta = p_h + p_cte + d_cte + i_cte
    else:
        integral = integral_new

    delta = float(np.clip(raw_delta, -DELTA_MAX, DELTA_MAX))

    # Longitudinal: P speed controller for simulation; real hardware uses compute_throttle.
    a = float(np.clip(K_SPEED_SIM * (ref_vel - state.v), MAX_DECEL, MAX_ACCEL))

    control = ControlInput(delta=delta, a=a)

    # Preview trajectory: hold current delta and acceleration for PREVIEW_HORIZON steps.
    xs = [state.x]
    ys = [state.y]
    px, py, ppsi, pv = state.x, state.y, state.psi, state.v
    for _ in range(PREVIEW_HORIZON):
        px   += pv * math.cos(ppsi) * PREVIEW_DT
        py   += pv * math.sin(ppsi) * PREVIEW_DT
        ppsi  = normalize_angle(ppsi + pv / WHEELBASE * math.tan(delta) * PREVIEW_DT)
        pv    = float(np.clip(pv + a * PREVIEW_DT, 0.0, MAX_SPEED))
        xs.append(px)
        ys.append(py)
    preview_traj = np.column_stack([xs, ys])

    new_pid_state = [integral, signed_cte]
    return control, new_pid_state, preview_traj, (p_h, p_cte, d_cte, i_cte)


# ── Control conversion ────────────────────────────────────────────────────────
def delta_to_steering(delta: float) -> int:
    """Convert steering angle (rad) → servo command."""
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
        self.fig = plt.figure("PID Racing", figsize=(16, 9))
        gs = gridspec.GridSpec(
            3, 3, figure=self.fig,
            height_ratios=[1, 2.5, 0.35],
            hspace=0.50, wspace=0.35,
        )

        self.ax_cte   = self.fig.add_subplot(gs[0, 0])
        self.ax_head  = self.fig.add_subplot(gs[0, 1])
        self.ax_vel   = self.fig.add_subplot(gs[0, 2])
        self.ax_map   = self.fig.add_subplot(gs[1, :2])
        self.ax_pid   = self.fig.add_subplot(gs[1:, 2])   # steering breakdown (replaces MPPI cost)
        self.ax_steer = self.fig.add_subplot(gs[2, :2])

        self.t_buf    = []
        self.cte_buf  = []
        self.head_buf = []
        self.vel_buf  = []
        self.ph_buf   = []
        self.pc_buf   = []
        self.dc_buf   = []
        self.ic_buf   = []
        self.d_buf    = []
        self.t0       = None

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
        self.l_traj, = self.ax_map.plot([], [], "b-",  lw=2, alpha=0.75, label="preview")
        self.l_car,  = self.ax_map.plot([], [], "ro",  ms=8,             label="car")
        self.ax_map.legend(loc="upper right", fontsize=7)
        self.ax_map.tick_params(labelsize=7)

        # PID steering breakdown — the most useful tuning view.
        # Green = heading P, Blue = CTE P, Red = CTE D, Magenta = CTE I, Black = total δ.
        # Goal: green and blue should dominate; red should be small and quiet;
        #       magenta should be near zero (only grows if there is persistent drift).
        self.ax_pid.set_title("Steering Breakdown (rad)", fontsize=9)
        self.ax_pid.set_xlabel("t (s)", fontsize=8)
        self.ax_pid.tick_params(labelsize=7)
        self.ax_pid.axhline(0, color="gray", lw=0.5, ls="--")
        self.l_ph,   = self.ax_pid.plot([], [], "g-", lw=1,   label="P heading")
        self.l_pc,   = self.ax_pid.plot([], [], "b-", lw=1,   label="P cte")
        self.l_dc,   = self.ax_pid.plot([], [], "r-", lw=1,   label="D cte")
        self.l_ic,   = self.ax_pid.plot([], [], "m-", lw=1,   label="I cte")
        self.l_dtot, = self.ax_pid.plot([], [], "k-", lw=1.5, label="δ total")
        self.ax_pid.legend(loc="upper right", fontsize=6)

        # Steering indicator
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
        state: VehicleState,
        preview_traj: np.ndarray,
        delta: float,
        pid_terms: Tuple[float, float, float, float],
    ) -> None:
        if self.t0 is None:
            self.t0 = t_now
        t = t_now - self.t0
        p_h, p_cte, d_cte, i_cte = pid_terms

        self.t_buf.append(t)
        self.cte_buf.append(abs(cte))
        self.head_buf.append(abs(head_err))
        self.vel_buf.append(vel_err)
        self.ph_buf.append(p_h)
        self.pc_buf.append(p_cte)
        self.dc_buf.append(d_cte)
        self.ic_buf.append(i_cte)
        self.d_buf.append(delta)

        if len(self.t_buf) > PLOT_WINDOW:
            self.t_buf    = self.t_buf[-PLOT_WINDOW:]
            self.cte_buf  = self.cte_buf[-PLOT_WINDOW:]
            self.head_buf = self.head_buf[-PLOT_WINDOW:]
            self.vel_buf  = self.vel_buf[-PLOT_WINDOW:]
            self.ph_buf   = self.ph_buf[-PLOT_WINDOW:]
            self.pc_buf   = self.pc_buf[-PLOT_WINDOW:]
            self.dc_buf   = self.dc_buf[-PLOT_WINDOW:]
            self.ic_buf   = self.ic_buf[-PLOT_WINDOW:]
            self.d_buf    = self.d_buf[-PLOT_WINDOW:]

        ta = self.t_buf

        self.l_cte.set_data(ta, self.cte_buf)
        self.ax_cte.relim(); self.ax_cte.autoscale_view()

        self.l_head.set_data(ta, self.head_buf)
        self.ax_head.relim(); self.ax_head.autoscale_view()

        self.l_vel.set_data(ta, self.vel_buf)
        self.ax_vel.relim(); self.ax_vel.autoscale_view()

        self.l_car.set_data([state.x], [state.y])
        self.l_traj.set_data(preview_traj[:, 0], preview_traj[:, 1])

        self.l_ph.set_data(ta, self.ph_buf)
        self.l_pc.set_data(ta, self.pc_buf)
        self.l_dc.set_data(ta, self.dc_buf)
        self.l_ic.set_data(ta, self.ic_buf)
        self.l_dtot.set_data(ta, self.d_buf)
        self.ax_pid.relim(); self.ax_pid.autoscale_view()

        self.l_steer.set_data([delta], [0])

        self.fig.canvas.flush_events()
        self.fig.canvas.draw_idle()


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="PID raceline follower — RoboRacer baseline.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--raceline",       default="raceline.csv",  help="Path to raceline CSV")
    p.add_argument("--port",           default="/dev/ttyUSB0",  help="Serial port")
    p.add_argument("--laps",           type=int,   default=3,   help="Laps to complete; 0 = run forever")
    p.add_argument("--yaw-correction", type=float, default=0.0, help="Yaw offset added to Vicon heading (rad)")
    p.add_argument("--speed-gain",     type=float, default=20.0,help="Feedforward throttle gain")
    p.add_argument("--speed-kp",       type=float, default=5.0, help="Proportional gain on speed error")
    p.add_argument("--max-throttle",   type=int,   default=200, help="Maximum throttle command [0–2047]")
    p.add_argument("--subject",        default="UGV",           help="Vicon subject name")
    p.add_argument("--server",         default="192.168.11.2",  help="Vicon server IP")
    p.add_argument("--simulation",     action="store_true",     help="Run in simulation mode (no Vicon or radio)")
    p.add_argument("--sim-v0",         type=float, default=0.0, help="Initial speed in simulation (m/s)")
    # PID gains — exposed as CLI args so you can tune without editing source.
    p.add_argument("--k-heading", type=float, default=K_HEADING, help="P gain on heading error (rad/rad)")
    p.add_argument("--k-cte",     type=float, default=K_CTE,     help="P gain on cross-track error (rad/m)")
    p.add_argument("--k-cte-d",   type=float, default=K_CTE_D,   help="D gain on cross-track error (rad·s/m)")
    p.add_argument("--k-cte-i",   type=float, default=K_CTE_I,   help="I gain on cross-track error (rad/(m·s))")
    p.add_argument("--i-max",     type=float, default=I_MAX,     help="Max integral steering contribution (rad)")
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()

    # Apply CLI-supplied gains to module-level constants used inside pid_step().
    global K_HEADING, K_CTE, K_CTE_D, K_CTE_I, I_MAX
    K_HEADING = args.k_heading
    K_CTE     = args.k_cte
    K_CTE_D   = args.k_cte_d
    K_CTE_I   = args.k_cte_i
    I_MAX     = args.i_max

    raceline = load_raceline(args.raceline)
    n_pts = len(raceline)
    print(f"Loaded raceline: {n_pts} points, {raceline.total_length:.2f} m total")
    print(f"PID gains — K_heading={K_HEADING}  K_cte={K_CTE}  "
          f"K_cte_d={K_CTE_D}  K_cte_i={K_CTE_I}  I_max={I_MAX}")

    live = LivePlot(raceline)

    vicon = None
    ser   = None
    seq   = 0

    # Simulation state — initialised at the first raceline point.
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

        # PID controller memory: [integral (m·s), prev_signed_cte (m)].
        # Analogous to working_sequence in mppi_racing.py.
        pid_state: List[float] = [0.0, 0.0]

        prev_x, prev_y, prev_t = None, None, None
        v_est = 0.0

        laps_completed     = 0
        near_end           = False
        prev_closest_index = -1
        last_plot_t        = 0.0
        last_pid_t         = time.time()

        lap_target_str = str(args.laps) if args.laps > 0 else "unlimited"
        mode_str = "SIMULATION" if args.simulation else "PID"
        print(f"Running {mode_str}. Target laps: {lap_target_str}. Press Ctrl-C to abort.")

        while True:
            t_now  = time.time()
            dt_pid = min(t_now - last_pid_t, 0.2)   # cap first-step spike
            last_pid_t = t_now

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

            # ── PID step ──────────────────────────────────────────────────────
            control, pid_state, preview_traj, pid_terms = pid_step(
                state, raceline, closest_index, pid_state, dt_pid,
            )

            v_ref       = float(raceline.points[closest_index, 2])
            cte         = math.hypot(
                state.x - raceline.points[closest_index, 0],
                state.y - raceline.points[closest_index, 1],
            )
            heading_err = normalize_angle(state.psi - raceline.psis[closest_index])
            vel_err     = v_ref - v_est

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

            if t_now - last_plot_t >= 0.1:
                live.update(t_now, cte, heading_err, vel_err,
                            state, preview_traj, control.delta, pid_terms)
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
