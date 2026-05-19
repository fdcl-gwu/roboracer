#!/usr/bin/env python3
"""
mpc_racing.py — Nonlinear MPC (acados) raceline follower for the RoboRacer physical platform.

Mirrors mppi_racing.py in structure; the only functional difference is that the
stochastic MPPI rollout / weighted-average step is replaced by a deterministic
NMPC solve (acados SQP-RTI). Everything else — dashboard, simulation mode,
warm-starting from raceline curvature, windowed closest-point search, hardware
interface, lap counter — is identical.

On first run acados generates and compiles C code (~30 s). Subsequent runs reuse
the compiled solver if c_generated_code/ and roboracer_mpc.json are present and
the horizon/dt have not changed.

Usage:
    python mpc_racing.py [--raceline PATH] [--port PORT] [--laps N] [options]

Key options:
    --raceline PATH       Path to raceline CSV  [raceline.csv]
    --port PORT           Serial port           [/dev/ttyUSB0]
    --laps N              Laps before stopping; 0 = run forever  [3]
    --yaw-correction F    Yaw offset added to Vicon heading (rad) [0.0]
    --speed-gain F        Feedforward gain: throttle_ff = speed_gain * v_ref  [20.0]
    --speed-kp F          Proportional gain on speed error  [5.0]
    --max-throttle N      Maximum throttle command  [200]
    --horizon N           MPC horizon steps  [20]
    --mpc-dt F            MPC prediction step size (s) [0.025]
    --subject NAME        Vicon subject name  [UGV]
    --server IP           Vicon server IP  [192.168.11.2]
    --simulation          Run in simulation mode (no Vicon or radio required)
    --sim-v0 F            Initial speed in simulation (m/s)  [0.0]
"""

import argparse
import csv
import math
import os
import struct
import time
from typing import Tuple

import casadi as ca
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import serial
import vicon_tracker
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


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

MAX_SPEED  = 15.0   # m/s  (state upper bound inside MPC)
MAX_ACCEL  =  6.15  # m/s²
MAX_DECEL  = -6.15  # m/s²
DELTA_MAX  =  0.44  # rad, max front-wheel steering angle


# ── MPC hyper-parameters ──────────────────────────────────────────────────────
MPC_DT = 0.025  # prediction step size (s) — must match the main loop period
MPC_N  = 20     # default horizon steps

# Cost weights — LINEAR_LS cost is ||error||²_W on squared residuals.
# Residual stack: [X-err, Y-err, ψ-err, v-err, δ-err, δ_rate, a].  Heading is
# kept *below* CTE so the solver doesn't defend ψ_err at the cost of letting
# the car parallel the raceline without converging.
W_CTE       = 25.0   # position deviation (applied to both x and y residuals)
W_HEADING   =  8.0   # heading error — kept below W_CTE so heading is a result of tracking, not a competing objective
W_SPEED     =  0.5   # speed tracking
W_DELTA     =  0.5   # steering deviation from curvature feedforward — gentle pull onto geometry
W_DELTA_RATE=  1.0   # penalty on steering rate (rad/s) — replaces the post-solve clip with a real cost term
W_ACCEL     =  0.1   # acceleration regularisation

MPC_MIN_LOOKAHEAD_VEL = 2.0  # m/s — minimum arc speed for reference point spread
MAX_DELTA_RATE = 10.0         # rad/s — hard rate bound enforced by the solver as |u[0]| ≤ MAX_DELTA_RATE

# ── Shared comparable cost basis (identical across PID / MPPI / MPC) ─────────
# Used to produce a controller-agnostic performance metric for cross-comparison.
COMP_W_CTE     = 23.0   # weight on Euclidean cross-track error (m)
COMP_W_HEADING = 20.0   # weight on absolute heading error (rad)
COMP_W_SPEED   = 0.5    # weight on absolute speed error (m/s)

# Coordinate offset applied to raceline x to align with Vicon frame.
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
        # Windowed search: prevents crossing-point confusion on figure-8 tracks
        indices = ((prev_index + d) % n for d in range(-window, window + 1))

    best_index, best_dist = 0, float("inf")
    for i in indices:
        dist = math.dist((state.x, state.y), raceline.points[i, :2])
        if dist < best_dist:
            best_dist  = dist
            best_index = i
    return best_index


def initialize_solver_from_raceline(
    solver: AcadosOcpSolver,
    state: VehicleState,
    closest_index: int,
    raceline: Raceline,
    N: int,
    dt: float,
    prev_delta: float = 0.0,
) -> None:
    """
    Seed the MPC's primal warm-start with state/input trajectories derived
    from raceline curvature.  Counterpart to initialize_sequence_from_raceline()
    in mppi_racing.py.  Only needed on the first iteration; acados warm-starts
    from the previous solution thereafter.

    With augmented state, x[k] = [X, Y, ψ, v, δ_guess] where δ_guess is the
    curvature-implied steering angle for that stage; u[k] = [δ_rate, a] is
    seeded with finite-difference rate so the warm start is dynamics-consistent.
    """
    lookahead_vel = max(state.v, MPC_MIN_LOOKAHEAD_VEL)
    s0 = raceline.arc_lengths[closest_index]

    # Pre-compute the curvature-implied δ along the horizon so we can take
    # finite differences for the δ_rate warm-start.
    delta_seq = [prev_delta]
    psi_unwrapped = state.psi
    for k in range(N):
        idx_a = raceline.index_at_arc_length(s0 + k * lookahead_vel * dt)
        idx_b = raceline.index_at_arc_length(s0 + (k + 1) * lookahead_vel * dt)
        dpsi  = normalize_angle(raceline.psis[idx_b] - raceline.psis[idx_a])
        d     = math.atan2(WHEELBASE * dpsi, dt * lookahead_vel)
        delta_seq.append(float(np.clip(d, -DELTA_MAX, DELTA_MAX)))

    solver.set(0, "x", np.array([state.x, state.y, state.psi, state.v, prev_delta]))
    psi_unwrapped = state.psi
    for k in range(N):
        idx_b = raceline.index_at_arc_length(s0 + (k + 1) * lookahead_vel * dt)
        idx_a = raceline.index_at_arc_length(s0 +  k      * lookahead_vel * dt)
        dpsi  = normalize_angle(raceline.psis[idx_b] - raceline.psis[idx_a])

        delta_rate_guess = float(np.clip(
            (delta_seq[k + 1] - delta_seq[k]) / dt,
            -MAX_DELTA_RATE, MAX_DELTA_RATE,
        ))
        solver.set(k, "u", np.array([delta_rate_guess, 0.0]))

        # State guess for the next stage: walk along the raceline, keeping psi continuous
        psi_unwrapped += dpsi
        solver.set(k + 1, "x", np.array([
            raceline.points[idx_b, 0],
            raceline.points[idx_b, 1],
            psi_unwrapped,
            raceline.points[idx_b, 2],
            delta_seq[k + 1],
        ]))


# ── MPC solver construction ───────────────────────────────────────────────────
def create_mpc_solver(N: int, dt: float) -> AcadosOcpSolver:
    """
    Build the acados NMPC solver with the kinematic bicycle model augmented
    with steering angle as a state.  The control input is steering rate,
    which lets the solver enforce |dδ/dt| ≤ MAX_DELTA_RATE as a hard bound
    and penalise (δ_k − δ_{k-1})² directly in the cost — replacing the
    earlier post-solve clip and W_STEER·(δ − prev_δ)² reference trick.

    Model:
      state x = [X, Y, ψ, v, δ]   (5)
      input u = [δ_rate, a]       (2)
      ẋ = [v cos ψ, v sin ψ, v/L tan δ, a, δ_rate]

    Notes:
      - Continuous-time ODEs; acados integrates with ERK (explicit RK4).
      - Velocity and steering both bounded via state-inequality constraints.
      - Steering rate bounded via input-inequality constraints (true rate cap).
      - Angle wrapping is NOT applied inside the model; reference heading is
        unwrapped to stay near state.psi inside mpc_step().
    """
    x_sym = ca.MX.sym('x', 5)   # [X, Y, psi, v, delta]
    u_sym = ca.MX.sym('u', 2)   # [delta_rate, a]
    xdot  = ca.MX.sym('xdot', 5)

    X, Y, psi, v, delta = x_sym[0], x_sym[1], x_sym[2], x_sym[3], x_sym[4]
    delta_rate, a       = u_sym[0], u_sym[1]

    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        v / WHEELBASE * ca.tan(delta),
        a,
        delta_rate,
    )

    model              = AcadosModel()
    model.name         = 'roboracer_bicycle'
    model.x            = x_sym
    model.u            = u_sym
    model.xdot         = xdot
    model.f_expl_expr  = f_expl
    model.f_impl_expr  = xdot - f_expl

    ocp       = AcadosOcp()
    ocp.model = model

    nx   = 5
    nu   = 2
    ny   = nx + nu   # stage residual:    [X, Y, psi, v, delta, delta_rate, a]
    ny_e = nx        # terminal residual: [X, Y, psi, v, delta]

    ocp.dims.N = N

    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    Vx = np.zeros((ny, nx)); Vx[:nx, :] = np.eye(nx)
    Vu = np.zeros((ny, nu)); Vu[nx:, :] = np.eye(nu)

    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.W  = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_DELTA, W_DELTA_RATE, W_ACCEL])
    ocp.cost.yref = np.zeros(ny)

    ocp.cost.Vx_e   = np.eye(nx)
    ocp.cost.W_e    = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_DELTA])
    ocp.cost.yref_e = np.zeros(ny_e)

    # Input bounds: |δ_rate| ≤ MAX_DELTA_RATE, a ∈ [MAX_DECEL, MAX_ACCEL]
    ocp.constraints.lbu   = np.array([-MAX_DELTA_RATE, MAX_DECEL])
    ocp.constraints.ubu   = np.array([ MAX_DELTA_RATE, MAX_ACCEL])
    ocp.constraints.idxbu = np.array([0, 1])

    # State bounds: v ∈ [0, MAX_SPEED], δ ∈ [-DELTA_MAX, DELTA_MAX]
    ocp.constraints.lbx   = np.array([0.0,       -DELTA_MAX])
    ocp.constraints.ubx   = np.array([MAX_SPEED,  DELTA_MAX])
    ocp.constraints.idxbx = np.array([3, 4])

    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.tf              = N * dt
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.print_level     = 0

    solver = AcadosOcpSolver(ocp, json_file='roboracer_mpc.json')

    # Push current module-level weights into the running solver so that weight
    # changes take effect without a full recompile (acados supports runtime
    # cost_set for LINEAR_LS).
    W   = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_DELTA, W_DELTA_RATE, W_ACCEL])
    W_e = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_DELTA])
    for k in range(N):
        solver.cost_set(k, 'W', W)
    solver.cost_set(N, 'W', W_e)

    return solver


# ── MPC step ─────────────────────────────────────────────────────────────────
def mpc_step(
    state: VehicleState,
    raceline: Raceline,
    closest_index: int,
    solver: AcadosOcpSolver,
    N: int,
    mpc_dt: float,
    prev_delta: float = 0.0,
) -> Tuple[ControlInput, np.ndarray, float]:
    """
    Counterpart to mppi_step() in mppi_racing.py.

    Sets the current state and the per-stage references, runs one SQP-RTI
    iteration, returns (control, planned_traj, cost).  No working_sequence is
    threaded through — acados warm-starts internally from the previous solve.

    State: [X, Y, ψ, v, δ] — δ is part of the state and pinned at x0[4] =
    prev_delta.  Control: [δ_rate, a].  The solver enforces |δ_rate| ≤
    MAX_DELTA_RATE as a hard bound and penalises δ_rate² in cost — no
    post-solve rate clip is needed.

    Steering reference: yref[4] is the curvature-implied steering angle
    (kinematic-bicycle inversion of κ = (v/L)·tan δ).  No anchoring to
    prev_delta — the reference tracks geometry, not correction history.
    """
    x0 = np.array([state.x, state.y, state.psi, state.v, prev_delta])
    # Use state.v directly (with a floor) so the horizon extends far enough at
    # speed.  Overspeed registers as CTE since the reference points themselves
    # come from raceline indices spaced by lookahead_vel·dt.
    lookahead_vel = max(state.v, MPC_MIN_LOOKAHEAD_VEL)

    # Pin the initial state (including δ at index 4)
    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)

    # Stage references.  Heading is unwrapped relative to state.psi so the
    # residual stays small across raceline wrap points (figure-8 near index 377).
    # delta_ref is the curvature-implied steering angle for each stage:
    # δ = atan(L·dψ / (v·dt)).
    psi_unwrapped = state.psi
    psi_prev      = state.psi
    delta_ref     = 0.0
    for k in range(N):
        next_arc = raceline.arc_lengths[closest_index] + (k + 1) * lookahead_vel * mpc_dt
        ref_idx  = raceline.index_at_arc_length(next_arc)
        psi_raw  = raceline.psis[ref_idx]
        psi_unwrapped = psi_unwrapped + normalize_angle(psi_raw - psi_unwrapped)

        dpsi      = psi_unwrapped - psi_prev
        delta_ref = float(np.clip(
            math.atan2(WHEELBASE * dpsi, lookahead_vel * mpc_dt),
            -DELTA_MAX, DELTA_MAX,
        ))
        psi_prev = psi_unwrapped

        solver.set(k, 'yref', np.array([
            raceline.points[ref_idx, 0],
            raceline.points[ref_idx, 1],
            psi_unwrapped,
            raceline.points[ref_idx, 2],
            delta_ref,
            0.0,   # δ_rate target — penalise any non-zero rate
            0.0,   # a target
        ]))

    # Terminal reference: include the curvature-implied δ so horizon end is
    # consistent with the upcoming track geometry.
    arc_e         = raceline.arc_lengths[closest_index] + N * lookahead_vel * mpc_dt
    ref_e         = raceline.index_at_arc_length(arc_e)
    psi_e_unwrap  = psi_unwrapped + normalize_angle(raceline.psis[ref_e] - psi_unwrapped)
    delta_ref_e   = float(np.clip(
        math.atan2(WHEELBASE * (psi_e_unwrap - psi_unwrapped), lookahead_vel * mpc_dt),
        -DELTA_MAX, DELTA_MAX,
    ))
    solver.set(N, 'yref', np.array([
        raceline.points[ref_e, 0],
        raceline.points[ref_e, 1],
        psi_e_unwrap,
        raceline.points[ref_e, 2],
        delta_ref_e,
    ]))

    status = solver.solve()
    if status not in (0, 2):   # 0 = success, 2 = max_iter (acceptable for RTI)
        print(f'[WARN] acados solver returned status {status}')

    # Control output: u0 = [δ_rate, a].  The commanded δ is the next stage's
    # state value, which equals prev_delta + δ_rate·dt and respects all bounds
    # the solver enforced.
    u0    = solver.get(0, 'u')
    accel = float(u0[1])
    delta = float(solver.get(1, 'x')[4])
    control = ControlInput(delta=delta, a=accel)

    # Planned (x,y) trajectory for the dashboard — counterpart to MPPI's planned_traj
    planned_traj = np.array([solver.get(k, 'x')[:2] for k in range(N + 1)])

    try:
        cost = float(solver.get_cost())
    except Exception:
        cost = 0.0

    return control, planned_traj, cost


# ── Control conversion ────────────────────────────────────────────────────────
def delta_to_steering(delta: float) -> int:
    """Convert MPC steering angle (rad) → servo command. Same sign convention as mppi_racing."""
    raw = STEER_CENTER - STEER_GAIN * delta + STEER_TRIM
    return int(np.clip(raw, STEER_MIN, STEER_MAX))


def compute_throttle(v_ref: float, v_est: float,
                     speed_gain: float, speed_kp: float,
                     max_throttle: int) -> int:
    """Feedforward + proportional speed controller → throttle command."""
    throttle = speed_gain * v_ref + speed_kp * (v_ref - v_est)
    return int(np.clip(throttle, 0, max_throttle))


# ── Live visualization (mirrors mppi_racing.LivePlot) ────────────────────────
class LivePlot:
    def __init__(self, raceline: Raceline):
        plt.ion()
        self.fig = plt.figure("MPC Racing", figsize=(16, 9))
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

        self.ax_cte.set_title("CTE (m)", fontsize=9)
        self.ax_cte.set_xlabel("t (s)", fontsize=8)
        self.ax_cte.tick_params(labelsize=7)
        self.l_cte, = self.ax_cte.plot([], [], "r-", lw=1)

        self.ax_head.set_title("Heading Error (rad)", fontsize=9)
        self.ax_head.set_xlabel("t (s)", fontsize=8)
        self.ax_head.tick_params(labelsize=7)
        self.l_head, = self.ax_head.plot([], [], "g-", lw=1)

        self.ax_vel.set_title("Velocity Error (m/s)", fontsize=9)
        self.ax_vel.set_xlabel("t (s)", fontsize=8)
        self.ax_vel.tick_params(labelsize=7)
        self.l_vel, = self.ax_vel.plot([], [], "b-", lw=1)

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

        self.ax_cost.set_title("Comparable Cost", fontsize=9)
        self.ax_cost.set_xlabel("t (s)", fontsize=8)
        self.ax_cost.tick_params(labelsize=7)
        self.l_cost, = self.ax_cost.plot([], [], "m-", lw=1)

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
        description='Nonlinear MPC raceline follower for the RoboRacer physical platform.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--raceline',       default='raceline.csv',  help='Path to raceline CSV')
    p.add_argument('--port',           default='/dev/ttyUSB0',  help='Serial port')
    p.add_argument('--laps',           type=int,   default=10,  help='Laps to complete; 0 = run forever')
    p.add_argument('--yaw-correction', type=float, default=0.0, help='Yaw offset added to Vicon heading (rad)')
    p.add_argument('--speed-gain',     type=float, default=20.0,help='Feedforward throttle gain (throttle_ff = gain * v_ref)')
    p.add_argument('--speed-kp',       type=float, default=5.0, help='Proportional gain on speed error')
    p.add_argument('--max-throttle',   type=int,   default=200, help='Maximum throttle command [0–2047]')
    p.add_argument('--horizon',        type=int,   default=MPC_N, help='MPC horizon steps')
    p.add_argument('--mpc-dt',         type=float, default=MPC_DT, help='MPC prediction step size (s)')
    p.add_argument('--subject',        default='UGV',           help='Vicon subject name')
    p.add_argument('--server',         default='192.168.11.2',  help='Vicon server IP')
    p.add_argument('--simulation',     action='store_true',     help='Run in simulation mode (no Vicon or radio required)')
    p.add_argument('--sim-v0',         type=float, default=0.0, help='Initial speed in simulation (m/s)')
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()

    raceline = load_raceline(args.raceline)
    n_pts    = len(raceline)
    print(f'Loaded raceline: {n_pts} points, {raceline.total_length:.2f} m total')

    live = LivePlot(raceline)

    print('Building MPC solver (first run compiles C code — may take ~30 s)...')
    solver = create_mpc_solver(args.horizon, args.mpc_dt)
    print('Solver ready.')

    # ── Hardware handles (real mode only) ─────────────────────────────────────
    vicon = None
    ser   = None
    seq   = 0
    total_comp_cost = 0.0
    tick_count      = 0

    # ── Simulation state ──────────────────────────────────────────────────────
    sim_x   = float(raceline.points[0, 0])
    sim_y   = float(raceline.points[0, 1])
    sim_psi = float(raceline.psis[0])
    sim_v   = args.sim_v0
    sim_delta = 0.0
    sim_a     = 0.0
    prev_sim_t: float = None  # type: ignore[assignment]

    try:
        if not args.simulation:
            object_name = f'{args.subject}@{args.server}'
            vicon = vicon_tracker.vicon()
            vicon.open(object_name)
            print(f'Vicon connected: {object_name}')

            ser = serial.Serial(args.port, BAUD_RATE, timeout=0.1)
            time.sleep(0.1)
            ser.reset_input_buffer()
            print(f'Serial open: {args.port} @ {BAUD_RATE}')
        else:
            print('Simulation mode — no Vicon or radio connection.')

        # Velocity estimation (real mode only)
        prev_x, prev_y, prev_t = None, None, None
        v_est = 0.0

        laps_completed     = 0
        near_end           = False
        prev_closest_index = -1
        last_plot_t        = 0.0
        prev_delta         = 0.0

        lap_target_str = str(args.laps) if args.laps > 0 else 'unlimited'
        mode_str = 'SIMULATION' if args.simulation else 'MPC'
        print(f'Running {mode_str}. Target laps: {lap_target_str}. Press Ctrl-C to abort.')

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
                initialize_solver_from_raceline(
                    solver, state, closest_index, raceline,
                    args.horizon, args.mpc_dt, prev_delta,
                )

            # ── Lap counter ───────────────────────────────────────────────────
            if prev_closest_index != -1 and prev_closest_index > int(n_pts * 0.9):
                near_end = True
            if near_end and closest_index < int(n_pts * 0.1):
                laps_completed += 1
                near_end = False
                print(f'Lap {laps_completed} complete.')
                if args.laps > 0 and laps_completed >= args.laps:
                    print('Target lap count reached.')
                    break
            prev_closest_index = closest_index

            # ── MPC step ──────────────────────────────────────────────────────
            control, planned_traj, mpc_cost = mpc_step(
                state, raceline, closest_index, solver,
                args.horizon, args.mpc_dt, prev_delta,
            )
            prev_delta = control.delta

            v_ref       = float(raceline.points[closest_index, 2])
            cte         = math.hypot(
                state.x - raceline.points[closest_index, 0],
                state.y - raceline.points[closest_index, 1],
            )
            heading_err = normalize_angle(state.psi - raceline.psis[closest_index])
            vel_err     = v_est - v_ref

            comparable_cost = (COMP_W_CTE * cte
                               + COMP_W_HEADING * abs(heading_err)
                               + COMP_W_SPEED   * abs(vel_err))
            total_comp_cost += comparable_cost
            tick_count      += 1

            # ── Command dispatch ──────────────────────────────────────────────
            if args.simulation:
                sim_delta = float(np.clip(control.delta, -DELTA_MAX, DELTA_MAX))
                sim_a     = float(np.clip(control.a,     MAX_DECEL,  MAX_ACCEL))
                print(
                    f'[SIM] x={sim_x:.3f} y={sim_y:.3f} psi={sim_psi:.3f} '
                    f'v={sim_v:.2f} v_ref={v_ref:.2f} '
                    f'lap={laps_completed} idx={closest_index} '
                    f'delta={control.delta:.3f} a={control.a:.3f}'
                )
            else:
                steering = delta_to_steering(control.delta)
                throttle = compute_throttle(v_ref, v_est,
                                            args.speed_gain, args.speed_kp,
                                            args.max_throttle)
                print(
                    f'x={state.x:.3f} y={state.y:.3f} yaw={state.psi:.3f} '
                    f'v_est={v_est:.2f} v_ref={v_ref:.2f} '
                    f'lap={laps_completed} idx={closest_index} '
                    f'delta={control.delta:.3f} throttle={throttle} steer={steering}'
                )
                pkt = build_packet(seq, throttle, steering)
                ser.write(pkt)
                seq += 1

            time.sleep(0.025)  # ~40 Hz — must match MPC_DT

            if t_now - last_plot_t >= 0.1:  # update visualization at ~10 Hz
                live.update(t_now, cte, heading_err, vel_err, comparable_cost,
                            state, planned_traj, control.delta)
                last_plot_t = t_now

    except KeyboardInterrupt:
        print('\nKeyboard interrupt.')
    except Exception as e:
        print(f'Exception: {e}')
        raise
    finally:
        if not args.simulation:
            print('Sending stop command...')
            if ser is not None and ser.is_open:
                stop_pkt = build_packet(seq, 0, STEER_CENTER)
                ser.write(stop_pkt)
                time.sleep(0.05)
                ser.close()
            if vicon is not None:
                vicon.close()
        if tick_count > 0:
            print(f'Average comparable cost per tick: {total_comp_cost / tick_count:.4f}  '
                  f'({tick_count} ticks)')
        os.makedirs('results', exist_ok=True)
        live.fig.savefig('results/mpc_final_plot.png', dpi=150, bbox_inches='tight')
        print('Saved: results/mpc_final_plot.png')
        plt.ioff()
        plt.close('all')
        print('Stopped.')


if __name__ == '__main__':
    main()
