#!/usr/bin/env python3
"""
mpc_revised_racing.py — Nonlinear MPC (acados + CasADi, NONLINEAR_LS cost)
raceline follower for the RoboRacer physical platform.

Reads position and heading from a Vicon tracker, solves an MPC OCP over a
raceline CSV using acados + CasADi, and sends steering + throttle commands
over serial.

Usage:
    python mpc_revised_racing.py [--raceline PATH] [--port PORT] [--laps N] [options]

Key options:
    --raceline PATH       Path to raceline CSV  [raceline.csv]
    --port PORT           Serial port           [/dev/ttyUSB0]
    --laps N              Laps before stopping; 0 = run forever  [10]
    --yaw-correction F    Yaw offset added to Vicon heading (rad) [0.0]
    --speed-gain F        Feedforward gain: throttle_ff = speed_gain * v_ref  [20.0]
    --speed-kp F          Proportional gain on speed error  [5.0]
    --max-throttle N      Maximum throttle command  [200]
    --horizon N           MPC prediction horizon steps  [20]
    --mpc-dt F            MPC step size in seconds  [0.025]
    --max-iter N          Maximum QP iterations per solve step  [50]
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
STEER_GAIN   = 750  # servo units per radian of steering angle


# ── Vehicle constants (measured on FDCL RoboRacer) ───────────────────────────
WHEELBASE = 0.3240  # metres, kinematic bicycle model

MAX_SPEED  = 15.0   # m/s  (hard ceiling)
MAX_ACCEL  =  6.15  # m/s²
MAX_DECEL  = -6.15  # m/s²
DELTA_MAX  =  0.44  # rad, max front-wheel steering angle


# ── MPC hyper-parameters ──────────────────────────────────────────────────────
MPC_DT                = 0.025  # prediction step size (s) — must match main loop period
MPC_N                 = 20     # default horizon steps
MPC_MIN_LOOKAHEAD_VEL = 5.0   # m/s — minimum arc speed for reference spreading
MAX_DELTA_RATE        = 9.5   # rad/s — hard bound on steering rate (control input)

# NONLINEAR_LS cost weights (on squared residuals).
# Stage output h(x,u): [px, py, psi, v, delta, delta_dot, a]   dim=7
# Terminal output h_e(x): [px, py, psi, v, delta]               dim=5
W_CTE        =  8.0
W_HEADING    =  1.0   # heading anchors chassis to raceline tangent — kept small to stay numerically stable at startup
W_SPEED      =  0.5   # low: deemphasise speed — the external FF+PI throttle loop handles it
W_DELTA      =  2.5   # pull toward curvature-implied delta_ref (feedforward steering)
W_DELTA_e    =  5.0   # terminal: penalty on residual delta — keeps unwind planned
W_DELTA_RATE =  0.3   # mild rate smoothing
W_ACCEL      =  0.1


# ── Shared comparable cost basis (identical across PID / MPPI / MPC) ─────────
COMP_W_CTE     = 23.0
COMP_W_HEADING = 20.0
COMP_W_SPEED   = 0.5

RACELINE_X_OFFSET = 0.0


# ── Velocity estimator EMA factor ────────────────────────────────────────────
VEL_ALPHA = 0.3


# ── Live plot settings ────────────────────────────────────────────────────────
PLOT_WINDOW = 200


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


# ── MPC solver construction ───────────────────────────────────────────────────
def create_mpc_solver(N: int, dt: float, max_iter: int = 50) -> AcadosOcpSolver:
    """Build and compile the acados OCP solver. First call takes ~30 s for C code generation."""

    # ── Symbolic model ────────────────────────────────────────────────────────
    # Augmented state: [px, py, psi, v, delta]; controls: [delta_dot, a]
    # Adding delta as a state lets the MPC reason about steering rate naturally.
    x_sym = ca.MX.sym('x', 5)
    u_sym = ca.MX.sym('u', 2)
    xdot  = ca.MX.sym('xdot', 5)

    px, py, psi, v, delta = x_sym[0], x_sym[1], x_sym[2], x_sym[3], x_sym[4]
    delta_dot, a          = u_sym[0], u_sym[1]

    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        v / WHEELBASE * ca.tan(delta),
        a,
        delta_dot,
    )

    model             = AcadosModel()
    model.name        = 'roboracer_revised_bicycle'  # distinct from mpc_racing.py
    model.x           = x_sym
    model.u           = u_sym
    model.xdot        = xdot
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    # NONLINEAR_LS cost output functions — attached to model so acados auto-differentiates them.
    # Stage:    h(x,u) = [px, py, psi, v, delta, delta_dot, a]
    # Terminal: h_e(x) = [px, py, psi, v, delta]
    # Including delta in the residual lets the solver receive per-stage feedforward
    # steering references (yref[4] = curvature-implied delta_ref), giving explicit
    # geometric guidance rather than relying solely on CTE/heading pressure.
    model.cost_y_expr   = ca.vertcat(x_sym[0], x_sym[1], x_sym[2], x_sym[3],
                                      x_sym[4], u_sym[0], u_sym[1])
    model.cost_y_expr_e = ca.vertcat(x_sym[0], x_sym[1], x_sym[2], x_sym[3], x_sym[4])

    # ── OCP ───────────────────────────────────────────────────────────────────
    ocp       = AcadosOcp()
    ocp.model = model

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf        = N * dt

    # Cost — NONLINEAR_LS uses model.cost_y_expr; do NOT set Vx/Vu (those are LINEAR_LS fields)
    ocp.cost.cost_type   = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W    = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_DELTA, W_DELTA_RATE, W_ACCEL])
    ocp.cost.W_e  = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_DELTA_e])
    ocp.cost.yref   = np.zeros(7)  # placeholder; overwritten each step
    ocp.cost.yref_e = np.zeros(5)

    # Input constraints: |delta_dot| <= MAX_DELTA_RATE, a in [MAX_DECEL, MAX_ACCEL]
    ocp.constraints.lbu   = np.array([-MAX_DELTA_RATE, MAX_DECEL])
    ocp.constraints.ubu   = np.array([ MAX_DELTA_RATE, MAX_ACCEL])
    ocp.constraints.idxbu = np.array([0, 1])

    # State constraints on v and delta at stages 1..N-1
    ocp.constraints.lbx   = np.array([0.0,       -DELTA_MAX])
    ocp.constraints.ubx   = np.array([MAX_SPEED,  DELTA_MAX])
    ocp.constraints.idxbx = np.array([3, 4])  # indices of v and delta in x

    # Terminal state constraints (stage N) — same indices, same static bounds.
    # The ubx_e upper bound on v is tightened per-solve to v_ref_N via solver.set(N,'ubx',...).
    ocp.constraints.lbx_e   = np.array([0.0,       -DELTA_MAX])
    ocp.constraints.ubx_e   = np.array([MAX_SPEED,  DELTA_MAX])
    ocp.constraints.idxbx_e = np.array([3, 4])

    # Initial state constraint — all 5 states pinned at stage 0 (updated each step)
    ocp.constraints.x0 = np.zeros(5)

    # Solver options
    ocp.solver_options.integrator_type        = 'ERK'
    ocp.solver_options.sim_method_num_stages  = 4  # RK4 (4-stage explicit Runge-Kutta)
    ocp.solver_options.sim_method_num_steps   = 1
    ocp.solver_options.nlp_solver_type        = 'SQP_RTI'
    ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON'
    ocp.solver_options.qp_solver_iter_max    = max_iter
    ocp.solver_options.print_level           = 0

    # Use a distinct output directory to avoid colliding with mpc_racing.py's c_generated_code/
    ocp.code_export_directory = 'c_generated_code_revised'

    solver = AcadosOcpSolver(ocp, json_file='roboracer_mpc_revised.json')

    # Push weight matrices so they can be changed at runtime without recompile
    for k in range(N):
        solver.cost_set(k, 'W', ocp.cost.W)
    solver.cost_set(N, 'W', ocp.cost.W_e)

    return solver


def initialize_solver_from_raceline(
    solver: AcadosOcpSolver,
    state: VehicleState,
    closest_index: int,
    raceline: Raceline,
    N: int,
    dt: float,
    prev_delta: float = 0.0,
) -> None:
    """Seed the MPC warm-start from raceline curvature before the first solve."""
    lookahead_vel = max(state.v, MPC_MIN_LOOKAHEAD_VEL)
    s0 = raceline.arc_lengths[closest_index]

    # Build delta_seq[0..N] from consecutive raceline headings
    delta_seq = []
    for k in range(N + 1):
        idx_a = raceline.index_at_arc_length(s0 + k * lookahead_vel * dt)
        idx_b = raceline.index_at_arc_length(s0 + (k + 1) * lookahead_vel * dt)
        v_k   = max(float(raceline.points[idx_a, 2]), MPC_MIN_LOOKAHEAD_VEL)
        dpsi  = normalize_angle(raceline.psis[idx_b] - raceline.psis[idx_a])
        dk    = math.atan2(WHEELBASE * dpsi, v_k * dt)
        delta_seq.append(float(np.clip(dk, -DELTA_MAX, DELTA_MAX)))

    # Pin stage 0 to the actual current steering angle
    delta_seq[0] = prev_delta
    x0 = np.array([state.x, state.y, state.psi, state.v, prev_delta])
    solver.set(0, 'x', x0)

    psi_unwrapped = state.psi
    for k in range(N):
        idx = raceline.index_at_arc_length(s0 + (k + 1) * lookahead_vel * dt)
        psi_unwrapped = psi_unwrapped + normalize_angle(raceline.psis[idx] - psi_unwrapped)
        x_k1 = np.array([
            raceline.points[idx, 0],
            raceline.points[idx, 1],
            psi_unwrapped,
            float(raceline.points[idx, 2]),
            delta_seq[k + 1],
        ])
        solver.set(k + 1, 'x', x_k1)

        delta_dot_k = float(np.clip(
            (delta_seq[k + 1] - delta_seq[k]) / dt,
            -MAX_DELTA_RATE, MAX_DELTA_RATE,
        ))
        solver.set(k, 'u', np.array([delta_dot_k, 0.0]))


def mpc_step(
    state: VehicleState,
    raceline: Raceline,
    closest_index: int,
    closest_arc: float,
    solver: AcadosOcpSolver,
    N: int,
    mpc_dt: float,
    prev_delta: float = 0.0,
) -> Tuple[ControlInput, np.ndarray, dict]:
    """Run one RTI MPC step. Returns the optimal control, planned (x,y) trajectory, and solver diagnostics."""

    # ── Pin initial state ─────────────────────────────────────────────────────
    x0 = np.array([state.x, state.y, state.psi, state.v, prev_delta])
    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)

    # ── Set per-stage references ──────────────────────────────────────────────
    # Heading is unwrapped continuously to avoid ±π discontinuities on figure-8 tracks.
    # delta_ref is the curvature-implied steering angle at each stage: δ = atan2(L·dψ, v·dt).
    # It acts as a feedforward target so the solver knows what angle to adopt, not just
    # where to be — this dramatically improves convergence from any warm-start.
    lookahead_vel = max(state.v, MPC_MIN_LOOKAHEAD_VEL)
    psi_unwrapped = state.psi
    psi_prev      = state.psi

    for k in range(N):
        arc_k   = closest_arc + (k + 1) * lookahead_vel * mpc_dt
        ref_idx = raceline.index_at_arc_length(arc_k)
        v_ref_k = float(raceline.points[ref_idx, 2])
        psi_unwrapped = psi_unwrapped + normalize_angle(raceline.psis[ref_idx] - psi_unwrapped)

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
            v_ref_k,
            delta_ref,  # curvature-implied steering target
            0.0,        # delta_dot target = 0 (penalise rate)
            0.0,        # a target = 0 (regularise acceleration)
        ]))
        # Hard cap on velocity at stage k+1: clamped to whatever the vehicle could possibly
        # decelerate to from x0 within (k+1) steps, so the constraint is always feasible.
        # Otherwise, when v_actual > v_ref by more than MAX_DECEL·dt, the QP goes infeasible
        # (manifests as HPIPM MINSTEP → acados status 4 → emergency-stop loop).
        v_min_reachable = state.v + MAX_DECEL * (k + 1) * mpc_dt   # MAX_DECEL is negative
        v_cap           = max(v_ref_k, v_min_reachable)
        solver.set(k + 1, 'ubx', np.array([v_cap, DELTA_MAX]))

    arc_e       = closest_arc + N * lookahead_vel * mpc_dt
    ref_e       = raceline.index_at_arc_length(arc_e)
    psi_e       = psi_unwrapped + normalize_angle(raceline.psis[ref_e] - psi_unwrapped)
    dpsi_e      = psi_e - psi_unwrapped
    delta_ref_e = float(np.clip(
        math.atan2(WHEELBASE * dpsi_e, lookahead_vel * mpc_dt),
        -DELTA_MAX, DELTA_MAX,
    ))
    solver.set(N, 'yref', np.array([
        raceline.points[ref_e, 0],
        raceline.points[ref_e, 1],
        psi_e,
        float(raceline.points[ref_e, 2]),
        delta_ref_e,
    ]))

    # ── Solve ─────────────────────────────────────────────────────────────────
    t0 = time.perf_counter()
    status = solver.solve()
    solve_ms = (time.perf_counter() - t0) * 1e3

    if status in (1, 4):  # 1=NaN_DETECTED, 4=QP_FAILURE — hold steering, zero accel
        print(f"[ERROR] acados solver status {status}, applying zero control")
        return (
            ControlInput(delta=prev_delta, a=0.0),
            np.zeros((N + 1, 2)),
            {'status': status, 'solve_ms': solve_ms,
             'qp_iter': 0, 'cost': 0.0, 'primal_res': 0.0},
        )
    elif status not in (0, 2, 3):
        print(f"[WARN] acados solver status {status}")

    # ── Extract results (must happen before the warm-shift below) ─────────────
    # Actuated delta = delta state at stage 1; ERK4-consistent, respects delta constraints.
    delta = float(solver.get(1, 'x')[4])
    accel = float(solver.get(0, 'u')[1])

    planned_traj = np.array([solver.get(k, 'x')[:2] for k in range(N + 1)])

    try:
        cost = float(solver.get_cost())
    except Exception:
        cost = 0.0

    try:
        qp_iter_val = solver.get_stats('qp_iter')
        qp_iter = int(np.sum(qp_iter_val))
    except Exception:
        qp_iter = 0

    try:
        res = solver.get_residuals()
        primal_res = float(res[1])
    except Exception:
        primal_res = 0.0

    # ── Warm-shift primal solution forward by one step ────────────────────────
    for k in range(N - 1):
        solver.set(k, 'x', solver.get(k + 1, 'x'))
        solver.set(k, 'u', solver.get(k + 1, 'u'))
    solver.set(N - 1, 'u', np.zeros(2))

    return (
        ControlInput(delta=delta, a=accel),
        planned_traj,
        {'status': status, 'solve_ms': solve_ms,
         'qp_iter': qp_iter, 'cost': cost, 'primal_res': primal_res},
    )


# ── Control conversion ────────────────────────────────────────────────────────
def delta_to_steering(delta: float) -> int:
    raw = STEER_CENTER - STEER_GAIN * delta + STEER_TRIM
    return int(np.clip(raw, STEER_MIN, STEER_MAX))


def compute_throttle(v_ref: float, v_est: float,
                     speed_gain: float, speed_kp: float,
                     max_throttle: int) -> int:
    throttle = speed_gain * v_ref + speed_kp * (v_ref - v_est)
    return int(np.clip(throttle, 0, max_throttle))


# ── Live visualization ────────────────────────────────────────────────────────
class LivePlot:
    def __init__(self, raceline: Raceline):
        plt.ion()
        self.fig = plt.figure("MPC Revised Racing", figsize=(16, 9))
        gs = gridspec.GridSpec(
            3, 3, figure=self.fig,
            height_ratios=[1, 2.5, 0.35],
            hspace=0.50, wspace=0.35,
        )

        self.ax_cte   = self.fig.add_subplot(gs[0, 0])
        self.ax_head  = self.fig.add_subplot(gs[0, 1])
        self.ax_vel   = self.fig.add_subplot(gs[0, 2])
        self.ax_map   = self.fig.add_subplot(gs[1, :2])
        self.ax_diag  = self.fig.add_subplot(gs[1:, 2])
        self.ax_steer = self.fig.add_subplot(gs[2, :2])

        self.t_buf        = []
        self.cte_buf      = []
        self.head_buf     = []
        self.vel_buf      = []
        self.solve_ms_buf = []
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

        # Solver diagnostics — rolling solve time with 20 ms budget line
        self.ax_diag.set_title("Solver Diagnostics", fontsize=9)
        self.ax_diag.set_xlabel("t (s)", fontsize=8)
        self.ax_diag.set_ylabel("solve time (ms)", fontsize=8)
        self.ax_diag.axhline(20.0, color="red", lw=0.8, ls="--", label="20 ms budget")
        self.ax_diag.legend(fontsize=7)
        self.ax_diag.tick_params(labelsize=7)
        self.l_diag, = self.ax_diag.plot([], [], "m-", lw=1)
        self.diag_text = self.ax_diag.text(
            0.98, 0.95, "", transform=self.ax_diag.transAxes,
            ha="right", va="top", fontsize=7, family="monospace",
        )

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
        state: VehicleState,
        planned_traj: np.ndarray,
        delta: float,
        solver_info: dict,
    ) -> None:
        if self.t0 is None:
            self.t0 = t_now
        t = t_now - self.t0

        self.t_buf.append(t)
        self.cte_buf.append(abs(cte))
        self.head_buf.append(abs(head_err))
        self.vel_buf.append(vel_err)
        self.solve_ms_buf.append(solver_info['solve_ms'])

        if len(self.t_buf) > PLOT_WINDOW:
            self.t_buf        = self.t_buf[-PLOT_WINDOW:]
            self.cte_buf      = self.cte_buf[-PLOT_WINDOW:]
            self.head_buf     = self.head_buf[-PLOT_WINDOW:]
            self.vel_buf      = self.vel_buf[-PLOT_WINDOW:]
            self.solve_ms_buf = self.solve_ms_buf[-PLOT_WINDOW:]

        ta = self.t_buf

        self.l_cte.set_data(ta, self.cte_buf)
        self.ax_cte.relim(); self.ax_cte.autoscale_view()

        self.l_head.set_data(ta, self.head_buf)
        self.ax_head.relim(); self.ax_head.autoscale_view()

        self.l_vel.set_data(ta, self.vel_buf)
        self.ax_vel.relim(); self.ax_vel.autoscale_view()

        self.l_diag.set_data(ta, self.solve_ms_buf)
        self.ax_diag.relim(); self.ax_diag.autoscale_view()
        self.diag_text.set_text(
            f"status={solver_info['status']}  qp_iter={solver_info['qp_iter']}\n"
            f"primal_res={solver_info['primal_res']:.2e}\n"
            f"cost={solver_info['cost']:.1f}"
        )

        self.l_car.set_data([state.x], [state.y])
        self.l_traj.set_data(planned_traj[:, 0], planned_traj[:, 1])

        self.l_steer.set_data([delta], [0])

        self.fig.canvas.flush_events()
        self.fig.canvas.draw_idle()


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Nonlinear MPC (acados + CasADi) raceline follower for the RoboRacer.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--raceline",       default="raceline.csv",   help="Path to raceline CSV")
    p.add_argument("--port",           default="/dev/ttyUSB0",   help="Serial port")
    p.add_argument("--laps",           type=int,   default=10,   help="Laps to complete; 0 = run forever")
    p.add_argument("--yaw-correction", type=float, default=0.0,  help="Yaw offset added to Vicon heading (rad)")
    p.add_argument("--speed-gain",     type=float, default=20.0, help="Feedforward throttle gain (throttle_ff = gain * v_ref)")
    p.add_argument("--speed-kp",       type=float, default=5.0,  help="Proportional gain on speed error")
    p.add_argument("--max-throttle",   type=int,   default=200,  help="Maximum throttle command [0–2047]")
    p.add_argument("--horizon",        type=int,   default=MPC_N, help="MPC prediction horizon steps")
    p.add_argument("--mpc-dt",         type=float, default=MPC_DT, help="MPC step size (s)")
    p.add_argument("--max-iter",       type=int,   default=50,   help="Max QP iterations per solve step")
    p.add_argument("--subject",        default="UGV",            help="Vicon subject name")
    p.add_argument("--server",         default="192.168.11.2",   help="Vicon server IP")
    p.add_argument("--simulation",     action="store_true",      help="Run in simulation mode (no Vicon or radio required)")
    p.add_argument("--sim-v0",         type=float, default=0.0,  help="Initial speed in simulation (m/s)")
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()

    raceline = load_raceline(args.raceline)
    n_pts = len(raceline)
    print(f"Loaded raceline: {n_pts} points, {raceline.total_length:.2f} m total")

    live = LivePlot(raceline)

    print("Building MPC solver (first run ~30 s for C code generation)...")
    solver = create_mpc_solver(args.horizon, args.mpc_dt, args.max_iter)
    print("Solver ready.")

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

        prev_delta         = 0.0
        solver_initialized = False

        # Velocity estimation (real mode only)
        prev_x, prev_y, prev_t = None, None, None
        v_est = 0.0

        laps_completed     = 0
        near_end           = False
        prev_closest_index = -1
        last_plot_t        = 0.0

        lap_target_str = str(args.laps) if args.laps > 0 else "unlimited"
        mode_str = "SIMULATION" if args.simulation else "MPC"
        print(f"Running {mode_str}. Target laps: {lap_target_str}. Press Ctrl-C to abort.")

        loop_period        = args.mpc_dt
        next_loop_deadline = time.time()
        late_warn_count    = 0

        while True:
            t_now = time.time()

            # ── State acquisition ─────────────────────────────────────────────
            if args.simulation:
                if prev_sim_t is not None:
                    dt_sim  = t_now - prev_sim_t
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

            if not solver_initialized:
                initialize_solver_from_raceline(
                    solver, state, closest_index, raceline,
                    args.horizon, args.mpc_dt, prev_delta,
                )
                solver_initialized = True

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

            # ── MPC step ──────────────────────────────────────────────────────
            closest_arc = raceline.arc_lengths[closest_index]
            control, planned_traj, solver_info = mpc_step(
                state, raceline, closest_index, closest_arc,
                solver, args.horizon, args.mpc_dt, prev_delta,
            )
            prev_delta = control.delta

            v_ref       = float(raceline.points[closest_index, 2])
            cte         = math.hypot(
                state.x - raceline.points[closest_index, 0],
                state.y - raceline.points[closest_index, 1],
            )
            heading_err = normalize_angle(state.psi - raceline.psis[closest_index])
            vel_err     = v_est - v_ref

            comparable_cost = (COMP_W_CTE     * cte
                               + COMP_W_HEADING * abs(heading_err)
                               + COMP_W_SPEED   * abs(vel_err))
            total_comp_cost += comparable_cost
            tick_count      += 1

            # ── Command dispatch ──────────────────────────────────────────────
            if args.simulation:
                sim_delta = float(np.clip(control.delta, -DELTA_MAX, DELTA_MAX))
                sim_a     = float(np.clip(control.a,     MAX_DECEL,  MAX_ACCEL))
                print(
                    f"[SIM] x={sim_x:.3f} y={sim_y:.3f} psi={sim_psi:.3f} "
                    f"v={sim_v:.2f} v_ref={v_ref:.2f} "
                    f"lap={laps_completed} idx={closest_index} "
                    f"delta={control.delta:.3f} a={control.a:.3f} "
                    f"solve={solver_info['solve_ms']:.1f}ms"
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
                    f"delta={control.delta:.3f} throttle={throttle} steer={steering} "
                    f"solve={solver_info['solve_ms']:.1f}ms"
                )
                pkt = build_packet(seq, throttle, steering)
                ser.write(pkt)
                seq += 1

            # ── Visualization ─────────────────────────────────────────────────
            if t_now - last_plot_t >= 0.1:  # update at ~10 Hz
                live.update(t_now, cte, heading_err, vel_err,
                            state, planned_traj, control.delta, solver_info)
                last_plot_t = t_now

            # ── Deadline-based loop rate ──────────────────────────────────────
            next_loop_deadline += loop_period
            slack = next_loop_deadline - time.time()
            if slack > 0.0:
                time.sleep(slack)
            else:
                next_loop_deadline = time.time()
                late_warn_count   += 1
                if late_warn_count % 40 == 1:  # at ~40 Hz, prints ~once/second
                    print(f'[WARN] loop overrun by {-slack*1000:.1f} ms (count={late_warn_count})')

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
        if tick_count > 0:
            print(f"Average comparable cost per tick: {total_comp_cost / tick_count:.4f}  "
                  f"({tick_count} ticks)")
        os.makedirs("results", exist_ok=True)
        live.fig.savefig("results/mpc_revised_final_plot.png", dpi=150, bbox_inches="tight")
        print("Saved: results/mpc_revised_final_plot.png")
        plt.ioff()
        plt.close("all")
        print("Stopped.")


if __name__ == "__main__":
    main()
