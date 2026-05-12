#!/usr/bin/env python3
"""
mpc_racing.py — Nonlinear MPC (acados) raceline follower for the RoboRacer physical platform.

Mirrors mppi_racing.py in structure; the only differences are:
  - mppi_step()         → mpc_step()  (acados NMPC via casadi + SQP-RTI)
  - working_sequence    → solver object (acados warm-starts internally)
  - --rollouts removed  → no stochastic rollouts needed
  - --mpc-dt added      → prediction step size

On first run, acados generates and compiles C code (~30 s). Subsequent runs reuse
the compiled solver if c_generated_code/ and roboracer_mpc.json are present.

Usage:
    python3 mpc_racing.py [--raceline PATH] [--port PORT] [--laps N] [options]
"""

import argparse
import csv
import math
import os
import struct
import time
from typing import Tuple

import casadi as ca
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
WHEELBASE = 0.3240  # metres

MAX_SPEED  = 15.0   # m/s  (state upper bound inside MPC)
MAX_ACCEL  =  6.15  # m/s²
MAX_DECEL  = -6.15  # m/s²
DELTA_MAX  =  0.44  # rad


# ── MPC hyper-parameters ──────────────────────────────────────────────────────
MPC_DT = 0.05  # prediction step size (s)
MPC_N  = 20    # horizon steps

# Cost weights — same semantics as mppi_racing.py
W_CTE     = 5.0   # position deviation (applied to both x and y residuals)
W_HEADING = 2.0   # heading error
W_SPEED   = 1.0   # speed tracking
W_STEER   = 5.0   # steering effort
W_ACCEL   = 0.1   # acceleration regularisation


# ── Velocity estimator EMA factor ────────────────────────────────────────────
VEL_ALPHA = 0.3


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


# ── Raceline data structures (identical to mppi_racing) ──────────────────────
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
        self.points       = points
        self.psis         = psis
        self.arc_lengths  = arc_lengths
        self.total_length = total_length

    def __len__(self) -> int:
        return len(self.points)

    def index_at_arc_length(self, length: float) -> int:
        wrapped = length % self.total_length
        return int(np.searchsorted(self.arc_lengths, wrapped, side='right') - 1)


def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_raceline(path: str) -> Raceline:
    pts, psi, arcs = [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            pts.append([float(row['x']), float(row['y']), float(row['v_ref'])])
            psi.append(float(row['psi']))
            arcs.append(float(row['s']))
    points      = np.array(pts,  dtype=float)
    psis        = np.array(psi,  dtype=float)
    arc_lengths = np.array(arcs, dtype=float)
    last_seg    = float(np.hypot(*(points[-1, :2] - points[0, :2])))
    return Raceline(points=points, psis=psis,
                    arc_lengths=arc_lengths,
                    total_length=arc_lengths[-1] + last_seg)


def find_closest_raceline_point(state: VehicleState, raceline: Raceline) -> int:
    best_index, best_dist = 0, float('inf')
    for i in range(len(raceline)):
        d = math.dist((state.x, state.y), raceline.points[i, :2])
        if d < best_dist:
            best_dist  = d
            best_index = i
    return best_index


# ── MPC solver construction ───────────────────────────────────────────────────
def create_mpc_solver(N: int, dt: float) -> AcadosOcpSolver:
    """
    Build the acados NMPC solver with the kinematic bicycle model.

    Key differences from the MPPI dynamics:
      - Continuous-time ODEs; acados integrates with ERK (explicit RK4).
      - Velocity is bounded via inequality constraints, not clipping in dynamics.
      - Angle wrapping is NOT applied inside the model; reference heading is set
        close enough to the current heading that wrap-around is not an issue.
    """
    # ── CasADi symbolic model ─────────────────────────────────────────────────
    x_sym   = ca.MX.sym('x', 4)   # [X, Y, psi, v]
    u_sym   = ca.MX.sym('u', 2)   # [delta, a]
    xdot    = ca.MX.sym('xdot', 4)

    X, Y, psi, v     = x_sym[0], x_sym[1], x_sym[2], x_sym[3]
    delta, a         = u_sym[0], u_sym[1]

    # Continuous-time kinematic bicycle model
    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        v / WHEELBASE * ca.tan(delta),
        a,
    )

    model          = AcadosModel()
    model.name     = 'roboracer_bicycle'
    model.x        = x_sym
    model.u        = u_sym
    model.xdot     = xdot
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl   # needed if integrator_type is switched to IRK

    # ── OCP definition ────────────────────────────────────────────────────────
    ocp       = AcadosOcp()
    ocp.model = model

    nx = 4; nu = 2
    ny   = nx + nu  # stage residual: [X, Y, psi, v, delta, a]
    ny_e = nx       # terminal residual: [X, Y, psi, v]

    ocp.dims.N = N

    # LINEAR_LS cost:  l(x,u) = ||Vx·x + Vu·u − y_ref||²_W
    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # Vx maps state to the first nx residual components
    Vx = np.zeros((ny, nx))
    Vx[:nx, :] = np.eye(nx)

    # Vu maps inputs to the last nu residual components
    Vu = np.zeros((ny, nu))
    Vu[nx:, :] = np.eye(nu)

    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.W  = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED, W_STEER, W_ACCEL])
    ocp.cost.yref = np.zeros(ny)

    ocp.cost.Vx_e   = np.eye(nx)
    ocp.cost.W_e    = np.diag([W_CTE, W_CTE, W_HEADING, W_SPEED])
    ocp.cost.yref_e = np.zeros(ny_e)

    # Input constraints: steering angle and acceleration limits
    ocp.constraints.lbu   = np.array([-DELTA_MAX, MAX_DECEL])
    ocp.constraints.ubu   = np.array([ DELTA_MAX, MAX_ACCEL])
    ocp.constraints.idxbu = np.array([0, 1])

    # State constraints: velocity only (position is not bounded)
    ocp.constraints.lbx   = np.array([0.0])
    ocp.constraints.ubx   = np.array([MAX_SPEED])
    ocp.constraints.idxbx = np.array([3])

    # Initial state equality constraint (updated at each solve call)
    ocp.constraints.x0 = np.zeros(nx)

    # Solver options: SQP_RTI for real-time iteration (one linearisation per control step)
    ocp.solver_options.tf              = N * dt
    ocp.solver_options.integrator_type = 'ERK'                  # explicit Runge-Kutta
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'              # fast, warm-started
    ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.print_level     = 0

    solver = AcadosOcpSolver(ocp, json_file='roboracer_mpc.json')
    return solver


# ── MPC step ─────────────────────────────────────────────────────────────────
def mpc_step(
    state: VehicleState,
    raceline: Raceline,
    closest_index: int,
    solver: AcadosOcpSolver,
    N: int,
    mpc_dt: float,
) -> ControlInput:
    """
    Set current state and reference trajectory, run one RTI step, return u[0].
    acados warm-starts automatically from the previous call's solution.
    """
    x0 = np.array([state.x, state.y, state.psi, state.v])
    v_ref_at_closest = float(raceline.points[closest_index, 2])

    # Pin initial state
    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)

    # Stage references: project along raceline by arc length
    for k in range(N):
        arc      = raceline.arc_lengths[closest_index] + (k + 0.5) * v_ref_at_closest * mpc_dt
        ref_idx  = raceline.index_at_arc_length(arc)
        solver.set(k, 'yref', np.array([
            raceline.points[ref_idx, 0],  # X_ref
            raceline.points[ref_idx, 1],  # Y_ref
            raceline.psis[ref_idx],        # psi_ref
            raceline.points[ref_idx, 2],  # v_ref
            0.0,                           # delta — penalise deviation from zero effort
            0.0,                           # a
        ]))

    # Terminal reference
    arc_e   = raceline.arc_lengths[closest_index] + N * v_ref_at_closest * mpc_dt
    ref_e   = raceline.index_at_arc_length(arc_e)
    solver.set(N, 'yref', np.array([
        raceline.points[ref_e, 0],
        raceline.points[ref_e, 1],
        raceline.psis[ref_e],
        raceline.points[ref_e, 2],
    ]))

    status = solver.solve()
    if status not in (0, 2):   # 0 = success, 2 = max_iter (acceptable for RTI)
        print(f'[WARN] acados solver returned status {status}')

    u0 = solver.get(0, 'u')
    return ControlInput(delta=float(u0[0]), a=float(u0[1]))


# ── Control conversion (identical to mppi_racing) ────────────────────────────
def delta_to_steering(delta: float) -> int:
    raw = STEER_CENTER + STEER_GAIN * delta + STEER_TRIM
    return int(np.clip(raw, STEER_MIN, STEER_MAX))


def compute_throttle(v_ref: float, v_est: float,
                     speed_gain: float, speed_kp: float,
                     max_throttle: int) -> int:
    throttle = speed_gain * v_ref + speed_kp * (v_ref - v_est)
    return int(np.clip(throttle, 0, max_throttle))


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description='Nonlinear MPC raceline follower for the RoboRacer physical platform.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--raceline',       default='raceline.csv',  help='Path to raceline CSV')
    p.add_argument('--port',           default='/dev/ttyUSB0',  help='Serial port')
    p.add_argument('--laps',           type=int,   default=1,   help='Laps to complete; 0 = run forever')
    p.add_argument('--yaw-correction', type=float, default=0.3, help='Yaw offset added to Vicon heading (rad)')
    p.add_argument('--speed-gain',     type=float, default=20.0,help='Feedforward throttle gain (throttle_ff = gain × v_ref)')
    p.add_argument('--speed-kp',       type=float, default=5.0, help='Proportional gain on speed error')
    p.add_argument('--max-throttle',   type=int,   default=200, help='Maximum throttle command')
    p.add_argument('--horizon',        type=int,   default=20,  help='MPC horizon steps')
    p.add_argument('--mpc-dt',         type=float, default=0.05,help='MPC prediction step size (s)')
    p.add_argument('--subject',        default='UGV',           help='Vicon subject name')
    p.add_argument('--server',         default='192.168.10.1',  help='Vicon server IP')
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()

    raceline = load_raceline(args.raceline)
    n_pts    = len(raceline)
    print(f'Loaded raceline: {n_pts} points, {raceline.total_length:.2f} m total')

    print('Building MPC solver (first run compiles C code — may take ~30 s)...')
    solver = create_mpc_solver(args.horizon, args.mpc_dt)
    print('Solver ready.')

    object_name = f'{args.subject}@{args.server}'
    vicon = None
    ser   = None
    seq   = 0

    try:
        vicon = vicon_tracker.vicon()
        vicon.open(object_name)
        print(f'Vicon connected: {object_name}')

        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.1)
        time.sleep(0.1)
        ser.reset_input_buffer()
        print(f'Serial open: {args.port} @ {BAUD_RATE}')

        prev_x, prev_y, prev_t = None, None, None
        v_est = 0.0

        # Lap counting — see mppi_racing.py for full explanation
        laps_completed     = 0
        near_end           = False
        prev_closest_index = -1

        lap_target_str = str(args.laps) if args.laps > 0 else 'unlimited'
        print(f'Running MPC. Target laps: {lap_target_str}. Press Ctrl-C to abort.')

        while True:
            t_now      = time.time()
            x_v, R_vm  = vicon.loop()
            x, y, _    = x_v

            if prev_x is not None:
                dt = t_now - prev_t
                if dt > 0.0:
                    raw_v = math.hypot(x - prev_x, y - prev_y) / dt
                    v_est = VEL_ALPHA * raw_v + (1.0 - VEL_ALPHA) * v_est
            prev_x, prev_y, prev_t = x, y, t_now

            yaw   = np.arctan2(R_vm[1, 0], R_vm[0, 0]) + args.yaw_correction
            state = VehicleState(x=x, y=y, psi=yaw, v=v_est)

            closest_index = find_closest_raceline_point(state, raceline)

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

            control = mpc_step(state, raceline, closest_index, solver, args.horizon, args.mpc_dt)

            v_ref    = float(raceline.points[closest_index, 2])
            steering = delta_to_steering(control.delta)
            throttle = compute_throttle(v_ref, v_est,
                                        args.speed_gain, args.speed_kp,
                                        args.max_throttle)

            print(
                f'x={x:.3f} y={y:.3f} yaw={yaw:.3f} '
                f'v_est={v_est:.2f} v_ref={v_ref:.2f} '
                f'lap={laps_completed} idx={closest_index} '
                f'delta={control.delta:.3f} throttle={throttle} steer={steering}'
            )

            pkt = build_packet(seq, throttle, steering)
            ser.write(pkt)
            seq += 1

            time.sleep(0.025)  # ~40 Hz command rate

    except KeyboardInterrupt:
        print('\nKeyboard interrupt.')
    except Exception as e:
        print(f'Exception: {e}')
        raise
    finally:
        print('Sending stop command...')
        if ser is not None and ser.is_open:
            stop_pkt = build_packet(seq, 0, STEER_CENTER)
            ser.write(stop_pkt)
            time.sleep(0.05)
            ser.close()
        if vicon is not None:
            vicon.close()
        print('Stopped. Port and Vicon connection closed.')


if __name__ == '__main__':
    main()
