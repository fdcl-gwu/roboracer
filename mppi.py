import math
from typing import Tuple
import pygame
import numpy as np
import csv

# SIMULATION PARAMETERS
SCREEN_WIDTH  = 1200
SCREEN_HEIGHT = 800
FPS           = 60

SCALE = 176 # px per m

SIM_DT = 0.025 # 40hz (NVIDIA Jetson)
 
COLOR_BACKGROUND = (34, 139, 34)       
COLOR_CAR        = (220, 20, 20)    
COLOR_MPC_PATH   = (0, 200, 255)    
COLOR_REF_PATH   = (255, 140, 0)   

RACELINE_CSV = "raceline.csv" #oval

# car parameters measured on FDCL roboracer platform (metric)
CAR_LENGTH = 0.568   # for drawing/collision
CAR_WIDTH = 0.296   
 
WHEELBASE = 0.3240  # for bicycle model

MAX_SPEED    = 15.0  
MAX_ACCEL    = 6.15   
MAX_DECEL    = -6.15  

# steering
DELTA_MAX = 0.4400 

# MPPI PARAMETERS
MPPI_ROLLOUTS = 300  # number of random input sequences that get tested                   
MPPI_HORIZON = 20   # length of each rollout (rollout time = horizon * dt)                   
MPPI_DT = 0.05                         
MPPI_TEMPERATURE = 0.5 # how much the best found sequences matter vs average. higher temp = greedier exploitation  
MPPI_NOISE = np.array([0.15, 1.5]) # [delta, a] used to generate sample input sequences
 
# Cost weights (to tune)
W_CTE = 5.0   # cross-track error
W_HEADING = 2.0   # heading error
W_SPEED = 1.0   # speed tracking
W_STEER = 5.0   # steering effort (smoothness)

class VehicleState:
    def __init__(self, x, y, psi, v):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v

class ControlInput:
    def __init__(self, delta, a):
        self.delta = delta
        self.a = a
 
    def clamp(self) -> "ControlInput":
        return ControlInput(
            delta=np.clip(self.delta, -DELTA_MAX, DELTA_MAX),
            a=np.clip(self.a, MAX_DECEL, MAX_ACCEL),
        )
    
class Raceline:
    def __init__(self, points, psis, arc_lengths, total_length):
        self.points = points
        self.arc_lengths = arc_lengths
        self.psis = psis
        self.total_length = total_length
 
    def __len__(self) -> int:
        return len(self.points)
 
    def index_at_arc_length(self, length: float) -> int:
        length_wrapped = length % self.total_length
        #searchsorted because arc length is cumulative
        return int(np.searchsorted(self.arc_lengths, length_wrapped, side="right") - 1)

def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

# https://stackoverflow.com/questions/3518778/how-do-i-read-csv-data-into-a-record-array-in-numpy
def load_raceline(path: str = RACELINE_CSV) -> Raceline:
    pts, psi, arcs = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row["x"]), float(row["y"]), float(row["v_ref"])])
            psi.append(float(row["psi"]))
            arcs.append(float(row["s"]))
    points = np.array(pts,  dtype=float)
    psis = np.array(psi, dtype=float)
    arc_lengths = np.array(arcs, dtype=float)
    last_seg = float(np.hypot(*(points[-1, :2] - points[0, :2])))
    total_length = arc_lengths[-1] + last_seg
    return Raceline(points=points, psis=psis, arc_lengths=arc_lengths, total_length=total_length)

#slow but good enough
def find_closest_raceline_point(
    state:    VehicleState,
    raceline: Raceline,
) -> int:
    best_index = 0
    best_dist = 999999
    for i in range(len(raceline)):
        pt = raceline.points[i, :2]
        d = math.dist((state.x, state.y), pt)
        if d < best_dist:
            best_dist = d
            best_index = i
    return best_index


# derived from kinematic bicycle model in report
# returns a new state from the previous state and a control input.
def step(
    state: VehicleState,
    control: ControlInput,
    dt: float 
):
    clamped_control: ControlInput = control.clamp()
    return VehicleState(
        x = state.x + (state.v * math.cos(state.psi)) * dt,
        y = state.y + (state.v * math.sin(state.psi)) * dt,
        psi = normalize_angle(state.psi + (state.v / WHEELBASE) * math.tan(clamped_control.delta) * dt),
        v = float(np.clip(state.v + clamped_control.a * dt, 0.0, MAX_SPEED)),
    )

# MPPI 
# Steps:
# 1) Create the MPPI_ROLLOUTS input sequences matrix of shape (MPPI_ROLLOUTS, MPPI_HORIZON, DIM(ControlInput) (2 in this case, delta and a))
# 2) Populate this with perturbations based off of MPPI_TEMPERATURE (defines by how much the input can change each horizon step)
# 3) Perform all of the rollouts based on these perturbations, storing results as a KVP of (ROLLOUT_NUMBER, cost)
#     Cost is derived from W_CTE*cte + W_HEADING*h_err + W_SPEED*v_err + W_STEER*delta^2 (coefficients tunable based on vehicle behavior)
#     Note that the cost is calculated every horizon step instead of just the ending position
#     This avoids sequences that swing away from the racing line and then return to end up in a good position
# 4) Each rollout is assigned an importance weight w_k = exp(-cost / MPPI_TEMPERATURE), and then normalised 
# 5) Sum all sequences * importance weight to find optimal mixture of sequences
# 6) Apply first control in sequence and pass that back as the optimal control for this sim step
# 7) Repeat each frame of simulation, holding on to the previously derived working_sequence

def mppi_step(
    state: VehicleState,
    raceline: Raceline,
    raceline_closest_point_index: int,
    working_sequence: np.ndarray, # current best guess sequence from previous step, allows for warm start of new step (smaller MPPI_NOISE required)
) -> Tuple[ControlInput, np.ndarray, np.ndarray]: #(Cur_frame control input, working_sequence for next step, predicted path for visualization)
    ref_raceline_velocity = raceline.points[raceline_closest_point_index, 2]

    #Steps 1 and 2
    episode_perturbations = np.random.randn(MPPI_ROLLOUTS, MPPI_HORIZON, 2) * MPPI_NOISE
    episodes = working_sequence[np.newaxis] + episode_perturbations
    # enforce steering and accel limitations
    episodes[:, :, 0] = np.clip(episodes[:, :, 0], -DELTA_MAX, DELTA_MAX)
    episodes[:, :, 1] = np.clip(episodes[:, :, 1],  MAX_DECEL, MAX_ACCEL)

    #Step 3
    rollouts = np.full((MPPI_ROLLOUTS, 4), [state.x, state.y, state.psi, state.v]) # set of starting states that will have episodes applied
    costs = np.zeros(MPPI_ROLLOUTS) # combine this into matrix above?

    #https://numpy.org/doc/stable/user/basics.indexing.html
    for k in range(MPPI_HORIZON):
        delta = episodes[:, k, 0]
        a = episodes[:, k, 1]

        # apply kinematic bicycle transform (probably should functionalize)
        rollouts[:, 0] += rollouts[:, 3] * np.cos(rollouts[:, 2]) * MPPI_DT
        rollouts[:, 1] += rollouts[:, 3] * np.sin(rollouts[:, 2]) * MPPI_DT
        rollouts[:, 2] += rollouts[:, 3] / WHEELBASE * np.tan(delta) * MPPI_DT
        rollouts[:, 2] = (rollouts[:, 2] + math.pi) % (2 * math.pi) - math.pi
        rollouts[:, 3] = np.clip(rollouts[:, 3] + a * MPPI_DT, 0.0, MAX_SPEED)

        #calculate cost to add to running tally
        next_arc = raceline.arc_lengths[raceline_closest_point_index] + (k + 1) * ref_raceline_velocity * MPPI_DT
        ref_index = raceline.index_at_arc_length(next_arc)
        ref_pos = raceline.points[ref_index, :2]
        ref_psi = raceline.psis[ref_index]
        ref_vel = raceline.points[ref_index, 2] # different from ref_raceline_velocity, this is for current rollout, not starting point

        cross_track_error = np.hypot(rollouts[:, 0] - ref_pos[0], rollouts[:, 1] - ref_pos[1]) # unsigned, doesn't matter for cost function
        psi_err = np.abs((rollouts[:, 2] - ref_psi + math.pi) % (2 * math.pi) - math.pi)
        vel_err = np.abs(rollouts[:, 3] - ref_vel)

        costs += W_CTE * cross_track_error + W_HEADING * psi_err + W_SPEED * vel_err + W_STEER * delta**2 # penalize harsh steering

    #Step 4
    #turn all costs into a relative value compared to 0
    costs -= costs.min()
    #covnert to importance and normalize
    importance_weights = np.exp(-costs / MPPI_TEMPERATURE)
    importance_weights /= importance_weights.sum()

    #step 5
    working_sequence = np.zeros((MPPI_HORIZON, 2))
    for k in range(MPPI_ROLLOUTS):
        working_sequence += importance_weights[k] * episodes[k] 

    # step 6
    control = ControlInput(delta=float(working_sequence[0, 0]), a=float(working_sequence[0, 1]))
    working_sequence_copy = working_sequence.copy() # fpr visualization in pred
    working_sequence = np.roll(working_sequence, -1, axis=0)
    working_sequence[-1] = 0.0

    # just for visualization
    pred = np.zeros((MPPI_HORIZON, 2))
    sx, sy, spsi, sv = state.x, state.y, state.psi, state.v
    for k in range(MPPI_HORIZON):
        d, a = float(working_sequence_copy[k, 0]), float(working_sequence_copy[k, 1])
        sx   += sv * math.cos(spsi) * MPPI_DT
        sy   += sv * math.sin(spsi) * MPPI_DT
        spsi  = normalize_angle(spsi + sv / WHEELBASE * math.tan(d) * MPPI_DT)
        sv    = float(np.clip(sv + a * MPPI_DT, 0.0, MAX_SPEED))
        pred[k] = [sx, sy]

    return control, working_sequence, pred


def w2s(wx: float, wy: float) -> Tuple[int, int]:
    return int(wx * SCALE + SCREEN_WIDTH // 2), int(-wy * SCALE + SCREEN_HEIGHT // 2)
 
 
def draw_raceline(surface: pygame.Surface, raceline: Raceline) -> None:
    pts = [w2s(p[0], p[1]) for p in raceline.points]
    pygame.draw.lines(surface, COLOR_REF_PATH, True, pts, 2)
 
 
def draw_car(surface: pygame.Surface, state: VehicleState) -> None:
    hw, hl = CAR_WIDTH / 2, CAR_LENGTH / 2
    c, s = math.cos(state.psi), math.sin(state.psi)
    corners = [
        w2s(state.x + c*fx - s*fy, state.y + s*fx + c*fy) for fx, fy in [(-hl,-hw),(hl,-hw),(hl,hw),(-hl,hw)]
    ]
    pygame.draw.polygon(surface, COLOR_CAR, corners)

def draw_pred(surface: pygame.Surface, pred: np.ndarray) -> None:
    pts = [w2s(p[0], p[1]) for p in pred]
    pygame.draw.lines(surface, COLOR_MPC_PATH, False, pts, 2)

def main():
    pygame.init()
    screen  = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock   = pygame.time.Clock()
 
    raceline = load_raceline()
    p0  = raceline.points[0, :2]
    pred = np.zeros((MPPI_HORIZON, 2))
    # state = VehicleState(x=p0[0], y=p0[1], psi=raceline.psis[0], v=0.5)
    state = VehicleState(x=p0[0], y=p0[1], psi=math.pi*1.25, v=0.5)
    working_sequence = np.zeros((MPPI_HORIZON, 2))
    closest_index = 0
    accum = 0.0
    running = True
 
    while running:
        dt_real = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
 
        accum += dt_real
        while accum >= SIM_DT:
            closest_index   = find_closest_raceline_point(state, raceline)
            control, working_sequence, pred = mppi_step(state, raceline, closest_index, working_sequence)
            state = step(state, control, SIM_DT)
            accum -= SIM_DT
 
        screen.fill(COLOR_BACKGROUND)
        draw_raceline(screen, raceline)
        draw_pred(screen, pred)
        draw_car(screen, state)
        pygame.display.flip()
 
    pygame.quit()
 
 
if __name__ == "__main__":
    main()