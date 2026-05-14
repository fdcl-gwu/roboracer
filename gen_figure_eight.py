#!/usr/bin/env python3
"""
Generate figure_eight.csv for mppi_racing.py / mpc_racing.py.

Geometry (CSV / raw coordinates — load_raceline adds RACELINE_X_OFFSET=2 to x):
  Lower loop  ellipse centred at (1.25, 0), semi-axes 1.25 m (x) × 1.5 m (y)
    Leftmost  = start/end (0, 0)
    Rightmost = crossing  (2.5, 0)
  Upper loop  ellipse centred at (3.75, 0), same semi-axes
    Leftmost  = crossing  (2.5, 0)
    Rightmost = far tip   (5.0, 0)   ← "5 m above" start
    Sides: y = ±1.5 m

Path direction (ensures smooth, continuous heading at every junction):
  Segment 1: lower loop CCW, θ: π → 2π   (bottom half: start → crossing)
  Segment 2: upper loop CW,  θ: π → -π   (full circle: crossing → crossing)
  Segment 3: lower loop CCW, θ: 0 → π    (top half:    crossing → start)
"""

import csv
import numpy as np

# ── Parameters ────────────────────────────────────────────────────────────────
AX    = 1.25          # ellipse x semi-axis  (half the loop length along x)
AY    = 1.5           # ellipse y semi-axis  (half the side-to-side width)
CX1   = 1.25          # lower loop centre x
CX2   = 3.75          # upper loop centre x
V_REF = 1.5           # m/s written to CSV  (load_raceline adds +1 → 2.5 m/s effective)
N     = 600           # total waypoints  (150 per half-ellipse, 300 for full upper)

# ── Unified parameter t ∈ [0, 4) ─────────────────────────────────────────────
# t ∈ [0, 1)  →  segment 1 (lower bottom)
# t ∈ [1, 3)  →  segment 2 (upper full)
# t ∈ [3, 4)  →  segment 3 (lower top)
t = np.linspace(0.0, 4.0, N, endpoint=False)

m1 = t < 1.0
m2 = (t >= 1.0) & (t < 3.0)
m3 = t >= 3.0

x   = np.empty(N)
y   = np.empty(N)
psi = np.empty(N)

# Segment 1 — lower loop CCW (dθ > 0), θ: π → 2π
th1 = np.pi + t[m1] * np.pi
x[m1]   = CX1 + AX * np.cos(th1)
y[m1]   = AY  * np.sin(th1)
# CCW tangent: (dx/dθ, dy/dθ) = (-AX·sin θ,  AY·cos θ)
psi[m1] = np.arctan2( AY * np.cos(th1), -AX * np.sin(th1))

# Segment 2 — upper loop CW (dθ < 0), θ: π → -π
th2 = np.pi - (t[m2] - 1.0) * np.pi
x[m2]   = CX2 + AX * np.cos(th2)
y[m2]   = AY  * np.sin(th2)
# CW tangent: -(dx/dθ, dy/dθ) = (AX·sin θ, -AY·cos θ)
psi[m2] = np.arctan2(-AY * np.cos(th2),  AX * np.sin(th2))

# Segment 3 — lower loop CCW (dθ > 0), θ: 0 → π
th3 = (t[m3] - 3.0) * np.pi
x[m3]   = CX1 + AX * np.cos(th3)
y[m3]   = AY  * np.sin(th3)
psi[m3] = np.arctan2( AY * np.cos(th3), -AX * np.sin(th3))

# ── Arc length ────────────────────────────────────────────────────────────────
dx_seg = np.diff(x, append=x[0])
dy_seg = np.diff(y, append=y[0])
ds     = np.sqrt(dx_seg**2 + dy_seg**2)
s      = np.concatenate([[0.0], np.cumsum(ds[:-1])])
total  = s[-1] + ds[-1]

# ── Write CSV ─────────────────────────────────────────────────────────────────
out = "figure_eight.csv"
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["x", "y", "psi", "v_ref", "s"])
    w.writeheader()
    for i in range(N):
        w.writerow({
            "x":     f"{x[i]:.6f}",
            "y":     f"{y[i]:.6f}",
            "psi":   f"{psi[i]:.6f}",
            "v_ref": f"{V_REF:.6f}",
            "s":     f"{s[i]:.6f}",
        })

print(f"Written {out}: {N} waypoints, total arc = {total:.3f} m")

# ── Sanity checks ─────────────────────────────────────────────────────────────
print(f"  Start:    ({x[0]:.3f}, {y[0]:.3f})  psi={np.degrees(psi[0]):.1f}°")
print(f"  Crossing: ({x[N//6]:.3f}, {y[N//6]:.3f})  psi={np.degrees(psi[N//6]):.1f}°")
print(f"  Far tip:  ({x[N//3]:.3f}, {y[N//3]:.3f})  psi={np.degrees(psi[N//3]):.1f}°")
print(f"  x range:  [{x.min():.3f}, {x.max():.3f}]")
print(f"  y range:  [{y.min():.3f}, {y.max():.3f}]")
max_ds = ds.max()
print(f"  Max point spacing: {max_ds*1000:.1f} mm")
print(f"  Max psi jump: {np.degrees(np.abs(np.diff(psi, append=psi[0]))).max():.2f}°")
