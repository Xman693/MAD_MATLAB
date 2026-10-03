import numpy as np
from RK4 import RK4


def target_traj(SimParams):
    # Initial position and velocity (NED).
    p0 = np.array([100000.0, -50000.0])
    v0 = np.array([-250.0, 0.0])
    x0 = np.concatenate((p0, v0))

    Tmax = SimParams["Tmax"]
    dt = SimParams["dt"]
    N = int(np.floor(Tmax / dt))
    t = np.arange(N + 1) * dt
    if t[-1] < Tmax:
        t = np.append(t, Tmax)

    trajectory = np.zeros((len(t), len(x0)))
    trajectory[0, :] = x0
    x = x0.copy()
    for k in range(len(t) - 1):
        dt_k = t[k + 1] - t[k]
        x = RK4(_cv_dynamics, t[k], x, dt_k)
        trajectory[k + 1, :] = x
    return t, trajectory


def _cv_dynamics(_t, x):
    return np.array([x[2], x[3], 0.0, 0.0])
