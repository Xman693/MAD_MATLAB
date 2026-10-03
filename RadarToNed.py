import numpy as np


def RadarToNed(direction, x, phi, theta):
    # 0: Radar to NED; 1: NED to Radar.
    angle = phi + theta
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    if direction == 0:
        return rotation @ x
    return rotation.T @ x
