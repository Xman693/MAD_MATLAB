import numpy as np


def get_estimation_params():
    A = np.zeros((4, 4))
    A[0, 2] = 1
    A[1, 3] = 1
    Q = 1e-3 * np.eye(4)
    return {"ABT": {"A": A, "Q": Q}}
