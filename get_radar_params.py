import numpy as np


def get_radar_params():
    return {
        "MaxRange": 150000,
        "FOV": np.deg2rad(75),
        "NomRadarPitchAngle": np.deg2rad(30),
        "TrueRadarPitchAngle": np.deg2rad(30),
        "BeamWidth": np.deg2rad(2.5),
        # 1-sigma: range 50 m, range rate 5 m/s, line of sight 0.5 deg.
        "MeasCovar": np.diag([50.0**2, 5.0**2, np.deg2rad(0.5) ** 2]),
    }
