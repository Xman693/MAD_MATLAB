import numpy as np


def get_radar_params():
    return {
        "MaxRange": 100000,
        "FOV": np.deg2rad(75),
        "NomRadarPitchAngle": np.deg2rad(30),
        "TrueRadarPitchAngle": np.deg2rad(30),
        "BeamWidth": np.deg2rad(2.5),
        # Idealized default until sensor error specifications are available.
        "MeasCovar": np.zeros((3, 3)),
    }
