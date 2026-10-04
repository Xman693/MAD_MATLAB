import numpy as np


def get_ground_params():
    return {
        "CommitZone": 50000, # 50 km 
        "dt": 1, # extrap time 
        "Horizon": 300, # extrap horizon (s)
    }
