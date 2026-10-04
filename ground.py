import numpy as np
from RK4 import RK4


class ground:
    def __init__(self, GroundParams):
        self.GroundParams = GroundParams
        

   # ---- Initial Turn/Pre-Launch Guidance ----
   
    # Constant-velocity dynamics for state [x, y, vx, vy]
    @staticmethod
    def _cv_dynamics(_t, x):
        return np.array([x[2], x[3], 0.0, 0.0])

    def extrapolate_target(self, TargetStateEst):
        
        x = TargetStateEst[0]
        CommitZone = self.GroundParams["CommitZone"]
    
        dist_to_commit = x - CommitZone
        
        duration = dist_to_commit / abs(TargetStateEst[2]) + 10   # Assuming TargetStateEst[2] is the velocity in x-direction (10 is safety factor )
        
        tgo =  dist_to_commit / abs(TargetStateEst[2])
        
        if tgo <= 0:
            tgo = None
            traj = None
            return tgo, traj
        
       
        dt = self.GroundParams["dt"]
        
        
        """Integrate the target state with RK4 for `duration` seconds at step `dt`.

        Returns a (len(state), N+1) array; column k is the state at t = k*dt
        (last column lands exactly on `duration`).
        """
        x = np.asarray(TargetStateEst, dtype=float)
        t = np.arange(0.0, duration, dt)
        t = np.append(t, duration)  # final partial step lands on duration

        traj = np.zeros((x.size, t.size))
        traj[:, 0] = x
        for k in range(t.size - 1):
            x = RK4(self._cv_dynamics, t[k], x, t[k + 1] - t[k])
            traj[:, k + 1] = x
        return traj,tgo

    # 1) Get Launch Point
    
    def get_launch_point(self, TargetStateEst):
        
        traj, tgo = self.extrapolate_target(TargetStateEst)

        if tgo is None or traj is None:
            return None, None
   
        idx = np.flatnonzero(traj[0, :] <= self.GroundParams["CommitZone"])
        # The trajectory ends exactly at the commit zone; floating-point error can miss it.
        LaunchPoint_idx = idx[0] if idx.size else traj.shape[1] - 1
        LaunchPoint = traj[0:2, LaunchPoint_idx]
        
        
        return LaunchPoint, tgo
    
    
    
#     def find_initial_waypoint(self, LaunchPoint, ...):
        
#         gamma_guess = np.deg2rad(45) # Initial guess for flight path angle
#         iter_max = 10
#         delta_gamma = np.deg2rad(1)  # Small change in flight path angle for numerical derivative
#         tol = np.deg2rad(0.5)  # Tolerance for convergence in flight path angle
#         gamma_update = np.inf # arbitrary large initial difference
#         iter = 0
        
#         # ------ Iterative Optimization Loop for Flight Path Angle ------
#         while abs(gamma_update) > tol and iter <= iter_max:
            
#             traj_cost = get_cost(gamma_guess)
#             traj_cost_plus = get_cost(gamma_guess + delta_gamma)
#             traj_cost_minus = get_cost(gamma_guess - delta_gamma)
         
#             first_diff = (traj_cost_plus - traj_cost_minus) / (2 * delta_gamma)
#             second_diff = (traj_cost_plus - 2 * traj_cost + traj_cost_minus) / (delta_gamma ** 2)
         
#             gamma_update = - first_diff / second_diff
         
#             gamma_guess += gamma_update
#             iter += 1
         
         
         
#         gamma_waypoint = gamma_guess
        
#          # --------- Computes Nominal Burst Time and Initial Waypoint ----------------
         
#         [t_burst_nom, initial_waypoint] = get_nominal_burst_and_waypoint(gamma_guess) 
         
#         return t_burst_nom, initial_waypoint
        
        
        
        
    
