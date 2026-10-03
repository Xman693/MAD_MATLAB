import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from Sim import Sim
from get_estimation_params import get_estimation_params
from get_radar_params import get_radar_params
from get_sim_params import get_sim_params
from get_update_rates import get_update_rates
from target_traj import target_traj


SimParams = get_sim_params()
RadarParams = get_radar_params()
EstimationParams = get_estimation_params()
UpdateRates = get_update_rates()
TargetTime, TargetHistory = target_traj(SimParams)
TargetTrajectory = TargetHistory.T

Sim1 = Sim(SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams)
Sim1Outputs = Sim1.RunSim()

# Fire control, uplink, seeker, guidance, and autopilot are future work.

plt.figure()
plt.plot(Sim1Outputs["Time"], np.rad2deg(Sim1Outputs["BeamAngleHistory"]))
plt.xlabel("Time (s)")
plt.ylabel("Beam Angle (deg)")
plt.title("Radar Beam Angle vs Time")
plt.grid(True)

# Radar beam sweep and target trajectory.
TargetX = TargetTrajectory[0, :]
TargetZ = TargetTrajectory[1, :]
Time = Sim1Outputs["Time"]
BeamAngleHistory = Sim1Outputs["BeamAngleHistory"]
RadarPitchAngle = RadarParams["TrueRadarPitchAngle"]
BeamWidth = RadarParams["BeamWidth"]
BeamRange = np.max(np.sqrt(TargetX**2 + TargetZ**2))

fig, ax = plt.subplots()
ax.plot(TargetX, TargetZ, "k", linewidth=2)
UpperBeam, = ax.plot([0, 0], [0, 0], "b-", linewidth=1.5)
LowerBeam, = ax.plot([0, 0], [0, 0], "b-", linewidth=1.5)
Cone = Polygon([[0, 0], [0, 0], [0, 0]], color="b", alpha=0.15, edgecolor="none")
ax.add_patch(Cone)
ax.set_xlabel("X Position")
ax.set_ylabel("Z Position")
ax.set_title("Radar Beam Sweep")
ax.axis("equal")
ax.grid(True)

AnimationStep = 20
for i in range(0, len(Time), AnimationStep):
    RadarAngle = BeamAngleHistory[i] + RadarPitchAngle
    UpperAngle = RadarAngle + BeamWidth
    LowerAngle = RadarAngle - BeamWidth
    UpperX = BeamRange * np.cos(UpperAngle)
    UpperZ = BeamRange * np.sin(UpperAngle)
    LowerX = BeamRange * np.cos(LowerAngle)
    LowerZ = BeamRange * np.sin(LowerAngle)
    UpperBeam.set_data([0, UpperX], [0, UpperZ])
    LowerBeam.set_data([0, LowerX], [0, LowerZ])
    Cone.set_xy([[0, 0], [UpperX, UpperZ], [LowerX, LowerZ]])
    ax.set_title(f"Radar Beam Sweep - t = {Time[i]:.2f} s")
    fig.canvas.draw_idle()
    plt.pause(0.001)

plt.show()
