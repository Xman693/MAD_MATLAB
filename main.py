import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from Sim import Sim
from get_estimation_params import get_estimation_params
from get_radar_params import get_radar_params
from get_sim_params import get_sim_params
from get_update_rates import get_update_rates
from get_ground_params import get_ground_params
from target_traj import target_traj


SimParams = get_sim_params()
RadarParams = get_radar_params()
EstimationParams = get_estimation_params()
GroundParams = get_ground_params()
UpdateRates = get_update_rates()
TargetTime, TargetHistory = target_traj(SimParams)
TargetTrajectory = TargetHistory.T

Sim1 = Sim(SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams, GroundParams)
Sim1Outputs = Sim1.RunSim()

# Fire control, uplink, seeker, guidance, and autopilot are future work.

plt.figure()
plt.plot(Sim1Outputs["Time"], np.rad2deg(Sim1Outputs["BeamAngleHistory"]))
plt.xlabel("Time (s)")
plt.ylabel("Beam Angle (deg)")
plt.title("Radar Beam Angle vs Time")
plt.grid(True)

# True vs estimated NED state.
TrueState = TargetTrajectory[:, : len(Sim1Outputs["Time"])]
EstState = Sim1Outputs["EstTargetStateHistory"]
StateLabels = ["North Position (m)", "Down Position (m)", "North Velocity (m/s)", "Down Velocity (m/s)"]
fig_state, axes = plt.subplots(2, 2, sharex=True, figsize=(11, 7))
for k, axis in enumerate(axes.flat):
    axis.plot(Sim1Outputs["Time"], TrueState[k], "k", label="True")
    axis.plot(Sim1Outputs["Time"], EstState[k], "r--", label="Estimate")
    axis.set_ylabel(StateLabels[k])
    axis.grid(True)
    if k in (2, 3):
        axis.set_xlabel("Time (s)")
axes[0, 0].legend()
fig_state.suptitle("True vs Estimated Target State (NED)")

# Estimation covariance (1-sigma of each state).
CovarHist = Sim1Outputs["CovarHist"]
SigmaLabels = ["North Pos 1-sigma (m)", "Down Pos 1-sigma (m)", "North Vel 1-sigma (m/s)", "Down Vel 1-sigma (m/s)"]
fig_cov, cov_axes = plt.subplots(2, 2, sharex=True, figsize=(11, 7))
for k, axis in enumerate(cov_axes.flat):
    axis.semilogy(Sim1Outputs["Time"], np.sqrt(CovarHist[k, k, :]))
    axis.set_ylabel(SigmaLabels[k])
    axis.grid(True)
    if k in (2, 3):
        axis.set_xlabel("Time (s)")
fig_cov.suptitle("Estimation Covariance History")

# Radar beam sweep and target trajectory.
TargetX = TargetTrajectory[0, :]
TargetZ = TargetTrajectory[1, :]
Time = Sim1Outputs["Time"]
BeamAngleHistory = Sim1Outputs["BeamAngleHistory"]
LaunchPointHistory = Sim1Outputs["LaunchPointHistory"]
RadarPitchAngle = RadarParams["TrueRadarPitchAngle"]
BeamWidth = RadarParams["BeamWidth"]
BeamRange = np.max(np.sqrt(TargetX**2 + TargetZ**2))

fig, ax = plt.subplots()
TargetPath, = ax.plot([], [], "k-", linewidth=2, label="True")
TargetPosition, = ax.plot([], [], "ko", markersize=5)
EstPath, = ax.plot([], [], "r--", linewidth=1.5, label="Estimate")
EstPosition, = ax.plot([], [], "r^", markersize=6)
LaunchPointMarker, = ax.plot([], [], "g*", markersize=14, label="Launch Point")
UpperBeam, = ax.plot([0, 0], [0, 0], "b-", linewidth=1.5)
LowerBeam, = ax.plot([0, 0], [0, 0], "b-", linewidth=1.5)
Cone = Polygon([[0, 0], [0, 0], [0, 0]], color="b", alpha=0.15, edgecolor="none")
ax.add_patch(Cone)
ax.set_xlabel("X Position")
ax.set_ylabel("Z Position (NED, negative up)")
ax.set_title("Radar Beam Sweep")
ax.set_xlim(-BeamRange, BeamRange)
ax.set_ylim(BeamRange, -BeamRange)  # inverted so negative Z is up
ax.set_aspect("equal", adjustable="box")
ax.grid(True)
ax.legend(loc="upper right")

TgoHistory = Sim1Outputs["TgoHistory"]
TgoAx = ax.inset_axes([0.07, 0.07, 0.3, 0.22])
TgoLine, = TgoAx.plot([], [], "g-")
TgoAx.set_xlim(Time[0], Time[-1])
TgoValid = TgoHistory[~np.isnan(TgoHistory)]
TgoAx.set_ylim(0, (TgoValid.max() if TgoValid.size else 1) * 1.05)
TgoAx.set_title("Tgo (s)", fontsize=8)
TgoAx.tick_params(labelsize=7)
TgoAx.grid(True)

AnimationStep = 400
for i in range(0, len(Time), AnimationStep):
    RadarAngle = BeamAngleHistory[i] + RadarPitchAngle
    UpperAngle = RadarAngle + BeamWidth
    LowerAngle = RadarAngle - BeamWidth
    UpperX = BeamRange * np.cos(UpperAngle)
    UpperZ = -BeamRange * np.sin(UpperAngle)
    LowerX = BeamRange * np.cos(LowerAngle)
    LowerZ = -BeamRange * np.sin(LowerAngle)
    UpperBeam.set_data([0, UpperX], [0, UpperZ])
    LowerBeam.set_data([0, LowerX], [0, LowerZ])
    Cone.set_xy([[0, 0], [UpperX, UpperZ], [LowerX, LowerZ]])
    TargetPath.set_data(TargetX[: i + 1], TargetZ[: i + 1])
    TargetPosition.set_data([TargetX[i]], [TargetZ[i]])
    EstPath.set_data(EstState[0, : i + 1], EstState[1, : i + 1])
    EstPosition.set_data([EstState[0, i]], [EstState[1, i]])
    ValidLaunch = np.flatnonzero(~np.isnan(LaunchPointHistory[0, : i + 1]))
    if ValidLaunch.size:
        LaunchPointMarker.set_data(
            [LaunchPointHistory[0, ValidLaunch[-1]]], [LaunchPointHistory[1, ValidLaunch[-1]]]
        )
    TgoLine.set_data(Time[: i + 1], TgoHistory[: i + 1])
    ax.set_title(f"Radar Beam Sweep - t = {Time[i]:.2f} s")
    fig.canvas.draw_idle()
    plt.pause(0.001)

plt.show()
