import numpy as np
from estimation import estimation
from get_schedule import get_schedule
from radar import radar
from ground import ground


class Sim:
    def __init__(self, SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams, GroundParams):
        self.SimParams = SimParams
        self.RadarParams = RadarParams
        self.UpdateRates = UpdateRates
        self.TargetTrajectory = TargetTrajectory
        self.EstimationParams = EstimationParams
        self.GroundParams = GroundParams

    def RunSim(self):
        Radar = radar(self.RadarParams)
        Estimation = estimation(self.EstimationParams)
        Ground = ground(self.GroundParams)
        
        # random initialization -----
        clocks = {}
        beamAngle = 0.0
        directionFlag = "CCW"
        targetStateEst = None
        P = None
        time = 0.0
        i = 0
        n = self.TargetTrajectory.shape[1]
        LaunchPoint = None
        tgo = None

        # ------------ History Initialization ------------
        rawHistory = np.full((3, n), np.nan)
        trueHistory = np.full((3, n), np.nan)
        estimateHistory = np.full((4, n), np.nan)
        covarianceHistory = np.full((4, 4, n), np.nan)
        beamHistory = np.full(n, np.nan)
        launchPointHistory = np.full((2, n), np.nan)
        tgoHistory = np.full(n, np.nan)

        while time < self.SimParams["Tmax"] and i < n:
            schedule, clocks = get_schedule(self, clocks)
            trueTargetState = self.TargetTrajectory[:, i]
            rawMeasurement = None
            trueMeasurement = None

            if schedule["RadarProcessingDue"]:
                rawMeasurement, trueMeasurement = Radar.get_radar_measurement(
                    beamAngle, trueTargetState
                )
            if schedule["RadarTargetStateEstDue"]:
                targetStateEst, P = Estimation.get_target_state_est(
                    "ABT",
                    targetStateEst,
                    P,
                    self.UpdateRates["target_estimation"],
                    self.RadarParams["MeasCovar"],
                    rawMeasurement,
                    beamAngle + self.RadarParams["TrueRadarPitchAngle"],
                )
            if schedule["RadarGimbleDue"]:
                targetDetected, _ = Radar.TargetDetected(trueTargetState, beamAngle)
                beamAngle, directionFlag = Radar.get_radar_gimble(
                    beamAngle,
                    targetDetected,
                    directionFlag,
                    targetStateEst,
                )
            # Skip until the estimate has a usable closing velocity (first estimate has vx = 0).
            if schedule["GroundDue"] and targetStateEst is not None and abs(targetStateEst[2]) > 1.0:
                LaunchPoint, tgo = Ground.get_launch_point(targetStateEst)
              

            if LaunchPoint is not None:
                launchPointHistory[:, i] = LaunchPoint
                
            if tgo is not None:
                tgoHistory[i] = tgo
                
        




            if rawMeasurement is not None:
                rawHistory[:, i] = [
                    rawMeasurement["Range"],
                    rawMeasurement["RangeRate"],
                    rawMeasurement["LineOfSight"],
                ]
            if trueMeasurement is not None:
                trueHistory[:, i] = [
                    trueMeasurement["Range"],
                    trueMeasurement["RangeRate"],
                    trueMeasurement["LineOfSight"],
                ]
            if targetStateEst is not None:
                estimateHistory[:, i] = targetStateEst
                covarianceHistory[:, :, i] = P
            beamHistory[i] = beamAngle

            time += self.SimParams["dt"]
            i += 1

        SimOutputs = {
            "Time": np.arange(i) * self.SimParams["dt"],
            "RawRadarMeasurementHistory": rawHistory[:, :i],
            "TrueRadarMeasurementHistory": trueHistory[:, :i],
            "EstTargetStateHistory": estimateHistory[:, :i],
            "CovarHist": covarianceHistory[:, :, :i],
            "BeamAngleHistory": beamHistory[:i],
            "LaunchPointHistory": launchPointHistory[:, :i],
            "TgoHistory": tgoHistory[:i],
        }
        return SimOutputs
