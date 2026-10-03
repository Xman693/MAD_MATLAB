import numpy as np
from estimation import estimation
from get_schedule import get_schedule
from radar import radar


class Sim:
    def __init__(self, SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams):
        self.SimParams = SimParams
        self.RadarParams = RadarParams
        self.UpdateRates = UpdateRates
        self.TargetTrajectory = TargetTrajectory
        self.EstimationParams = EstimationParams

    def RunSim(self):
        Radar = radar(self.RadarParams)
        Estimation = estimation(self.EstimationParams)
        clocks = {}
        beamAngle = 0.0
        directionFlag = "CCW"
        targetStateEst = None
        P = None
        time = 0.0
        i = 0
        n = self.TargetTrajectory.shape[1]

        rawHistory = np.full((3, n), np.nan)
        trueHistory = np.full((3, n), np.nan)
        estimateHistory = np.full((4, n), np.nan)
        covarianceHistory = np.full((4, 4, n), np.nan)
        beamHistory = np.full(n, np.nan)

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
                )
            if schedule["RadarGimbleDue"]:
                beamAngle, directionFlag = Radar.get_radar_gimble(
                    beamAngle, rawMeasurement, directionFlag, targetStateEst
                )

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
        }
        return SimOutputs
