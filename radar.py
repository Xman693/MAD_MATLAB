import numpy as np
from RadarToNed import RadarToNed


class radar:
    def __init__(self, RadarParams):
        self.RadarParams = RadarParams

    def BeamSteeringTrack(self, TargetEstNed, BeamAngle):
        TargetPosRadar = RadarToNed(
            1,
            TargetEstNed[0:2],
            BeamAngle,
            self.RadarParams["NomRadarPitchAngle"],
        )
        line_of_sight = -np.arctan2(TargetPosRadar[1], TargetPosRadar[0])
        step = np.deg2rad(1)
        if line_of_sight > step:
            BeamAngle += step
        elif line_of_sight < -step:
            BeamAngle -= step
        return BeamAngle

    def BeamSteeringSearch(self, BeamAngle, DirectionFlag):
        lowerLimit = -self.RadarParams["NomRadarPitchAngle"]
        upperLimit = self.RadarParams["FOV"]
        step = np.deg2rad(1)
        if BeamAngle >= upperLimit:
            DirectionFlag = "CW"
        elif BeamAngle <= lowerLimit:
            DirectionFlag = "CCW"
        if DirectionFlag == "CCW":
            BeamAngle = min(BeamAngle + step, upperLimit)
        else:
            BeamAngle = max(BeamAngle - step, lowerLimit)
        return BeamAngle, DirectionFlag

    def TargetDetected(self, TrueTargetNED, BeamAngle):
        TrueTargetPos = RadarToNed(
            1,
            TrueTargetNED[0:2],
            BeamAngle,
            self.RadarParams["TrueRadarPitchAngle"],
        )
        TrueTargetVel = RadarToNed(
            1,
            TrueTargetNED[2:4],
            BeamAngle,
            self.RadarParams["TrueRadarPitchAngle"],
        )
        TrueMeasurements = {
            "Range": np.linalg.norm(TrueTargetPos),
            "LineOfSight": -np.arctan2(TrueTargetPos[1], TrueTargetPos[0]),
        }
        TrueMeasurements["RangeRate"] = (
            np.dot(TrueTargetPos, TrueTargetVel)
            / max(TrueMeasurements["Range"], np.finfo(float).eps)
        )
        detected = (
            TrueMeasurements["Range"] <= self.RadarParams["MaxRange"]
            and abs(TrueMeasurements["LineOfSight"]) <= self.RadarParams["BeamWidth"]
        )
        return detected, TrueMeasurements

    def GetRawMeasurement(self, TrueMeasurements):
        covariance = self.RadarParams["MeasCovar"]
        noise = np.linalg.cholesky(
            covariance + np.finfo(float).eps * np.eye(3)
        ) @ np.random.randn(3)
        return {
            "Range": TrueMeasurements["Range"] + noise[0],
            "RangeRate": TrueMeasurements["RangeRate"] + noise[1],
            "LineOfSight": TrueMeasurements["LineOfSight"] + noise[2],
        }

    def get_radar_measurement(self, BeamAngle, TrueTargetState):
        detected, TrueRadarMeasurement = self.TargetDetected(TrueTargetState, BeamAngle)
        if detected:
            return self.GetRawMeasurement(TrueRadarMeasurement), TrueRadarMeasurement
        return None, TrueRadarMeasurement

    def get_radar_gimble(
        self, BeamAngle, TargetDetected, DirectionFlag, TargetStateEst
    ):
        if not TargetDetected:
            return self.BeamSteeringSearch(BeamAngle, DirectionFlag)
        if TargetStateEst is None:
            return BeamAngle, DirectionFlag
        BeamAngle = self.BeamSteeringTrack(TargetStateEst, BeamAngle)
        return BeamAngle, DirectionFlag
