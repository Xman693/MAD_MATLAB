import numpy as np


class estimation:
    def __init__(self, EstimationParams):
        self.EstimationParams = EstimationParams


    def get_ABT_KF_MODELS(self, ProcessingTime, MeasCovar):
        A = self.EstimationParams["ABT"]["A"]
        F = np.eye(4) + A * ProcessingTime
        Q = self.EstimationParams["ABT"]["Q"]
        R = MeasCovar
        return F, Q, R

    def get_H_model(self, TargetStateEst):
        px, pz, vx, vz = TargetStateEst
        range_ = max(np.linalg.norm([px, pz]), np.finfo(float).eps)
        radial_velocity_numerator = px * vx + pz * vz
        return np.array(
            [
                [px / range_, pz / range_, 0, 0],
                [
                    vx / range_ - radial_velocity_numerator * px / range_**3,
                    vz / range_ - radial_velocity_numerator * pz / range_**3,
                    px / range_,
                    pz / range_,
                ],
                [pz / range_**2, -px / range_**2, 0, 0],
            ]
        )

    def get_target_state_est(
        self,
        TargetType,
        TargetStateEst,
        P,
        ProcessingTimeInterval,
        MeasCovar,
        RawMeasurements,
        RadarAngle,
    ):
        if TargetType != "ABT":
            raise ValueError(f"Unsupported target type: {TargetType}")
        F, Q, R = self.get_ABT_KF_MODELS(ProcessingTimeInterval, MeasCovar)

        if P is None:
            if RawMeasurements is None:
                return None, None
            range_ = RawMeasurements["Range"]
            # Radar LOS is -atan2(z, x) relative to the beam, so NED angle is -(los + RadarAngle).
            ned_angle = RawMeasurements["LineOfSight"] + RadarAngle
            TargetStateEst = np.array(
                [range_ * np.cos(ned_angle), -range_ * np.sin(ned_angle), 0.0, 0.0]
            )
            P = 1e6 * np.eye(4)
            return TargetStateEst, P

        TargetStateEst = F @ TargetStateEst
        P = F @ P @ F.T + Q
        if RawMeasurements is None:
            return TargetStateEst, P

        measurement = np.array(
            [
                RawMeasurements["Range"],
                RawMeasurements["RangeRate"],
                RawMeasurements["LineOfSight"],
            ]
        )
        px, pz, vx, vz = TargetStateEst
        range_ = max(np.linalg.norm([px, pz]), np.finfo(float).eps)
        predicted = np.array(
            [range_, (px * vx + pz * vz) / range_, -np.arctan2(pz, px) - RadarAngle]
        )
        H = self.get_H_model(TargetStateEst)
        innovation = measurement - predicted
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))
        S = H @ P @ H.T + R
        K = np.linalg.solve(S.T, (P @ H.T).T).T
        TargetStateEst = TargetStateEst + K @ innovation
        P = (np.eye(4) - K @ H) @ P
        return TargetStateEst, P
