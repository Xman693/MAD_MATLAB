classdef radar
    properties
        RadarParams
    end

    methods
        function obj = radar(RadarParams)
            obj.RadarParams = RadarParams;
        end

        function BeamAngle = BeamSteeringTrack(obj, TargetEstNed, BeamAngle)
            TargetPosRadar = RadarToNed(1, TargetEstNed(1:2), BeamAngle, obj.RadarParams.NomRadarPitchAngle);
            lineOfSight = atan2(TargetPosRadar(2), TargetPosRadar(1));
            step = deg2rad(1);
            if lineOfSight > step
                BeamAngle = BeamAngle - step;
            elseif lineOfSight < -step
                BeamAngle = BeamAngle + step;
            end
        end

        function [BeamAngle, DirectionFlag] = BeamSteeringSearch(obj, BeamAngle, DirectionFlag)
            lowerLimit = -obj.RadarParams.NomRadarPitchAngle;
            upperLimit = obj.RadarParams.FOV;
            step = deg2rad(1);
            if BeamAngle >= upperLimit
                DirectionFlag = "CW";
            elseif BeamAngle <= lowerLimit
                DirectionFlag = "CCW";
            end
            if DirectionFlag == "CCW"
                BeamAngle = min(BeamAngle + step, upperLimit);
            else
                BeamAngle = max(BeamAngle - step, lowerLimit);
            end
        end

        function [TargetDetectionTrue, TrueMeasurements] = TargetDetected(obj, TrueTargetNED, BeamAngle)
            TrueTargetPos = RadarToNed(1, TrueTargetNED(1:2), BeamAngle, obj.RadarParams.TrueRadarPitchAngle);
            TrueTargetVel = RadarToNed(1, TrueTargetNED(3:4), BeamAngle, obj.RadarParams.TrueRadarPitchAngle);
            TrueMeasurements.Range = norm(TrueTargetPos);
            TrueMeasurements.LineOfSight = -atan2(TrueTargetPos(2), TrueTargetPos(1));
            TrueMeasurements.RangeRate = dot(TrueTargetPos, TrueTargetVel) / max(TrueMeasurements.Range, eps);
            TargetDetectionTrue = TrueMeasurements.Range <= obj.RadarParams.MaxRange && ...
                abs(TrueMeasurements.LineOfSight) <= obj.RadarParams.BeamWidth;
        end

        function RawMeasurements = GetRawMeasurement(obj, TrueMeasurements)
            covariance = obj.RadarParams.MeasCovar;
            noise = chol(covariance + eps*eye(3), 'lower') * randn(3,1);
            RawMeasurements.Range = TrueMeasurements.Range + noise(1);
            RawMeasurements.RangeRate = TrueMeasurements.RangeRate + noise(2);
            RawMeasurements.LineOfSight = TrueMeasurements.LineOfSight + noise(3);
        end

        function [RawRadarMeasurement, TrueRadarMeasurement] = get_radar_measurement(obj, BeamAngle, TrueTargetState)
            [detected, TrueRadarMeasurement] = obj.TargetDetected(TrueTargetState, BeamAngle);
            if detected
                RawRadarMeasurement = obj.GetRawMeasurement(TrueRadarMeasurement);
            else
                RawRadarMeasurement = [];
            end
        end

        function [BeamAngle, DirectionFlag] = get_radar_gimble(obj, BeamAngle, RawRadarMeasurement, DirectionFlag, TargetStateEst)
            if isempty(RawRadarMeasurement) || isempty(TargetStateEst)
                [BeamAngle, DirectionFlag] = obj.BeamSteeringSearch(BeamAngle, DirectionFlag);
            else
                BeamAngle = obj.BeamSteeringTrack(TargetStateEst, BeamAngle);
            end
        end
    end
end
