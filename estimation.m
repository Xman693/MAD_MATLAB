classdef estimation
    properties
        EstimationParams
    end

    methods
        function obj = estimation(EstimationParams)
            obj.EstimationParams = EstimationParams;
        end

        function [F, Q, R] = get_ABT_KF_MODELS(obj, ProcessingTime, MeasCovar)
            A = obj.EstimationParams.ABT.A;
            F = eye(4) + A * ProcessingTime;
            Q = obj.EstimationParams.ABT.Q;
            R = MeasCovar;
        end

        function H = get_H_model(~, TargetStateEst)
            px = TargetStateEst(1);
            pz = TargetStateEst(2);
            vx = TargetStateEst(3);
            vz = TargetStateEst(4);
            range = max(norm([px; pz]), eps);
            radialVelocityNumerator = px*vx + pz*vz;
            H = [px/range, pz/range, 0, 0; ...
                vx/range-radialVelocityNumerator*px/range^3, vz/range-radialVelocityNumerator*pz/range^3, px/range, pz/range; ...
                pz/range^2, -px/range^2, 0, 0];
        end

        function [TargetStateEst, P] = get_target_state_est(obj, TargetType, TargetStateEst, P, ProcessingTimeInterval, MeasCovar, RawMeasurements)
            if TargetType ~= "ABT"
                error('estimation:UnsupportedTargetType', 'Unsupported target type: %s', TargetType);
            end
            [F, Q, R] = obj.get_ABT_KF_MODELS(ProcessingTimeInterval, MeasCovar);

            if isempty(P)
                if isempty(RawMeasurements)
                    TargetStateEst = [];
                    P = [];
                    return;
                end
                range = RawMeasurements.Range;
                los = RawMeasurements.LineOfSight;
                TargetStateEst = [range*cos(los); -range*sin(los); 0; 0];
                P = 1e6 * eye(4);
                return;
            end

            TargetStateEst = F * TargetStateEst;
            P = F * P * F' + Q;
            if isempty(RawMeasurements)
                return;
            end

            measurement = [RawMeasurements.Range; RawMeasurements.RangeRate; RawMeasurements.LineOfSight];
            px = TargetStateEst(1);
            pz = TargetStateEst(2);
            vx = TargetStateEst(3);
            vz = TargetStateEst(4);
            range = max(norm([px; pz]), eps);
            predicted = [range; (px*vx + pz*vz)/range; -atan2(pz,px)];
            H = obj.get_H_model(TargetStateEst);
            innovation = measurement - predicted;
            innovation(3) = atan2(sin(innovation(3)), cos(innovation(3)));
            S = H * P * H' + R;
            K = (P * H') / S;
            TargetStateEst = TargetStateEst + K * innovation;
            P = (eye(4) - K*H) * P;
        end
    end
end
