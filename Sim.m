classdef Sim
    properties
        SimParams
        RadarParams
        UpdateRates
        TargetTrajectory
        EstimationParams
    end

    methods
        function obj = Sim(SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams)
            obj.SimParams = SimParams;
            obj.RadarParams = RadarParams;
            obj.UpdateRates = UpdateRates;
            obj.TargetTrajectory = TargetTrajectory;
            obj.EstimationParams = EstimationParams;
        end

        function SimOutputs = RunSim(obj)
            Radar = radar(obj.RadarParams);
            Estimation = estimation(obj.EstimationParams);
            clocks = struct();
            beamAngle = 0;
            directionFlag = "CCW";
            targetStateEst = [];
            P = [];
            time = 0;
            i = 1;
            n = size(obj.TargetTrajectory, 2);

            rawHistory = nan(3,n);
            trueHistory = nan(3,n);
            estimateHistory = nan(4,n);
            covarianceHistory = nan(4,4,n);
            beamHistory = nan(1,n);

            while time < obj.SimParams.Tmax && i <= n
                [schedule, clocks] = get_schedule(obj, clocks);
                trueTargetState = obj.TargetTrajectory(:,i);
                rawMeasurement = [];
                trueMeasurement = [];

                if schedule.RadarProcessingDue
                    [rawMeasurement, trueMeasurement] = Radar.get_radar_measurement(beamAngle, trueTargetState);
                end
                if schedule.RadarTargetStateEstDue
                    [targetStateEst, P] = Estimation.get_target_state_est( ...
                        "ABT", targetStateEst, P, obj.UpdateRates.target_estimation, ...
                        obj.RadarParams.MeasCovar, rawMeasurement);
                end
                if schedule.RadarGimbleDue
                    [beamAngle, directionFlag] = Radar.get_radar_gimble( ...
                        beamAngle, rawMeasurement, directionFlag, targetStateEst);
                end

                if ~isempty(rawMeasurement)
                    rawHistory(:,i) = [rawMeasurement.Range; rawMeasurement.RangeRate; rawMeasurement.LineOfSight];
                end
                if ~isempty(trueMeasurement)
                    trueHistory(:,i) = [trueMeasurement.Range; trueMeasurement.RangeRate; trueMeasurement.LineOfSight];
                end
                if ~isempty(targetStateEst)
                    estimateHistory(:,i) = targetStateEst;
                    covarianceHistory(:,:,i) = P;
                end
                beamHistory(i) = beamAngle;

                time = time + obj.SimParams.dt;
                i = i + 1;
            end

            SimOutputs.Time = (0:i-2) * obj.SimParams.dt;
            SimOutputs.RawRadarMeasurementHistory = rawHistory(:,1:i-1);
            SimOutputs.TrueRadarMeasurementHistory = trueHistory(:,1:i-1);
            SimOutputs.EstTargetStateHistory = estimateHistory(:,1:i-1);
            SimOutputs.CovarHist = covarianceHistory(:,:,1:i-1);
            SimOutputs.BeamAngleHistory = beamHistory(1:i-1);
        end
    end
end
