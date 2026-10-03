
classdef radar
    properties
     RadarParams
       
    end

    methods
        function obj = Radar(RadarParams)
          obj.RadarParams = RadarParams;

        end

        function [Mode, TrueMeasurements] = SetMode(obj, TrueTarget, BeamAngle)

           [TargetDetectionTrue, TrueMeasurements] = obj.TargetDetected(TrueTarget, BeamAngle); 

            if TargetDetectionTrue
                Mode = "track";
            else
                Mode = "search";
            end
        end

        function BeamAngle = BeamSteeringTrack(obj, TargetEstNed, BeamAngle)

            TargetPosNED = TargetEstNed(1:2); 

            TargetPosRadar = RadarToNed(1, TargetPosNED, BeamAngle, obj.RadarParams.NomRadarPitchAngle); 

           
            LineOfSight = atan2(TargetPosRadar(2), TargetPosRadar(1)); % if using seeker z up, 

            if LineOfSight > deg2rad(1)
                BeamAngle = BeamAngle - deg2rad(1);
            elseif LineOfSight < deg2rad(1)
                BeamAngle = BeamAngle + deg2rad(1); 
            else 
                BeamAngle = 0;
            end




        end

        function [BeamAngle, DirectionFlag] = BeamSteeringSearch(obj, BeamAngle, DirectionFlag)

            % Bounds BeamAngle Between +FOV and -RadarPitchAngle (could not
            % be optimal for certain radar emplacements aka high
            % elevation)

            % checks if change in beam search direction is needed
            if BeamAngle >= obj.RadarParams.FOV
                DirectionFlag = "CW";
            end

            if BeamAngle <= obj.RadarParams.NomRadarPitchAngle
                DirectionFlag = "CCW"; 
            end

            if DirectionFlag == "CCW"
                BeamAngle = BeamAngle + deg2rad(1); % add 1 degree 
            else
                BeamAngle = BeamAngle - deg2rad(1); 
            end

        end



        function [TargetDetectionTrue, TrueMeasurements] = TargetDetected(obj, TrueTargetNED, BeamAngle)

           TrueTargetPos = RadarToNed(1,TrueTargetNED(1:2), BeamAngle, obj.RadarParams.NomRadarPitchAngle); 
           TrueTargetVel = RadarToNed(1,TrueTargetNED(3:4), BeamAngle, obj.RadarParams.NomRadarPitchAngle);

           TrueMeasurements.LineOfSight = -atan2(TrueTargetPos(2), TrueTargetPos(1)); 
           TrueMeasurements.Range = norm(TrueTargetPos); 
           TrueMeasurements.RangeRate = dot(TrueTargetPos, TrueTargetVel)/Range; 

           if Range > obj.RadarParams.MaxRange
               TargetDetectionTrue = false; 
           elseif abs(LineOfSight) > obj.RadarParams.BeamWidth
               TargetDetectionTrue = false; 
           else 
               TargetDetectionTrue = true; 
           end 
           

        end

        function RawMeasurements = GetRawMeasurement(obj, TrueMeasurements)

         sigma_los = sqrt(obj.RadarParams.MeasCovar(1,1)); 
         sigma_range = sqrt(obj.RadarParams.MeasCovar(2,2)); 
         sigma_range_rate = sqrt(obj.RadarParams.MeasCovar(3,3)); 

         RawMeasurements.LineOfSight = TrueMeasurements.LineOfSight + sigma_los*randn; 
         RawMeasurements.Range = TrueMeasurements.Range * sigma_range * randn; 
         RawMeasurements.RangeRate = TrueMeasurements.RangeRate * sigma_range_rate * randn; 

        end

% called in the main function
        function [RawRadarMeasurement, TrueRadarMeasurement] = get_radar_measurement(obj, BeamAngle, TrueTargetState)

            [TargetDetected, TrueRadarMeasurement] = radar.TargetDetected(obj, TrueTargetState, BeamAngle); 

            if TargetDetected
                RawRadarMeasurement = obj.GetRawMeasurement(TrueRadarMeasurement);
            else
                RawRadarMeasurement = []; % no measurements (dropped track / no track)
            end
        end
    
        function [BeamAngle, DirectionFlag] = get_radar_gimble(obj, BeamAngle, RawRadarMeasurement, DirectionFlag, TargetStateEst)

             if RawRadarMeasurement == []

                 [BeamAngle, DirectionFlag] = radar.BeamSteeringSearch(obj, BeamAngle, DirectionFlag); 
             else 
                 BeamAngle = radar.BeamSteeringTrack(obj,TargetStateEst, BeamAngle);

             end
             
        end



    end
end
