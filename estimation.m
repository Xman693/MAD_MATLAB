classdef estimation
    properties
    EstimationParams
    end
   
    methods
        function obj = estimation(EstimationParams)
           obj.EstimationParams = EstimationParams;

        end
     

        function [F, Q, R] = get_ABT_KF_MODELS(obj, ProcessingTime, MeasCovar)

            dt = ProcessingTime; % time interval at which KF is ran at (100 Hz = 1/100, etc) 

            % ABT is Constant Velocity Model (x = [px pz, vx, vz])

            A = obi.EstimationParams.ABT.A; % process model
            Q = obj.EstimationParams.ABT.Q; % process noise covariance for CV (small value)



            F = eye(4) + A*dt; % discrete process matrix

            R = MeasCovar;

        end

        function H = get_H_model(TargetStateEst)

            % NED 
            px = TargetStateEst(1); pz = TargetStateEst(2); 
            vx = TargetStateEst(3); vz = TargetStateEst(4);

            R = norm([px, pz]); 


            H1 = [px/R, pz/R, 0,0];
            H2 = [(vx*pz^2 - px*pz*vz)/R^3, (vz*px^2 - px*pz*vx)/R^3, px/R, pz/R];
            H3 = [z/R^2, -x/R^2, 0,0];

            H = [H1; H2; H3]; 
        




        end















        function [TargetStateEst, P] = get_target_state_est(obj, TargetType, TargetStateEst, P, ProcessingTimeInterval, MeasCovar, RawMeasurements)

            if TargetType == "ABT"
               
                [F,Q,R] = get_ABT_KF_MODELS(obj, ProcessingTimeInterval, MeasCovar);
            end

                if isempty(P) % inital detection

                    sigma = 1e6; % default large intial target state uncertainity
                    P = sigma * eye(4); % initial covariancd matrix

                    % if P is empty --> TargetStateEst is empty as well,
                    % initialize it

                    R = RawMeasurements.Range;
                    los = RawMeasurements.los; 

                    % initial guess (R/los in radar beam down frame but
                    % target state est needed in radar NED, could convert
                    % but its fine because the target is far and the initial
                    % covariance is high)
                    TargetStateEst = [R*cos(los); -R*sin(los); 300; 0]; 
                else 
                    % use P and TargetStateEst to propagate forward

                    if isempty(RawMeasurements) 
                        TargetStateEst = F * TargetStateEst; % propogate using model only
                        P = F * P * F' + Q;
                    else

                        measurement = [RawMeasurements.Range; RawMeasurements.RangeRate; RawMeasurements.los];
                        H = get_H_model(TargetStateEst); 

                        innovation = measurement - H * TargetStateEst;

                        S = H * P * H' + R; % innvovation covariance
                        K = P * H' / S; % Kalman Gain
                       
                        TargetStateEst = TargetStateEst + K * innovation;
                        P = (eye(size(P)) - K * H) * P;
                    end

                        
                      

  
                

            

            end
        end
    end
end
