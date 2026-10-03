
function SimParams = get_sim_params()

SimParams.dt = 1/2000; % MCC
SimParams.Tmax = 100; 


end

function RadarParams = get_radar_params()

RadarParams.MaxRange = 100000; 
RadarParams.FOV = deg2rad(75); 
RadarParams.NomRadarPitchAngle = deg2rad(30); 
RadarParams.TrueRadarPitchAngle = deg2rad(30); 
RadarParams.BeamWidth = deg2rad(2.5); 

end

function EstimationParams = get_estimation_params()

A = zeros(4,4); 
A(1,3) = 1; A(2,4) = 1; 

Q = 1e-3 * eye(4); 

EstimationParams.ABT.A = A;
EstimationParams.ABT.Q = Q;


end




function update_rates = get_update_rates()

        update_rates.radar_beam = 1/20; 
        update_rates.radar_measurement = 1/20; 
        update_target_estimation = 1/100; 

end
