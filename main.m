
% -------- Sim1 --------------------------------------
SimParams = get_sim_params();
RadarParams = get_radar_params();
EstimationParams = get_estimation_params(); 
UpdateRates = get_updates_rates();
TargetTraj = targettraj(SimParams); 


Sim1 = Sim(SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams) ; 
Sim1Outputs = Sim1.RunSim(); 

% ------- Sim2 ---------------------------

