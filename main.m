% -------- Active radar and target-estimation simulation --------
SimParams = get_sim_params();
RadarParams = get_radar_params();
EstimationParams = get_estimation_params();
UpdateRates = get_update_rates();
[~, TargetTrajectory] = target_traj(SimParams);
TargetTrajectory = TargetTrajectory.';

Sim1 = Sim(SimParams, RadarParams, UpdateRates, TargetTrajectory, EstimationParams);
Sim1Outputs = Sim1.RunSim();

% Fire control, uplink, seeker, guidance, and autopilot scenarios are future work.
% Keep their implementation disabled until those subsystems are ready.
