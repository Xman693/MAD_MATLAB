function RadarParams = get_radar_params()
%GET_RADAR_PARAMS Default radar configuration.
RadarParams.MaxRange = 100000;
RadarParams.FOV = deg2rad(75);
RadarParams.NomRadarPitchAngle = deg2rad(30);
RadarParams.TrueRadarPitchAngle = deg2rad(30);
RadarParams.BeamWidth = deg2rad(2.5);
% Idealized default until sensor error specifications are available.
RadarParams.MeasCovar = zeros(3,3);
end
