

function y = RadarToNed(dir, x, phi, theta)

% 0 --> Radar 2 NED
% 1 --> NED 2 Radar

u = phi + theta;

R = [cos(u) -sin(u); sin(u) cos(u) ];

    if dir == 0
    y = R * x;
    else
    y = R' * x;

    end
end
