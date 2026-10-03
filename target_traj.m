function [t, target_traj] = target_traj(SimParams)
%TARGET_TRAJ Generate target trajectory using a constant-velocity model.

% Initial position and velocity (NED)
p0 = [100000; -50000];
v0 = [250; 0];
x0 = [p0; v0];

Tmax = SimParams.Tmax;
dt = SimParams.dt;

% Time vector, including Tmax exactly
N = floor(Tmax / dt);
t = (0:N)' * dt;
if t(end) < Tmax
    t = [t; Tmax];
end

% State history
target_traj = zeros(numel(t), numel(x0));
target_traj(1, :) = x0.';

% Integrate
x = x0;
for k = 1:(numel(t) - 1)
    dt_k = t(k + 1) - t(k);
    x = RK4(@cvDynamics, t(k), x, dt_k);
    target_traj(k + 1, :) = x.';
end

end

function xdot = cvDynamics(~, x)
% Constant-velocity model.

xdot = [x(3);
    x(4);
    0;
    0];
end
