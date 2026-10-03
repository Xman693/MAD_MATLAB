 

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


function x_next = RK4(f, t, x, dt)

k1 = f(t, x);

k2 = f(t + dt/2, ...
    x + dt*k1/2);

k3 = f(t + dt/2, ...
    x + dt*k2/2);

k4 = f(t + dt, ...
    x + dt*k3);

x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

end