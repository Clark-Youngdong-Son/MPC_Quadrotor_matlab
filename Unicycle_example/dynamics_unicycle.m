function x_new = dynamics_unicycle(x_now, u_now, dt)

%States
x     = x_now(1);
y     = x_now(2);
theta = x_now(3);

%Inputs
v = u_now(1);
w = u_now(2);

x_new = zeros(size(x_now));

x_new(1) = x + v*cos(theta)*dt;
x_new(2) = y + v*sin(theta)*dt;
x_new(3) = theta + w*dt;

end