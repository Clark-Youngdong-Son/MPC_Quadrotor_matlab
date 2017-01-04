function x_new = dynamics_slung_load(x_now, u_now, m_Q, m_L, l, I, dt)

%Position of the load
x_L = x_now(1);
y_L = x_now(2);
z_L = x_now(3);
%Roll, pitch, yaw
phi_Q = x_now(4);
theta_Q = x_now(5);
psi_Q = x_now(6);
R = vec2RotMat(phi_Q,theta_Q,psi_Q);
%Cable vector
p_x = x_now(7);
p_y = x_now(8);
p_z = x_now(9);

%Velocity of the load
x_L_dot = x_now(10);
y_L_dot = x_now(11);
z_L_dot = x_now(12);
%p,q,r (Bofy frame angular velocity)
p = x_now(13);
q = x_now(14);
r = x_now(15);

thrust = u_now(1);
M_x = u_now(2);
M_y = u_now(3);
M_z = u_now(4);

x_new = zeros(size(x_now)); %18X1 column vector

x_new(1:3) = [x_L y_L z_L].' + [x_L_dot y_L_dot z_L_dot].'*dt;
x_new(4:6) = [phi_Q theta_Q psi_Q].' + pqr2rpyDot([p q r],[phi_Q,theta_Q,psi_Q])*dt;

p_dot = cross([p q r],[p_x p_y p_z])*dt;
x_new(7:9) = [p_x p_y p_z].' + p_dot*dt;

x_new(10:12) = [x_Q_dot y_Q_dot z_Q_dot].' + ((dot([p_x p_y p_z].',(thrust*R*[0 0 1].')) - m_Q*l*dot(p_dot,p_dot))/(m_Q+m_L)*[p_x p_y p_z].'-[0 0 g].')*dt;
x_new(13:15) = [p q r].' + inv(I)*([M_x M_y M_z].'-cross([p q r].',(I*[p q r].')))*dt;

end