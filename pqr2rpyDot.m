function [rpyDot,conversion] = pqr2rpyDot(pqr,rpy)

p = pqr(1);
q = pqr(2);
r = pqr(3);
phi = rpy(1);
theta = rpy(2);
% psi = rpy(3); %Useless

conversion = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
    0 cos(phi) -sin(phi);
    0 sin(phi)*sec(theta) cos(phi)*sec(theta)];

rpyDot = conversion*[p q r].';

end