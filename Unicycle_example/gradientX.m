function fx = gradientX(x,u,dt)

dx1 = 0.0001;
dx2 = 0.0001;
dx3 = 0.0001;
dX = repmat([dx1 dx2 dx3],3,1);

dx1 = [dx1 0 0].';
dx2 = [0 dx2 0].';
dx3 = [0 0 dx3].';

f_old = repmat(dynamics_unicycle(x,u,dt),1,3);
f_new = [dynamics_unicycle(x+dx1,u,dt) dynamics_unicycle(x+dx2,u,dt) dynamics_unicycle(x+dx3,u,dt)];
fx = (f_new-f_old)./dX;

end