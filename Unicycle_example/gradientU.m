function fu = gradientU(x,u,dt)

du1 = 0.0001;
du2 = 0.0001;
dU = repmat([du1 du2],3,1);

du1 = [du1 0].';
du2 = [0 du2].';

f_old = repmat(dynamics_unicycle(x,u,dt),1,2);
f_new = [dynamics_unicycle(x,u+du1,dt) dynamics_unicycle(x,u+du2,dt)];
fu = (f_new-f_old)./dU;

end