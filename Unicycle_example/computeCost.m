function cost = computeCost(L,Q,R,x,u,xf,N,dt)

cost = 0.5*(x(:,end)-xf).'*L*(x(:,end)-xf);

for i=1:N-1
    cost = cost + 0.5*dt*((x(:,i)-xf).'*Q*(x(:,i)-xf) + u(:,i).'*R*u(:,i));
end

end