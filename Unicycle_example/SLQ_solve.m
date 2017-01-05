function [x,u,cost] = SLQ_solve(x,u,L,Q,R,N,dt,xf,iterMax_SLQ,iterMax_line)

cost_old = computeCost(L,Q,R,x,u,xf,N,dt);

x_old = x;
u_old = u;

for i=1:iterMax_SLQ
    P_tf = L;             %final value which depends on the cost function
    p_tf = L*(x_old(:,N)-xf); %final value which depends on the cost function
    
    [x_new,u_new,cost_new] = backwardRiccati(P_tf,p_tf,L,Q,R,x_old,u_old,N,xf,dt,iterMax_line);
    delta = cost_new - cost_old;
    if(delta>=0)
        break;
    end
    
    x_old = x_new;
    u_old = u_new;
    cost_old = cost_new;
end
fprintf('=========SLQ_iter : %d=============================\n',i);
x = x_old;
u = u_old;
cost = cost_old;