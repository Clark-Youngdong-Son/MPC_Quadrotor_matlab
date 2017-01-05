function [x,u,cost] = backwardRiccati(P_tf,p_tf,L,Q,R,x,u,N,xf,dt,iterMax_line)
    P = zeros(3,3,N);
    P(:,:,N) = P_tf;
    p = zeros(3,N);
    p(:,N) = p_tf;
    K = zeros(2,3,N-1);
    l = zeros(2,N-1);
    
%backward update
for i=N-1:-1:1
    
    A = gradientX(x(:,i),u(:,i),dt);
    B = gradientU(x(:,i),u(:,i),dt);

    H = R + B.'*P(:,:,i+1)*B;
    H_inv = inv(H);
    G = B.'*P(:,:,i+1)*A;
    
    P(:,:,i) = Q + A.'*P(:,:,i+1)*A - G.'*H_inv*G;
    
    r = R*u(:,i);
    g = r + B.'*p(:,i+1);
    q = Q*(x(:,i)-xf);
    p(:,i) = q + A.'*p(:,i+1) - G.'*H_inv*g;
    
    K(:,:,i) = -H_inv*G;
    l(:,i) = -H_inv*g;
    
end


%Line search
cost_l_old = computeCost(L,Q,R,x,u,xf,N,dt);
x_old = x;
x_new = x; %dummy
u_old = u;
u_new = u; %dummy

alpha = 1;
for i=1:iterMax_line
    for j=1:N-1
        u_new(:,j) = u_old(:,j) + alpha*l(:,j) + K(:,:,j)*(x_old(:,j)-x(:,j));
    end
x_new = forwardSimulation(x_old(:,1),u_new,N,dt);
cost_l_new = computeCost(L,Q,R,x_new,u_new,xf,N,dt);
delta = cost_l_new - cost_l_old;
% subplot(1,3,1); plot(1:N, x_new(1,:)); hold on;
% subplot(1,3,2); plot(1:N, x_new(2,:)); hold on;
% subplot(1,3,3); plot(1:N, x_new(3,:)); hold on;
% drawnow;

if(delta>=0) 
    alpha = alpha*0.9;
    x_new = x_old;
    u_new = u_old;
    cost_l_new = cost_l_old;
    %    break;
elseif(delta>-0.05)
    break;
else
    alpha = alpha*0.9;
    x_old = x_new;
    u_old = u_new;
    cost_l_old = cost_l_new;
end

end
fprintf('=====================line_iter : %d======================\n',i);
x = x_new;
u = u_new;
cost = cost_l_new;
end