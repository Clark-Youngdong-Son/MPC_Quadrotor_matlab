clear; clc; close all;

parameters_unicycle;

N_original = N;

%Norminal input and state
u_nominal = zeros(2,N-1);
x_nominal = forwardSimulation(X0, u_nominal, N, dt);

%Real input and state
X = zeros(3,length(t));
X(:,1) = X0;
U = zeros(2,length(t)-1);
%%
%MPC loop
% for i=1:length(t)-1
for i=1:length(t)-1
    t_now = t(i);
    if(t_now > (tf-dt*(N_original-1)))
        N = N-1;                             %decrease N near the final time
        x_nominal = x_nominal(:,2:2+N-1);
        u_nominal = u_nominal(:,2:2+N-2);
    else
        if(i~=1)
            % Update nominal input and state
            x_nominal(:,1:end-1) = x_nominal(:,2:end);
            u_nominal(:,1:end-1) = u_nominal(:,2:end);
            u_nominal(:,end) = u_nominal(:,end-1);
            x_nominal(:,end) = dynamics_unicycle(x_nominal(:,end-1),u_nominal(:,end),dt);
        end
    end

    [x_nominal,u_nominal,cost] = SLQ_solve(x_nominal,u_nominal,L,Q,R,N,dt,Xf,iterMax_SLQ,iterMax_line); %solve a SLQ problem to find control input
    U(:,i) = u_nominal(:,1);
    X(:,i+1) = dynamics_unicycle(X(:,i),U(:,i),dt);
end

%Plot simulation result
figure(1);
subplot(1,3,1); plot(t, X(1,:),'LineWidth',2); hold on;
plot(t(end),Xf(1),'o','MarkerFaceColor','r');
xlabel('time'); title('x');
subplot(1,3,2); plot(t, X(2,:),'LineWidth',2); hold on;
plot(t(end),Xf(2),'o','MarkerFaceColor','r');
xlabel('time'); title('y');
subplot(1,3,3); plot(t, X(3,:),'LineWidth',2); hold on;
plot(t(end),Xf(3),'o','MarkerFaceColor','r');
xlabel('time'); title('\theta');

figure(2);
subplot(1,2,1); plot(t(1:end-1), U(1,:),'LineWidth',2); hold on;
xlabel('time'); title('v');
subplot(1,2,2); plot(t(1:end-1), U(2,:),'LineWidth',2); hold on;
xlabel('time'); title('\omega');