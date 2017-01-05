function x = forwardSimulation(x0, u, N, dt)
x = zeros(3,N);
x(:,1) = x0;

for i=1:N-1
   x(:,i+1) = dynamics_unicycle(x(:,i),u(:,i),dt); 
end

end