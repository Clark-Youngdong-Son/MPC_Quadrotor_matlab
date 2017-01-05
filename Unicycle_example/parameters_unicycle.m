%% MPC parameters
%time parameters
dt = 0.01;
tf = 2;
t = 0: dt : tf;
timeHorizon = 0.5;
N = timeHorizon/dt+1;

%state parameters
X0 = [0 0 0].';
Xf = [3 3 pi/6].';

%SLQ parameters
iterMax_SLQ = 200;    %maximum iteration of SLQ
iterMax_line = 200;   %maximum iteration of line search

L = diag([4 4 1])*10;         %final state weighting matrix
Q = L;            %intermediate state weighting matrix
R = eye(2);            %intermediate input weighting matrix