% https://uk.mathworks.com/matlabcentral/answers/59641-implementation-of-iterative-learning-control-in-matlab-simulink
close all
clear all
clc
Ts = 0.01;
num = 100;
den = [1  120];
sysc = tf(num,den);
sysd = c2d(sysc,Ts,'ZOH');


% Get state space values for ILC
[Ad, Bd, Cd, Dd] = ssdata(sysd);

% Initial condition x0, time range t - assume 1 second, pure time delay n0, relative
% degree r, and matrix size N
x0 = 0;
t = 0:Ts:0.6;
n0 = 0;
r = 1;
N = length(t);

% Define input vector U and reference J - Refernce = input for this example
Rj= sin(20*t');
U = Rj;


% Set up ILC
jmax = 10;

l0 = 0.95; L = l0 * eye(N,N);
q0 = 1.00; Q = q0 * eye(N,N);

Uj = zeros(N,1); Ujold = Uj;
Ej = zeros(N,1); Ejold = Ej;
Yj = zeros(N,1);

% G0 not formulated as initial condition is 0
% Formulate G
Gvec = zeros(N,1);
rVec = ((r-1):(N-n0-1))';

for ii = 1:length(rVec)
    ApowVec = Ad^rVec(ii);
    Gvec(ii) = Cd*ApowVec*Bd;
end
G = toeplitz(Gvec);



% Run ILC and plot the response for each iteration
for ii = 1:jmax
    for jj = 1:length(rVec)
        Uj(jj) = q0*(Ujold(jj) + l0*Ejold(jj));
        
        for xx=1:jj
            Yj(xx) = G(xx,:) * Uj;
        end
        
        Ej(jj) = Rj(jj) - Yj(jj); Ej(1) = 0;
        Ejold(jj) = Ej(jj);
        Ujold(jj) = Uj(jj);

    end
    plotter(ii,t,Ej,Yj,Uj,Rj,U)
end
