% https://uk.mathworks.com/matlabcentral/answers/59641-implementation-of-iterative-learning-control-in-matlab-simulink
close all
clear all
clc
Ts = 0.1;
num = 100;
den = [1  120];
sysc = tf(num,den);
sysd = c2d(sysc,Ts,'ZOH');

% Define the numerator and denominator of the systems and Tranfer Function
plant_num = [8.331499999999999e-04 -0.002411635990000 0.001646764298800 0.001386774757085 -0.002226794738657 7.735179924151595e-04 ];
plant_den =  [1 -5.7154 13.723534400000000 -17.734220581599999 13.016931722320001  -5.148338416112000 0.857492875392000];
plant_tf = tf(plant_num,plant_den,Ts);

% Define the numerator and denominator of the Feedback controller and Tranfer Function
feedback_num = [12.33590000000000 -35.651984590000005 34.340092218032005  -11.023646014692989];
feedback_den = [1    -2.832900000000000    2.672854940000000    -0.839948725176000];
fback_tf = tf(feedback_num,feedback_den,Ts);



% Get state space values for ILC
[Ag, Bg, Cg, Dg] = ssdata(plant_tf);
[Ac, Bc, Cc, Dc] = ssdata(fback_tf);

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
jmax = 5;

l0 = 0.95; L = l0 * eye(N,N);
q0 = 1.00; Q = q0 * eye(N,N);

Uj = zeros(N,1); Ujold = Uj;
Ej = zeros(N,1); Ejold = Ej;
Yj = zeros(N,1);
UCj = zeros(N,1);
UFj = zeros(N,1);

% G0 not formulated as initial condition is 0
% Formulate G
Gvec = zeros(N,1);
Cvec = zeros(N,1);

rVec = ((r-1):(N-n0-1))';

for ii = 1:length(rVec)
    Gvec(ii) = Cg*Ag^rVec(ii)*Bg;
    Cvec(ii) = Cc*Ac^rVec(ii)*Bc;
end

G = toeplitz(Gvec);
C = toeplitz(Cvec);



% Run ILC and plot the response for each iteration
for ii = 1:jmax
    for jj = 1:length(rVec)

        UFj(jj) = q0*(Ujold(jj) + l0*Ejold(jj));
        for xx=1:jj
            UCj(xx) = C(xx,:) * Ej;
        end
        Uj = UCj + UFj;

        for xx=1:jj
            Yj(xx) = G(xx,:) * Uj;
        end

        Ej(jj) = Rj(jj) - Yj(jj);


        Ej(1) = 0;
        Ejold(jj) = Ej(jj);
        Ujold(jj) = Uj(jj);

    end
    plotter(ii,t,Ej,Yj,Uj,Rj,U)
end
