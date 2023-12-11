% https://uk.mathworks.com/matlabcentral/answers/59641-implementation-of-iterative-learning-control-in-matlab-simulink
close all
clear all
clc
Ts = 0.001
% Define the numerator and denominator of the systems and Tranfer Function
plant_num = [8.331499999999999e-04 -0.002411635990000 0.001646764298800 0.001386774757085 -0.002226794738657 7.735179924151595e-04 ];
plant_den =  [1 -5.7154 13.723534400000000 -17.734220581599999 13.016931722320001  -5.148338416112000 0.857492875392000];
plant_tf = tf(plant_num,plant_den,Ts);

% Define the numerator and denominator of the Feedback controller and Tranfer Function
feedback_num = [12.33590000000000 -35.651984590000005 34.340092218032005  -11.023646014692989];
feedback_den = [1    -2.832900000000000    2.672854940000000    -0.839948725176000];
fback_tf = tf(feedback_num,feedback_den,Ts);


% Get state space values for ILC
[Ad, Bd, Cd, Dd] = ssdata(plant_tf);
[Af, Bf, Cf, Df] = ssdata(fback_tf);
% Initial condition x0, time range t - assume 1 second, pure time delay n0, relative
% degree r, and matrix size N
x0 = 0;
t = 0:Ts:0.6;
n0 = 0;
r = 1;
N = length(t);
% Define input vector U and reference J - Refernce = input for this example
% Rj = [0*ones(1,(N-1)/5) 1*ones(1,(N-1)/5) 1*ones(1,(N-1)/5) 1*ones(1,(N-1)/5) 0*ones(1,(N-1)/5+1)]';

Rj = zeros(size(t))';
Rj(t < 0.3) = 0;
Rj((t >= 0.3) & (t < 0.4)) = linspace(0, 1, sum((t >= 0.3) & (t < 0.4)));
Rj(t >= 0.4) = 1;

% ILC parameters
gamma = 0.1; % learning rate
max_iterations = 100; % maximum number of iterations

% Initialize variables
u_ilc = zeros(N, 1); % control input
x_state = x0; % state initialization

% ILC iteration
for k = 1:max_iterations
    % Calculate error at each iteration
    e = Rj - lsim(plant_tf, u_ilc, t);

    % Update control input using ILC
    u_ilc = u_ilc + gamma * lsim(fback_tf, e, t);

    % Simulate the plant with the updated control input
    y_ilc = lsim(plant_tf, u_ilc, t);

    % Update the state of the plant
    x_state = Ad * x_state + Bd * u_ilc(1) + Bd * n0 * Rj(1) - Bd * n0 * y_ilc(1);
end

% Plot the results
figure;
subplot(2,1,1);
plot(t, Rj, 'r', t, y_ilc, 'b');
title('Reference and Output');
legend('Reference', 'Output');

subplot(2,1,2);
plot(t, u_ilc, 'g');
title('Control Input');

