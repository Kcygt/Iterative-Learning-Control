close all 
clear all
clc

% https://juliacontrol.github.io/ControlSystems.jl/dev/examples/ilc/
G    = doubleMassModel(1.0);
Gact = doubleMassModel(1.5); % 50 more load than modeled

bodeplot(G, Gact);
legend('G model', 'G actual');

% Define the PID controller and plant transfer function
Kp = 10;
Ki = 1;
Kd = 1;
numerator = 1;
denominator = [0.02, 1];

C = pid(Kp, Ki, Kd) * tf(numerator, denominator);

% Sample time
Ts = 0.02;

% Discretize the closed-loop system
Gc = c2d(feedback(G*C,1), Ts);
Gcact = c2d(feedback(Gact*C,1), Ts);

% Plot the closed-loop step response
figure;
step(Gc, 10);
hold on;
step(Gcact, 10);
title('Closed-loop step response');
legend('Model', 'Actual');
hold off;


T = 3*pi;   % Duration
t = 0:Ts:T; % Time vector
r = zeros(length(t),1);

for i = 1:length(r)

    r(i) = funnysin(t(i))'; %|> Array # Reference signal
end
res = lsim(Gcact, r, t);

% L and Q filter

z = tf("z", Ts);
Q = c2d(tf(1, [0.05, 1]), Ts);
L = 0.9*z^1; % A more conservative and heuristic choice
L = 0.5*inv(Gc); % Make the scaling factor smaller to take smaller steps

% Create a figure for Bode plots
bodeplot(inv(Q), (1 - L*Gc),(1 - L*Gcact))



ilc(Gc, Q, L,r,t)
