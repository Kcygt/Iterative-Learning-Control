Ts = 0.001;
t = 0:Ts:1;

Rj = zeros(size(t));
Rj(t < 0.3) = 0;
Rj((t >= 0.3) & (t < 0.4)) = linspace(0, 1, sum((t >= 0.3) & (t < 0.4)));
Rj(t >= 0.4) = 1;

plant_num = [8.331499999999999e-04 -0.002411635990000 0.001646764298800 0.001386774757085 -0.002226794738657 7.735179924151595e-04];
plant_den = [1 -5.7154 13.723534400000000 -17.734220581599999 13.016931722320001 -5.148338416112000 0.857492875392000];
plant_tf = tf(plant_num, plant_den, Ts);

feedback_num = [12.33590000000000 -35.651984590000005 34.340092218032005 -11.023646014692989];
feedback_den = [1 -2.832900000000000 2.672854940000000 -0.839948725176000];
fback_tf = tf(feedback_num, feedback_den, Ts);

% Internal Model Control (IMC) Simulation with Feedback
sys_open_loop = plant_tf * fback_tf;
sys_closed_loop = feedback(sys_open_loop, 1);

% Simulate the closed-loop system
[y, t] = lsim(sys_closed_loop, Rj, t);

% Plot the results
figure;
subplot(2, 1, 1);
plot(t, Rj, 'b-', 'LineWidth', 1.5);
title('Reference Input');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(2, 1, 2);
plot(t, y, 'r-', 'LineWidth', 1.5);
title('Output Response');
xlabel('Time (s)');
ylabel('Amplitude');

sgtitle('Internal Model Control (IMC) Simulation with Feedback');
