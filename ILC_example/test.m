Ts = 0.001;

% Define the numerator and denominator of the systems and Transfer Function
plant_num = [8.331499999999999e-04 -0.002411635990000 0.001646764298800 0.001386774757085 -0.002226794738657 7.735179924151595e-04];
plant_den = [1 -5.7154 13.723534400000000 -17.734220581599999 13.016931722320001 -5.148338416112000 0.857492875392000];
plant_tf = tf(plant_num, plant_den, Ts);

% Define the numerator and denominator of the Feedback controller and Transfer Function
feedback_num = [12.33590000000000 -35.651984590000005 34.340092218032005 -11.023646014692989];
feedback_den = [1 -2.832900000000000 2.672854940000000 -0.839948725176000];
fback_tf = tf(feedback_num, feedback_den, Ts);
system_tf = feedback(plant_tf * fback_tf, 1);

t = 0:Ts:0.6;

% Define input vector U and reference J - Reference = input for this example
Rj = zeros(size(t))';
Rj(t < 0.3) = 0;
Rj((t >= 0.3) & (t < 0.4)) = linspace(0, 1, sum((t >= 0.3) & (t < 0.4)));
Rj(t >= 0.4) = 1;

% Initialize the system response vector
num_states = length(plant_den) - 1;
X = zeros(num_states, 1);
system_response = zeros(size(t));

% Simulate the system manually
for k = 1:length(t)
    % Calculate the output at the current time step
    Y = plant_num * Rj(k) - plant_den(2:end) * X;

    % Update the state vector for the next iteration
    X = [Y; X(1:end)'];

    % Store the system response at the current time step
    system_response(k) = Y(1);  % Take the first element of Y as it is a scalar
end

% Plot the system response
plot(t, system_response);
xlabel('Time (s)');
ylabel('System Response');
title('System Response Without lsim');

