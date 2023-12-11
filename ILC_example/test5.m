% Plant model
plant_num = [8.331499999999999e-04 -0.002411635990000 0.001646764298800 0.001386774757085 -0.002226794738657 7.735179924151595e-04];
plant_den = [1 -5.7154 13.723534400000000 -17.734220581599999 13.016931722320001 -5.148338416112000 0.857492875392000];
plant_tf = tf(plant_num, plant_den, Ts);

% Feedback controller
feedback_num = [12.33590000000000 -35.651984590000005 34.340092218032005 -11.023646014692989];
feedback_den = [1 -2.832900000000000 2.672854940000000 -0.839948725176000];
fback_tf = tf(feedback_num, feedback_den, Ts);

% Get state space values for ILC
[Ag, Bg, Cg, Dg] = ssdata(plant_tf);
[Af, Bf, Cf, Df] = ssdata(fback_tf);

% Initialize vectors to store simulation results
num_steps = length(t);
x_plant = zeros(size(Ag, 1), num_steps);
x_feedback = zeros(size(Af, 1), num_steps);
y = zeros(1, num_steps);

% IMC Simulation with input first through feedback
for k = 1:num_steps
    
    % Feedback controller output
    feedback_output = Cf * x_feedback(:, k) + Df * Rj(k);
    
    % Plant output
    plant_output = Cg * x_plant(:, k) + Dg * feedback_output;
    
    % Closed-loop system output
    y(k) = plant_output;
    
    % Update states for the next time step
    x_feedback(:, k + 1) = Af * x_feedback(:, k) + Bf * Rj(k);
    x_plant(:, k + 1) = Ag * x_plant(:, k) + Bg * feedback_output;
end


   