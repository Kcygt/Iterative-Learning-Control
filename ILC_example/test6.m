% Iterative Learning Control (ILC) Example in MATLAB

% Define system parameters
A = [1 0.1; 0 1];  % System matrix
B = [0; 1];       % Input matrix
C = eye(2);       % Output matrix

% Define desired trajectory (reference signal)
T = 10;           % Total time
dt = 0.1;         % Time step
t = 0:dt:T;       % Time vector
xd = sin(t);      % Desired trajectory

% Initialize variables
x0 = [0; 0];      % Initial state
u = zeros(size(t)); % Control input

% Iterative Learning Control
iterations = 3;   % Number of iterations

for k = 1:iterations
    x = x0;       % Reset state to initial condition
    
    for i = 1:length(t)
        % Error computation
        e = xd(i) - C*x;
        
        % Control law (proportional controller)
        u(i) = -B'*inv(A)*C'*e;
        
        % System simulation
        x = A*x + B*u(i);
    end
    
    % Update initial condition for the next iteration
    x0 = x;
    
    % Plot actual trajectory for each iteration
    figure;
    plot(t, xd, 'k--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');
    hold on;
    plot(t, C*x0, 'b-', 'LineWidth', 2, 'DisplayName', ['Actual Trajectory - Iteration ', num2str(k)]);
    xlabel('Time');
    ylabel('Output');
    legend('show');
    title('Iterative Learning Control Example');
    grid on;
end
