% Clear workspace and close all figures
clear;
clc;

% Define the number of joints (prismatic)
n = input('Enter the number of prismatic joints: ');
% Initialize arrays for DH parameters and mechanical properties
theta = sym('theta', [1 n], 'real');  % Joint displacement (prismatic)
a = sym('a', [1 n]);                  % Link lengths
alpha = sym('alpha', [1 n]);          % Link twists
d = sym('d', [1 n]);                  % Link offsets

% Get user input for DH parameters if not loaded from file
for i = 1:n
    a(i) = input(['Enter link length a', num2str(i), ': ']);
    alpha(i) = deg2rad(input(['Enter link twist alpha', num2str(i), ' (degrees): ']));
    d(i) = input(['Enter link offset d', num2str(i), ': ']);
    theta(i) = deg2rad(input(['Enter joint displacement theta', num2str(i), ' (degrees): ']));
end

% Initialize arrays to store user inputs for each link
mass_link = zeros(1, n);
inertia_link = cell(1, n); % Initialize cell array for inertia matrix for links

% Request user input for link masses and inertias
for i = 1:n
    mass_link(i) = input(['Enter mass of link ', num2str(i), ' (kg): ']);
    inertia_val = input(['Enter inertia vector [Ixx Iyy Izz] for link ', num2str(i), ': ']);
    inertia_link{i} = diag(inertia_val); % Diagonal matrix with inertia values as diagonal elements
end

% Initialize arrays to store user inputs for each motor
mass_motor = zeros(1, n);
inertia_motor = cell(1, n); % Initialize cell array for inertia matrix for motors
gear_ratio = zeros(1, n);    % Initialize array for gear ratios

% Request user input for motor masses, inertias, and gear ratios
for i = 1:n
    mass_motor(i) = input(['Enter mass of motor ', num2str(i), ' (kg): ']);
    disp(['Enter inertia vector [Ixx Iyy Izz] for motor ', num2str(i), ': ']);
    inertia_val = input('');
    inertia_motor{i} = diag(inertia_val); % Diagonal matrix with inertia values as diagonal elements
    gear_ratio(i) = input(['Enter gear ratio for motor ', num2str(i), ': ']);
end

% Impedance control gains
Kp = input('Enter the proportional stiffness Kp: ');
Kd = input('Enter the derivative stiffness Kd: ');

% Simulation parameters
dt = 0.01;            % Time step
total_time = 5;       % Total simulation time
num_steps = total_time / dt;  % Number of simulation steps
time = linspace(0, total_time, num_steps); % Time vector

% Define desired trajectory
x_desired = 1 * sin(0.5 * time); % Simple sinusoidal trajectory

% Initialize arrays for simulation data
x_actual = zeros(1, num_steps); % Actual end-effector positions
f_contact = zeros(1, num_steps); % Contact forces experienced
f_impedance = zeros(1, num_steps); % Initialize impedance force for each time step

% Environmental interaction parameters
f_external = 5 * sin(1 * time); % External force example (sinusoidal)

% Assume k as joint stiffness for simplicity, define it properly based on your setup
% User input for mechanical property 'k'
k = input('Enter the joint stiffness or equivalent mechanical property k: ');
% Simulation loop
for i = 1:num_steps
    % Control input from inverse dynamics (Impedance control)
    if i == 1
        theta_dot = zeros(n, 1);  % Initialize theta_dot for the first step
        dx = 0;  % No previous data to calculate derivative
        f_impedance = 0;  % Initialize impedance force to zero
    elseif i == 2
        % Only one previous step, so use it directly without differencing
        x_error = x_desired(i) - x_actual(i-1);
        dx = (x_actual(i-1) - 0) / dt;  % Assuming initial position was at 0
        f_impedance = Kp * x_error - Kd * dx;  % Calculate impedance force
    else
        x_error = x_desired(i) - x_actual(i-1);
        dx = (x_actual(i-1) - x_actual(i-2)) / dt;
        f_impedance = Kp * x_error - Kd * dx;  % Calculate impedance force
    end

    f_total = f_impedance + f_external(i);  % Total force calculation
    theta_dot = f_total / k;  % Update joint velocities

    % Update joint positions
    theta = theta + theta_dot * dt;

    % Calculate actual end-effector position
    x_actual(i) = sum(theta);  % Sum of joint displacements as end-effector position approximation

    % Store contact force
    f_contact(i) = f_external(i) - f_impedance;
end
% Plotting desired vs. actual end-effector position
figure;  % This creates a new figure window for the positions
plot(time, x_desired, 'r-', 'DisplayName', 'Desired Position', 'LineWidth', 2);
hold on;
plot(time, x_actual, 'b--', 'DisplayName', 'Actual Position', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector Position');
legend show;

% Plotting end-effector contact forces in a separate figure
figure;  % This creates another new figure window for the forces
plot(time, f_contact, 'k-', 'DisplayName', 'Contact Force', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Force (N)');
title('End-Effector Contact Forces');
legend show;

