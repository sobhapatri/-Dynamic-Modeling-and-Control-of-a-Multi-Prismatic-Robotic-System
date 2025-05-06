% Clear workspace and close all figures
clear;
clc;

% Request the number of prismatic joints
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
gear_ratio = zeros(1, n);

% Request user input for motor masses, inertias, and gear ratios
for i = 1:n
    mass_motor(i) = input(['Enter mass of motor ', num2str(i), ' (kg): ']);
    disp(['Enter inertia vector [Ixx Iyy Izz] for motor ', num2str(i), ': ']);
    inertia_val = input('');
    inertia_motor{i} = diag(inertia_val); % Diagonal matrix with inertia values as diagonal elements
    gear_ratio(i) = input(['Enter gear ratio for motor ', num2str(i), ': ']);
end

% Simulation and plotting setup
Kp = input('Enter the proportional stiffness Kp: ');
Kd = input('Enter the derivative stiffness Kd: ');
desired_x_val = input('Enter the desired end-effector position [x; y; z]: ');
dt = 0.01;  % Define the time step for the simulation

% Define initial current end-effector position and velocity
current_x_val = zeros(3, 1); % Ensure it's properly initialized as a 3-by-1 vector
current_x_dot_val = zeros(3, 1);

% Initialize arrays to store data for plotting
time = linspace(0, 10, 1000); % Time vector
desired_positions = repmat(desired_x_val, 1, length(time)); % Store desired positions
actual_positions = zeros(3, length(time));  % Store actual positions
contact_forces = zeros(3, length(time));     % Store contact forces

% Gravity and environmental interaction setup
g = 9.81; % gravitational acceleration (m/s^2)
gravity_forces = g * sum(mass_link); % total gravitational force
k_environment = 1000; % Stiffness of the environmental interaction
interaction_zone = [1; 0; 0]; % Example interaction point

% Compliance control law and plotting
for i = 1:length(time)
    % Gravity compensation
    gravity_compensation = [0; 0; gravity_forces]; % Simplified to act in z-direction
    
    % Environmental interaction
    environmental_force = [0; 0; 0];
    if norm(current_x_val - interaction_zone) < 0.1
        environmental_force = -k_environment * (current_x_val - interaction_zone);
    end

    % Compute control force
    force_control = zeros(3, 1);
    for j = 1:n
        Jp{j} = eye(3); % Simplified Jacobian; replace with actual computation if available
        force_control = force_control + Jp{j}' * (Kp * (desired_x_val - current_x_val) - Kd * current_x_dot_val);
    end

    % Sum of control and external forces
    total_force = force_control - gravity_compensation + environmental_force;

    % Calculate acceleration and update velocities and positions
    acceleration = total_force / (sum(mass_link) + sum(mass_motor));
    current_x_dot_val = current_x_dot_val + acceleration * dt;
    current_x_val = current_x_val + current_x_dot_val * dt;

    % Store data for plotting
    actual_positions(:, i) = current_x_val;
    contact_forces(:, i) = force_control;  % Logging only the control forces
end

% Figure for combined positions in x, y, z directions
figure;
plot(time, actual_positions(1, :), 'r', 'LineWidth', 2);
hold on;
plot(time, actual_positions(2, :), 'g', 'LineWidth', 2);
plot(time, actual_positions(3, :), 'b', 'LineWidth', 2);
plot(time, desired_positions(1, :), 'r--', 'LineWidth', 2);
plot(time, desired_positions(2, :), 'g--', 'LineWidth', 2);
plot(time, desired_positions(3, :), 'b--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position');
legend('Actual X', 'Actual Y', 'Actual Z', 'Desired X', 'Desired Y', 'Desired Z');
title('Desired vs Actual End-Effector Positions');

% Figure for forces in x, y, z directions
figure;
plot(time, contact_forces(1, :), 'r', 'LineWidth', 2);
hold on;
plot(time, contact_forces(2, :), 'g', 'LineWidth', 2);
plot(time, contact_forces(3, :), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Force');
legend('X Force', 'Y Force', 'Z Force');
title('End-Effector Contact Forces');
