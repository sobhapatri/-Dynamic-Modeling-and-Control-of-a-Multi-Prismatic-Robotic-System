% Clear workspace and close all figures
clear;
clc;

% Request the number of prismatic joints
n = input('Enter the number of prismatic joints: ');

% Initialize symbolic variables for DH parameters
theta = sym('theta', [1 n], 'real');  % Joint displacement (prismatic)
a = sym('a', [1 n]);                  % Link lengths
alpha = sym('alpha', [1 n]);          % Link twists
d = sym('d', [1 n]);                  % Link offsets

% Get user input for DH parameters
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

% Compute transformation matrices and Z-axes
T = sym(eye(4));  % Base transformation matrix
Z = sym(zeros(3, n + 1));  % Z-axis for each joint, including the base which is [0;0;1]
Z(:, 1) = [0; 0; 1];  % Base frame Z-axis
transformation_matrices = cell(1, n);  % Store transformation matrices
PL = sym(zeros(3, n));  % Midpoints of each link
PM = sym(zeros(3, n));  % Motor positions
Jp = cell(1, n);  % Initialize cell array for linear Jacobian
Jol = sym(zeros(3, n)); % Angular Jacobian for links
Jpm = cell(1, n); % Initialize cell array for motor angular Jacobian
Jom = cell(1, n); % Initialize cell array for motor other Jacobian

% Calculate transformation matrices, Z-axes, and Jacobians
for i = 1:n
    Ti = DH_transform(a(i), alpha(i), d(i), theta(i));  % Individual transformation matrix
    T = T * Ti;  % Cumulative transformation matrix
    
    % Store the transformation matrix
    transformation_matrices{i} = T;
    
    % Extract Z-axis
    Z(:, i+1) = T(1:3, 3);  % Z-axis for prismatic joints
    
    % Calculate midpoints (PL) and motor positions (PM)
    P = T(1:3, 4);  % Position vector
    PL(:, i) = P / 2;  % Midpoint of each link
    PM(:, i) = P;  % Motor position
    
    % Fill the linear Jacobian
    Jp{i} = zeros(3, n);  % Initialize a 3x3 matrix with zeros
    for j = 1:i
        Jp{i}(:, j) = Z(:, j);  % Accumulate Z-axes for prismatic joints
    end

    % Angular Jacobian for links (always zero matrix for prismatic joints)
    Jol(:,:,i) = zeros(3,n); % 3x3 zero matrix

   % Angular Jacobian for motors
if i == 1
    Jpm{i} = zeros(3, n); % First motor Jacobian is a 3xN zero matrix
else
    Jpm{i} = zeros(3, n); % Initialize with zeros
    for j = 1:i-1
        Jpm{i}(:, j) = Z(:, j); % Fill with Z axes from previous joints
    end
    % Remaining columns should remain zero which are already initialized
end
    
    % Other Jacobian for motors
    Jom{i} = zeros(3, n); % Initialize with zeros
    for j = 1:n
        if j == i
            Jom{i}(:, j) = gear_ratio(i) * Z(:, j); % Fill with scaled Z-axis for the motor joint
        else
            Jom{i}(:, j) = zeros(3, 1); % Fill with zeros for other joints
        end
    end
end

% Display transformation matrices
disp('Transformation Matrices:');
for i = 1:n
    disp(['T0', num2str(i), ':']);
    disp(transformation_matrices{i});
end

% Display Z-axes
disp('Z-axes:');
for i = 1:n+1
    disp(['Z', num2str(i-1), ':']);
    disp(Z(:, i));
end

% Display midpoints (PL)
disp('Midpoints (PL):');
for i = 1:n
    disp(['PL', num2str(i), ':']);
    disp(PL(:, i));
end

% Display motor positions (PM)
disp('Motor Positions (PM):');
for i = 1:n
    disp(['PM', num2str(i), ':']);
    disp(PM(:, i));
end

% Display linear Jacobian (Jp) for each joint
disp('Linear Jacobian (Jp):');
for i = 1:n
    disp(['Jp for Joint ', num2str(i), ':']);
    disp(Jp{i});
end

% Display angular Jacobian for links
disp('Angular Jacobian for Links (Jol):');
for i = 1:n
    disp(['Jol for Joint ', num2str(i), ':']);
    disp(Jol(:,:,i));
end

% Display angular Jacobian for motors
disp('Angular Jacobian for Motors (Jpm):');
for i = 1:n
    disp(['Jpm for Motor ', num2str(i), ':']);
    disp(Jpm{i});
end

% Display other Jacobian for motors
disp('Other Jacobian for Motors (Jom):');
for i = 1:n
    disp(['Jom for Motor ', num2str(i), ':']);
    disp(Jom{i});
end

% Calculate B matrix
B = Get_B_Matrix(mass_link, mass_motor, inertia_link, inertia_motor, Jp, Jol, Jpm, Jom, n);

% Display B matrix
disp('B matrix:');
disp(B);


% Calculate g matrix
g = CalculateGravityMatrix(n, mass_link, mass_motor, Jp, Jpm);

% Display g matrix
disp('g matrix:');
disp(g);
disp(size(g));
ddot_d = sym('ddot_d', [n, 1]); % Define as a column vector

% Define Q.. matrix
ddot_Q = sym('ddot_d', [n, 1]); % Symbolic column vector

% Calculate Tou
Tou = B * ddot_Q + g;

% Display Tou
disp('Tou matrix:');
disp(Tou);

% Display individual elements of Tou using a loop
for i = 1:n
    disp(['Tou_', num2str(i), ':']);
    disp(Tou(i));
end

% Define time vector
t = linspace(0, 10, 100);  % Time from 0 to 10 seconds

% Define or calculate q, q_dot, and q_ddot
% This is an example where q is a simple linear function of time
q = linspace(0, 1, 100);  % Joint displacements
q_dot = gradient(q, t);   % Approximate velocity
q_ddot = gradient(q_dot, t);  % Approximate acceleration

% Plot for Joint Displacement
figure;
plot(t, q, 'LineWidth', 2);
title('Joint Displacement $q(t)$', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Displacement (m)');

% Plot for Joint Velocity
figure;
plot(t, q_dot, 'LineWidth', 2);
title('Joint Velocity $\dot{q}(t)$', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

% Plot for Joint Acceleration
figure;
plot(t, q_ddot, 'LineWidth', 2);
title('Joint Acceleration $\ddot{q}(t)$', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

save('robot_params.mat', 'T', 'Z', 'Jp', 'Jol', 'Jpm', 'Jom', 'B', 'g');

% Define the DH transformation matrix function
function T = DH_transform(a, alpha, d, theta)
    % Denavit-Hartenberg transformation for prismatic joints
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

function B = Get_B_Matrix(mass_link, mass_motor, inertia_link, inertia_motor, Jp, Jol, Jpm, Jom, n)
    B = sym(zeros(n)); % Initialize B as an nxn matrix
    for i = 1:n
    % Compute MliJpliTJpli term
    Mli = mass_link(i) * eye(3); % Inertia matrix for link i
    Jpli = Jp{i}; % Linear Jacobian for link i
    Term_1 = Jpli.' * Mli * Jpli;

    % Compute Joli*Ri*Ili*RiT*Joli term
    Joli = Jol(:, :, i); % Angular Jacobian for link i
    Ili = inertia_link{i}; % Inertia matrix for link i
    Term_2 = Joli.' * Ili * Joli;

    % Compute MmiJpmiT*Jpmi term
    Mmi = mass_motor(i) * eye(3); % Inertia matrix for motor i
    Jpmi = Jpm{i}; % Angular Jacobian for motor i
    Term_3 = Jpmi.' * Mmi * Jpmi;

    % Compute JomiT*Rmi*Imi*RmiTJomi term
    Jomi = Jom{i}; % Other Jacobian for motor i
    Imi = inertia_motor{i}; % Inertia matrix for motor i
    Term_4 = Jomi.' * Imi * Jomi;

    % Add all terms to B
    B = B + Term_1 + Term_2 + Term_3 + Term_4;
    end
   


end

function g = CalculateGravityMatrix(n, mass_link, mass_motor, Jp, Jpm)
    % Define g0
    g0 = [0; 0; -9.8]; % Gravity vector

    % Initialize g matrix
    g = sym(zeros(n, 1));

    % Calculate gi(q)
    for i = 1:n
        gi_q = -(mass_link(i) * g0.' * Jp{i} + mass_motor(i) * g0 .'* Jpm{i});
        g = g + gi_q;
    end
end