%% DATA
% Reference: Hummingbird

% Inertial Data:
m = 0.5;        % [Kg]              % Mass
    
A = 2.32e-3;    % [Kg / m^2]        % Principal Moment of Inertia X
B = 2.32e-3;    % [Kg / m^2]        % Principal Moment of Inertia Y
C = 4.00e-3;    % [Kg / m^2]        % Principal Moment of Inertia Z
I = diag([A, B, C]);                % Principal Inertia Matrix

% Geometric Data

L= 0.2;        % [m]                % Arm length

% Torque and Lift Coefficients
k_f = 6.11e-8;  % [N / rpm]         % Lift Coefficient                  
k_m = 1.5e-9;   % [N*m / rpm]       % Torque Coefficient

% Gravitational Data
g = -9.81;      % [m/s^2]           % gravity acceleration


%% Save data in a structure Data type
copter_data = struct();

% Inertial Data
copter_data.mass = m;
copter_data.principal_intertia_moments = [A, B, C];

% Geometric Data
copter_data.arm = L;

% Dynamics Data
copter_data.lift_coeff = k_f;
copter_data.torque_coeff = k_m;
copter_data.g = g;

% Save the data structure in the "Variables" folder
save('Variables/Data.mat', 'copter_data');





