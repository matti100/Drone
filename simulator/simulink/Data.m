%% DATA

%% Problem initialization
params = struct();

% Inertial Data:
params.m = 1.25;            % [Kg]              % Mass
params.m = 0.8;
params.Ix = 0.0232;          % [Kg / m^2]        % Principal Moment of Inertia X
params.Ix = 15.67*1e-3;
params.Iy = 0.0232;         % [Kg / m^2]        % Principal Moment of Inertia Y
params.Iy = 15.67*1e-3;
params.Iz = 0.0468;         % [Kg / m^2]        % Principal Moment of Inertia Z
params.Iz = 28.34*1e-3;
params.I = diag([params.Ix, params.Iy, params.Iz]);
% Principal Inertia Matrix

% Geometric Data
params.armLength = 0.265;   % [m]                % Arm length
params.armLength = 0.3;
params.bodySize = 0.2;      % [m]                % body size
params.rotRad = 0.1;        % [m]                % rotor radius

% Torque and Lift Coefficients
k_f = 6.11e-8;   % [N / rpm]         % Lift Coefficient
params.k_f = 0.02;
params.k_f = 192.32*1e-7;
% params.k_f = (k_f)*60/6.28;                % [N/rad/s]
k_m = 1.5e-9;    % [N*m / rpm]       % Torque Coefficient
params.k_m = 0.01;
params.k_m = 4.003*1e-7;
% params.k_m = (k_m)*60/6.28;                % [N/rad/s]

% Gravitational Data
params.g = 9.81;        % [m/s^2]            % Gravity acceleration

% Time interval
params.dt = 0.01;       % [s]                % Time interval

% Initial Condition
x0 = zeros(12,1);
x0(1) = 0;
x0(2) = 0;
x0(3) = 1;
x0(7) = 0;
x0(8) = 0;
x0(9) = 0;

% Desidered Position
desideredState = struct();
desideredState.rDes = [1, -0.5, 2]';      % [m]
desideredState.attDes = [0, 0, pi/2]';





