%% DATA
clear
clc
close all

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
params.k_f = 1.4*1e-5 / 9.54929658;
% params.k_f = (k_f)*60/6.28;                % [N/rad/s]
k_m = 1.5e-9;    % [N*m / rpm]       % Torque Coefficient
params.k_m = 0.01;
params.k_m = 4.003*1e-7;
params.k_m = 1.78*1e-6 / 9.54929658;
% params.k_m = (k_m)*60/6.28;                % [N/rad/s]

% Gravitational Data
params.g = 9.81;        % [m/s^2]            % Gravity acceleration

% Time interval
sampleTime = 0.01;       % [s]                % Time interval

% Initial Condition
x0 = zeros(12,1);
x0(1) = 0;
x0(2) = 0;
x0(3) = 1;
x0(7) = 0;
x0(8) = 0;
x0(9) = 0;

% Desidered Position
desiredState = struct();
desiredState.rDes = [1, -1, 2]';      % [m]
desiredState.attDes = [0, 0, 0]';

% PID gains
                                            % I set                 II set
kP = [20;           % -> kP_T                  20                     20
    -0.4;           % -> kP_phi                0
    0.4;            % -> kP_theta              0.4
    1;              % -> kP_R                  1                      1.55
    1;              % -> kP_P                  1                      1.55
    1.2];           % -> kP_Y                  0.1                  x 0.02

kI = [10;           % -> kI_T                  10                     10
    -0.09;          % -> kI_phi                0
    0.09;           % -> kI_theta              0.09
    0.13;           % -> kI_R                  0.13                   0.17
    0.13;           % -> KI_P                  0.13                   0.17
    0.03];          % -> KI_Y                  0.013                x 0.01

kD = [10;           % -> kD_T                  10                     10
    -0.25;          % -> kD_phi                0
    0.25;           % -> kD_theta              0.25
    0.66;           % -> kD_R                  0.66                   0.76
    0.66;           % -> kD_P                  0.66                   0.76
    0.9];           % -> kD_Y                  0.066                x 0.01

gains = gainBuilder(kP, kI, kD);

%% Estimator
sigma_acc = 2e-3;
sigma_gyro = 1e-3;

P_est = eye(12);
Q = eye(12).*1e-0;
R = diag([(sigma_acc^2).*ones(3,1); ...
            (sigma_gyro^2).*ones(3,1)]);

% % Measurements function model
% 
% f = @(x, u) [x(4:6);
%     (1/params.m).*rotMat(x)*[0; 0; u(1)] + [0; 0; -params.g];
%     x(10:12);
%     params.I \ ([u(2); u(3); u(4)] - cross(x(10:12), params.I*x(10:12)))
%     ];
% 
% acc = @(x, u) (1/params.m).*rotMat(x)*[0; 0; u(1)] + [0; 0; -params.g];
% 
% R_psi = @(x) [cos(x(9)),    -sin(x(9)),     0;
%     sin(x(9)),     cos(x(9)),     0;
%     0,             0,     1];
% 
% R_theta = @(x) [cos(x(8)),      0,      sin(x(8));
%     0,      1,              0;
%     -sin(x(8)),     0,       cos(x(8))];
% 
% R_phi = @(x) [1,        0,              0;
%     0,        cos(x(7)),      -sin(x(7));
%     0,        sin(x(7)),      cos(x(7))];
% 
% rotMat = @(x) R_psi(x) * R_theta(x) * R_phi(x);
% 
% h = @(x) [rotMat(x)' * (acc(x, u) + [0;0;-params.g]);
%     x(10);
%     x(11);
%     x(12)
%     ];


%% BUS Creator
clear elems;

% Dynamics BUS
elems(1) = Simulink.BusElement;
elems(1).Name = 'state_true';
elems(1).DataType = 'double';
elems(1).Dimensions = [12, 1];

elems(2) = Simulink.BusElement;
elems(2).Name = "acc_true";
elems(2).DataType = "double";
elems(2).Dimensions = [3, 1];

elems(3) = Simulink.BusElement;
elems(3).Name = "w_true";
elems(3).DataType = "double";
elems(3).Dimensions = [3, 1];

bus_dynamics = Simulink.Bus;
bus_dynamics.Elements = elems;

% OBC BUS
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'state_est';
elems(1).DataType = 'double';
elems(1).Dimensions = [12, 1];

elems(2) = Simulink.BusElement;
elems(2).Name = 'motor_cmd';
elems(2).DataType = 'double';
elems(2).Dimensions = [4, 1];

bus_OBC = Simulink.Bus;
bus_OBC.Elements = elems;

% Sensors BUS
clear elems;

