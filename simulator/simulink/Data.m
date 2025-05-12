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
r0 = [0; 0; 1];
v0 = [0; 0; 0];
att0 = [0; 0; 0];
w0 = [0; 0; 0];

% Desidered Position
desiredState = struct();
desiredState.rDes = [-1, 1, 2]';      % [m]
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
sigma_gyro = 0.005 * sqrt(1/sampleTime) * pi/180;
sigma_acc = 400*(1e-6) * sqrt(1/sampleTime) / params.g;

P_est = 0.*eye(12);
Q = 0.*eye(12).*1e-0;
R = diag([(sigma_acc^2).*ones(3,1); ...
    (sigma_gyro^2).*ones(3,1)]);

% Linear system A matrix
A = zeros(12,12);
A(1, 4) = 1;
A(2, 5) = 1;
A(3, 6) = 1;
A(4, 7) = params.g*sin(desiredState.attDes(3));
A(4, 8) = params.g*cos(desiredState.attDes(3));
A(5, 7) = -params.g*cos(desiredState.attDes(3));
A(5, 8) = params.g*sin(desiredState.attDes(3));
A(7, 10) = 1;
A(8, 11) = 1;
A(9, 12) = 1;

% Linear system B matrix
B = zeros(12, 4);
B(6, 1) = 1/params.m;
B(10, 2) = 1/params.Ix;
B(11, 3) = 1/params.Iy;
B(12, 4) = 1/params.Iz;

% Linear system C matrix
C = zeros(6, 12);
C(1, 4) = 1/sampleTime;
C(2, 5) = 1/sampleTime;
C(3, 6) = 1/sampleTime;
C(4, 10) = 1;
C(5, 11) = 1;
C(6, 12) = 1;

% Linear system D matrix
D = zeros(6,4);

alpha = 0.98;

%% BUS Creator
clear elems;

% Dynamics BUS
elems(1) = Simulink.BusElement;
elems(1).Name = 'state_true';
elems(1).DataType = 'double';
elems(1).Dimensions = [12, 1];

elems(2) = Simulink.BusElement;
elems(2).Name = "acc_body";
elems(2).DataType = "double";
elems(2).Dimensions = [3, 1];

elems(3) = Simulink.BusElement;
elems(3).Name = "w_true";
elems(3).DataType = "double";
elems(3).Dimensions = [3, 1];

elems(4) = Simulink.BusElement;
elems(4).Name = "acc_true";
elems(4).DataType = "double";
elems(4).Dimensions = [3, 1];

bus_dynamics = Simulink.Bus;
bus_dynamics.Elements = elems;

% OBC BUS
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'state_est';
elems(1).DataType = 'double';
elems(1).Dimensions = [6, 1];

elems(2) = Simulink.BusElement;
elems(2).Name = 'motor_cmd';
elems(2).DataType = 'double';
elems(2).Dimensions = [4, 1];

bus_OBC = Simulink.Bus;
bus_OBC.Elements = elems;

% Sensors BUS
clear elems;

