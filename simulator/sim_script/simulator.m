clear
clc
close all

%% Problem initialization
params = struct();

% Inertial Data:
params.m = 1.25;            % [Kg]              % Mass
params.Ix = 0.0232;          % [Kg / m^2]        % Principal Moment of Inertia X
params.Iy = 0.0232;         % [Kg / m^2]        % Principal Moment of Inertia x
params.Iz = 0.0468;         % [Kg / m^2]        % Principal Moment of Inertia Z
params.I = diag([params.Ix, params.Iy, params.Iz]);
% Principal Inertia Matrix

% Geometric Data
params.armLength = 0.265;   % [m]                % Arm length
params.bodySize = 0.2;      % [m]                % body size
params.rotRad = 0.1;        % [m]                % rotor radius

% Torque and Lift Coefficients
k_f = 6.11e-8;   % [N / rpm]         % Lift Coefficient
params.k_f = 0.02;
% params.k_f = (k_f)*60/6.28;                % [N/rad/s]
k_m = 1.5e-9;    % [N*m / rpm]       % Torque Coefficient
params.k_m = 0.01;
% params.k_m = (k_m)*60/6.28;                % [N/rad/s]

% Gravitational Data
params.g = 9.81;        % [m/s^2]            % Gravity acceleration

% Time interval
params.dt = 0.01;       % [s]                % Time interval

% Initial Condition
x0 = zeros(12,1);
% x0(1) = 1;

% Desidered Position
desideredState = struct();
desideredState.rDes = [0, 0, 2]';      % [m]
desideredState.attDes = [0, 0, pi/2]';

% Initialize simulation time
t0 = 0;                 % [s]
tmax = 20;              % [s]
tspan = [t0, tmax];
tvec = t0:params.dt:tmax;

% Plot flag
plot_flag = 1;               % 1 -> plot                0 -> no plot
anim_flag = 0;               % 1 -> animation           0 -> no animation

% Tuner flag
tuner_flag = 2;         % 1 -> Gradient Descent optimization
                        % 2 -> Genetic Algorithm
                        % 0 -> no tuning

if (tuner_flag)
    plot_flag = 0;
end

%% Simulation
% -------
% Tuning
% -------
if (tuner_flag == 1)
    disp('-----------------------------------');
    disp('-- Gradient Descent optimization --');
    disp('-----------------------------------');

    maxIter = 10000;
    tol = 1e-2;
    alpha = 0.0001;

    k0 = rand(6,3); %[kP, kI, kD]

    % k0(1, :) = k0(1, :) .* 1e2;
    % k0(2, :) = k0(2, :) .* 1e-2;
    % k0(3, :) = k0(3, :) .* 1e-3;
    % k0(4, :) = k0(4, :) .* 1e-2;
    % k0(5, :) = k0(5, :) .* 1e-4;
    % k0(6, :) = k0(5, :) .* 1e-4;

    gains = gainBuilder(k0(:, 1), k0(:, 2), k0(:, 3));

    Drone = Drone(params, x0, desideredState, gains, tspan);
    % Drone.linearizeHover();

    tunedGains = PID_tuner(Drone, k0, maxIter, tol, alpha);

    save('tunedGains_gradient', "tunedGains");

elseif (tuner_flag == 2)
    disp('-----------------------------------');
    disp('-------- Genetic Algorithm --------');
    disp('-----------------------------------');
    
    pop_size = 100;
    maxGen = 1500;
    tol = 1e-2;
    mutation_rate = 0.3;
    kMax = 1e-2;

    k0 = zeros(6,3);
    gains = gainBuilder(k0(:, 1), k0(:, 2), k0(:, 3));
    Drone = Drone(params, x0, desideredState, gains, tspan);
    % Drone.linearizeHover();

    tunedGains = ga_tuner(Drone, pop_size, maxGen, mutation_rate, kMax, tol);

    save('tunedGains_ga', "tunedGains");

else
    disp('-----------------------------------');
    disp('------------ Simulation -----------');
    disp('-----------------------------------');

    if (tuner_flag == 1)
        load('tunedGains_gradient');
    elseif (tuner_flag == 2)
        load('tunedGains_ga');
    end
    
    % Manual tuning
    kP = [20;        % -> kP_T
          1;        % -> kP_phi
          1;        % -> kP_theta
          0;        % -> kP_R
          1;        % -> kP_P
          10];       % -> kP_Y

    kI = [5;        % -> kI_T        
          0;        % -> kI_phi
          0;        % -> kI_theta
          0;        % -> kI_R
          0;        % -> KI_P
          0];       % -> KI_Y

    kD = [20;        % -> kD_T
          0;        % -> kD_phi
          0;        % -> kD_theta
          0;        % -> kD_R
          0;        % -> kD_P
          1];       % -> kD_Y

    gains = gainBuilder(kP, kI, kD);

    % gains = tunedGains;

    % Initialize Drone
    Drone = Drone(params, x0, desideredState, gains, tspan);
    Drone.linearizeHover();

    % Simulation
    for i = 1:length(tvec)-1
        Drone.updateState();
        Drone.logger();
    end
end

%% Plot
if (plot_flag)
    traj = Drone.traj(:, 1:3);
    vel = Drone.traj(:, 4:6);
    ang = Drone.traj(:, 7:9);
    w = Drone.traj(:, 10:12);

    figure;

    subplot(2,2,1)
    plot(tvec, traj);
    legend('x(t)', 'y(t)', 'z(t)');
    xlabel('Time [s]');
    ylabel('Trajectory [m]');
    title('Trajectory of the drone');
    
    subplot(2,2,2)
    plot(tvec, Drone.V);
    hold on;
    plot(tvec, Drone.dV);
    plot(tvec, zeros(size(tvec)), '-.r');
    xlabel('Time [s]');
    ylabel('Lyapunov function');
    legend('V(t)', 'dV(t)', '0 line');
    title('Lyapunov Function');
    
    subplot(2,2,3)
    plot(tvec, ang);
    xlabel('Time [s]');
    ylabel('attitude [rad]');
    legend('phi', 'theta', 'psi');
    title('Attitude of the drone');

    if (anim_flag)

    drone_dimensions = struct();
    drone_dimensions.armLength = params.armLength;
    drone_dimensions.bodySize = params.bodySize;
    drone_dimensions.rotRad = params.rotRad;

    pause = 0.001; % [s]

    close all
    figure
    subplot(2,2,1)
    xlabel('X (m)');
    ylabel('Y (m)');
    title('XY plane');
    drone_fig(2, 1, traj, ang, drone_dimensions, pause);

    subplot(2,2,2)
    zlabel('Z (m)');
    ylabel('Y (m)');
    title('YZ plane');
    drone_fig(2, 2, traj, ang, drone_dimensions, pause);

    subplot(2,2,3)
    xlabel('X (m)');
    zlabel('Z (m)');
    title('XZ plane');
    drone_fig(2, 3, traj, ang, drone_dimensions, pause);

    subplot(2,2,4)
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    title('3D view');
    drone_fig(3, 1, traj, ang, drone_dimensions, pause);
    end
end