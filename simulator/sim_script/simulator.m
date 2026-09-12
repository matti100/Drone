clear
clc
close all
w = warning ('on','all');
%% new branch

%% Problem initialization
params = struct();

% Inertial Data:             
params.m = 0.8;             % Mass [Kg]       
params.Ix = 15.67*1e-3;     % Principal X Moment of Inertia [Kg / m^2]       
params.Iy = 15.67*1e-3;     % Principal Y Moment of Inertia [Kg / m^2]
params.Iz = 28.34*1e-3;     % Principal Z Moment of Inertia [Kg / m^2]
params.I = diag([params.Ix, params.Iy, params.Iz]);  % Principal Inertia Matrix

% Geometric Data            
params.armLength = 0.3;     % Arm length [m]   
params.bodySize = 0.2;      % Body size [m]               
params.rotRad = 0.1;        % Rotor radius [m]                

% Torque and Lift Coefficients
params.k_f = 1.4*1e-5 / 9.54929658;          % Lift Coefficient [N / rpm]
% params.k_f = (k_f)*60/6.28;                % [N/rad/s]
params.k_m = 1.78*1e-6 / 9.54929658;         % Torque Coefficient [N*m / rpm]
% params.k_m = (k_m)*60/6.28;                % [N/rad/s]

% Gravitational Data
params.g = 9.81;                             % [m/s^2]

% Time interval
params.dt = 0.001;                           % [s]

% Initial Condition
x0 = zeros(12,1);
x0(1) = 0;       % Initial x position [m]
x0(2) = 0;       % Initial y position [m]
x0(3) = 0;       % Initial z position [m]

x0(4) = 0;       % Initial x velocity [m/s]
x0(5) = 0;       % Initial y velocity [m/s]
x0(6) = 0;       % Initial z velocity [m/s]

x0(7) = 0.3;     % Initial roll [rad]
x0(8) = 0.5;     % Initial pitch [rad]
x0(9) = 0;       % Initial yaw [rad]

x0(10) = 0;      % Initial roll rate [rad/s]
x0(11) = 0;      % Initial pitch rate [rad/s]
x0(12) = 0;      % Initial yaw rate [rad/s]

% Desidered Position and Attitude
desideredState = struct();
desideredState.rDes = [-0.5, 1, 1]';      % [m]
desideredState.attDes = [0, 0, 0]';       % [rad]

% Initialize simulation time
t0 = 0;                 % [s]
tmax = 10;              % [s]
tspan = [t0, tmax];
tvec = t0:params.dt:tmax;

% Control mode
control = 2;           % 0 -> PID + non-linear dynamics
                       % 1 -> LQR + linear dynamics
                       % 2 -> LQR + non-linear dynamics
                       % -1 -> PID + linear dynamics
                       % 3 -> mpc

% Estimator Flag
estimation = 0;        % 0 -> no estimation
                       % 1 -> kalman filter

% Discrete Time
sampleTime = 0.005;          % [s] sample time for the discrete system
                             % sampleTime = params.dt -> continous time

% Plot flag
plot_flag = 1;               % 1 -> plot                0 -> no plot
anim_flag = 0;               % 1 -> animation           0 -> no animation

% Tuner flag
tuner_flag = 0;         % 1 -> Gradient Descent optimization
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
    tol = 1e-6;
    alpha = 10;

    k0 = rand(6,3); %[kP, kI, kD]

    gains = gainBuilder(k0(:, 1), k0(:, 2), k0(:, 3));

    myDrone = Drone(params, x0, desideredState, gains, tspan, -1, estimation, sampleTime);

    tunedGains = PID_tuner(myDrone, k0, maxIter, tol, alpha);

    save('tunedGains_gradient', "tunedGains");

elseif (tuner_flag == 2)
    disp('-----------------------------------');
    disp('-------- Genetic Algorithm --------');
    disp('-----------------------------------');
    
    pop_size = 100;
    maxGen = 2000;
    tol = 1;
    mutation_rate = 0.6;
    kMax = 10;

    k0 = zeros(6,3);
    gains = gainBuilder(k0(:, 1), k0(:, 2), k0(:, 3));
    myDrone = Drone(params, x0, desideredState, gains, tspan, -1, estimation, sampleTime);

    tunedGains = ga_tuner(myDrone, pop_size, maxGen, mutation_rate, kMax, tol);

    save('tunedGains_ga', "tunedGains");

else
    disp('-----------------------------------');
    disp('------------ Simulation -----------');
    disp('-----------------------------------');
    
    % Manual tuning                             % I set                 II set
    kP = [20;           % -> kP_T                  20                     20
          -0.4;         % -> kP_phi                0
          0.4;          % -> kP_theta              0.4
          1;            % -> kP_R                  1                      1.55
          1;            % -> kP_P                  1                      1.55
          1.2];         % -> kP_Y                  0.1                  x 0.02

    kI = [10;           % -> kI_T                  10                     10
          -0.09;        % -> kI_phi                0
          0.09;         % -> kI_theta              0.09
          0.13;         % -> kI_R                  0.13                   0.17
          0.13;         % -> KI_P                  0.13                   0.17
          0.03];        % -> KI_Y                  0.013                x 0.01

    kD = [10;           % -> kD_T                  10                     10
          -0.25;        % -> kD_phi                0
          0.25;         % -> kD_theta              0.25
          0.66;         % -> kD_R                  0.66                   0.76
          0.66;         % -> kD_P                  0.66                   0.76
          0.9];         % -> kD_Y                  0.066                x 0.01
          
    
    % load("tunedGains_ga.mat");
    % gains = tunedGains;
    gains = gainBuilder(kP, kI, kD);

    % Initialize Drone
    myDrone = Drone3(params, x0, desideredState, gains, tspan, control, estimation, sampleTime);

    % Simulation
    t = 0;
    for i = 1:length(tvec)-1
        if (myDrone.control == 3)
            if ( abs(myDrone.r - desideredState.rDes) < 0.8*ones(3,1) )
                myDrone.control = 2;
            end
        end

        myDrone.updateState();
        myDrone.logger();
    end
end

%% Plot
if (plot_flag)
    close all

    % Variables definition
    traj = myDrone.traj(:, 1:3);
    vel = myDrone.traj(:, 4:6);
    ang = myDrone.traj(:, 7:9);
    w = myDrone.traj(:, 10:12);

    cmd = myDrone.U;

    traj_est = myDrone.traj_est(:, 1:3);
    vel_est = myDrone.traj_est(:, 4:6);
    ang_est = myDrone.traj_est(:, 7:9);
    w_est = myDrone.traj_est(:, 10:12);

    % Plots
    % State
    figure(1);

    subplot(2,1,1)
    plot(tvec, traj);
    hold on;
    plot(tvec, desideredState.rDes.*ones(size(tvec)), 'k--');
    legend('x(t)', 'y(t)', 'z(t)');
    xlabel('Time [s]');
    ylabel('Trajectory [m]');
    title('Trajectory of the drone');
    % ylim([-2, 6])
    
    subplot(2,1,2)
    plot(tvec, ang);
    xlabel('Time [s]');
    ylabel('attitude [rad]');
    legend('phi', 'theta', 'psi');
    title('Attitude of the drone');
    % ylim([-0.3, 0.3]);

    % Commands
    figure(2);
    stairs(tvec, cmd);
    xlabel('Time [s]');
    ylabel('Control');
    legend('u1', 'u2', 'u3', 'u4');
    title('Control inputs');

    % State Estimation
    figure(3);
    stairs(tvec, traj_est);
    hold on;
    plot(tvec, desideredState.rDes.*ones(size(tvec)), 'k--');
    xlabel('Time [s]');
    ylabel('Trajectory [m]');
    legend('x est', 'y est', 'z est');
    title('Trajectory Estimation');

    % % Lyapunov Function
    % figure(4);
    % plot(tvec, myDrone.V);
    % hold on;
    % plot(tvec, myDrone.dV);
    % plot(tvec, zeros(size(tvec)), '-.r');
    % xlabel('Time [s]');
    % ylabel('Lyapunov function');
    % legend('V(t)', 'dV(t)', '0 line');
    % title('Lyapunov Function');

    % Animation
    if (anim_flag)

    drone_dimensions = struct();
    drone_dimensions.armLength = params.armLength;
    drone_dimensions.bodySize = params.bodySize;
    drone_dimensions.rotRad = params.rotRad;

    pause = 0.001; % [s]

    figure(2);
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
