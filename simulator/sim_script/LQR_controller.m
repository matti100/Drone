clear
clc
close all

%% Parameters

% Inertial Data:
m = 0.5;        % [Kg]              % Mass
    
a = 2.32e-3;    % [Kg / m^2]        % Principal Moment of Inertia X
b = 2.32e-3;    % [Kg / m^2]        % Principal Moment of Inertia x
c = 4.00e-3;    % [Kg / m^2]        % Principal Moment of Inertia Z
I = diag([a, b, c]);                % Principal Inertia Matrix

% Geometric Data
L = 0.2;        % [m]                % Arm length
B = 0.2;        % [m]                % body size
r = 0.1;        % [m]                % rotor radius

% Torque and Lift Coefficients
k_f = 6.11e-8;  % [N / rpm]         % Lift Coefficient   
k_f = (6.11e-8)*60/6.28; % [N/rad/s]
k_m = 1.5e-9;   % [N*m / rpm]       % Torque Coefficient
k_m = (1.5e-9)*60/6.28; % [N/rad/s]

% Gravitational Data
g = 9.81;      % [m/s^2]           % gravitx acceleration

drone_dimensions = struct();
drone_dimensions.armLength = L;
drone_dimensions.bodySize = B;
drone_dimensions.rotorRadius = r;

%% Linearization around equilibrium point
% Hovering

% Free variables
x_h = 0;
y_h = 0;
z_h = 2;

phi_h = 0;
theta_h = 0;
psi_h = 0;

% Equilibrium point
x_h = [x_h;
       y_h;
       z_h;
       0;
       0;
       0;
       phi_h;
       theta_h;
       psi_h;
       0;
       0;
       0];

% Compute A-matrix
A = zeros(12,12);
A(1, 4) = 1;
A(2, 5) = 1;
A(3, 6) = 1;
A(4, 7) = g*sin(psi_h);
A(4, 8) = -g*cos(psi_h);
A(5, 7) = -g*cos(psi_h);
A(5, 8) = -g*sin(psi_h);
A(7, 10) = 1;
A(8, 11) = 1;
A(9, 12) = 1;

% Compute B-matrix
B = zeros(12, 4);
B(6, :) = sqrt((k_f*g)/m);
B(10, 3) = -(2*L*k_f/a)*sqrt((m*g)/(4*k_f));
B(10, 4) = (2*L*k_f/a)*sqrt((m*g)/(4*k_f));
B(11, 1) = -(2*L*k_f/b)*sqrt((m*g)/(4*k_f));
B(11, 2) = (2*L*k_f/b)*sqrt((m*g)/(4*k_f));
B(12, 1:2) = (2*k_m/c)*sqrt((m*g)/(4*k_f));
B(12, 3:4) = -(2*k_m/c)*sqrt((m*g)/(4*k_f));

% Verifying modes stability
eigs = eig(A);
stability = 1;
for (i = 1:length(eigs))
    if (real(eigs(i)) >= 0)
        fprintf("unstable mode: lambda %d = %f\n", i, eigs(i));
        stability = 0;
    end
end

if (~stability)
    fprintf("! Linearized system is unstable !\n");
end

% Verifying controllability
Kc = ctrb(A, B);
if (rank(Kc) < 12)
    disp("Linear system not fully controllable");
    fprintf("rank(Kc) = %d\n", rank(Kc));
else
    disp("Linear system fully controllable");
    fprintf("rank(Kc) = %d\n", rank(Kc));

    % LQR control design
    % State variables weigths;
    d = zeros(12, 1);
    d(1:3) = 100;        % x, y, z
    d(4:6) = 100;        % x_dot, y_dot, z_dot
    d(7:9) = 100;        % phi, theta, psi
    d(10:end) = 100;     % phi_dot, theta_dot, psi_dot
    % Q matrix computation
    Q = diag(d);

    % Input variables weights
    d = 100.*ones(4,1);
    % R matrix computation
    R = diag(d);

    % Control matrix computation
    K = lqr(A, B, Q, R);

    % Linearized ode system setup
    % Control law
    u = @(x) -K*(x_h - x); % reference
    % Linearized ode system
    F_l = @(t, x) A*x + B*u(x);

    % Initial conditions
    x0 = zeros(12, 1);
    x0(7) = 0.2;
    x0(8) = 2;
    x0(9) = 5;

    % Time interval
    tspan = [0; 100];

    % Ode system solver
    [t,x] = ode45(F_l, tspan, x0);
end

%% Plot
traj = [x(:, 1), x(:, 2), x(:, 3)];
ang = [x(:, 7), x(:, 8), x(:, 9)];

pause = 0.05; % [s]

close all
figure
subplot(2,2,1)
xlabel('X (m)');
ylabel('Y (m)');
title('XY plane');
drone_fig2(2, 1, traj, ang, drone_dimensions, pause);

subplot(2,2,2)
zlabel('Z (m)');
ylabel('Y (m)');
title('YZ plane');
drone_fig2(2, 2, traj, ang, drone_dimensions, pause);

subplot(2,2,3)
xlabel('X (m)');
zlabel('Z (m)');
title('XZ plane');
drone_fig2(2, 3, traj, ang, drone_dimensions, pause);

subplot(2,2,4)
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
title('3D view');
drone_fig2(3, 1, traj, ang, drone_dimensions, pause);

%% Non linear system
x_ref = x_h;
u = @(x) -K*(x_ref - x);
u4 = @(x) [0,0,0,1]*u(x);
u3 = @(x) [0,0,1,0]*u(x);
u2 = @(x) [0,1,0,0]*u(x);
u1 = @(x) [1,0,0,0]*u(x);

F = @(t,x) [
             x(4);
             x(5);
             x(6);
             (k_f/m)*(sin(x(9))*sin(x(7)) - cos(x(9))*sin(x(8))*cos(x(7)))*norm(u(x))^2;
             (k_f/m)*(-cos(x(9))*sin(x(7)) - sin(x(9))*sin(x(8))*cos(x(7)))*norm(u(x))^2;
             (k_f/m)*(cos(x(8))*cos(x(7)))*norm(u(x))^2 - g;
             x(10);
             x(11);
             x(12);
             (L*k_f/a)*( (u4(x))^2 - (u3(x))^2 ) + ((b-c)/a)*x(11)*x(12);
             (L*k_f/b)*( (u2(x))^2 - (u1(x))^2 ) + ((c-a)/b)*x(10)*x(12);
             (k_m/c)*( (u1(x))^2 + (u2(x))^2 - (u3(x))^2 - (u4(x))^2 ) + ((a-b)/c)*x(10)*x(11);
             ];

% Initial conditions
x0 = zeros(12, 1);
x0(3) = 2;

% Time interval
tspan = [0, 100];    % [s]

% Ode system solver
[t,x] = ode45(F, tspan, x0);

%% Plot
traj = [x(:, 1), x(:, 2), x(:, 3)];
ang = [x(:, 7), x(:, 8), x(:, 9)];

pause = 0.05; % [s]

close all
figure
subplot(2,2,1)
xlabel('X (m)');
ylabel('Y (m)');
title('XY plane');
drone_fig2(2, 1, traj, ang, drone_dimensions, pause);

subplot(2,2,2)
zlabel('Z (m)');
ylabel('Y (m)');
title('YZ plane');
drone_fig2(2, 2, traj, ang, drone_dimensions, pause);

subplot(2,2,3)
xlabel('X (m)');
zlabel('Z (m)');
title('XZ plane');
drone_fig2(2, 3, traj, ang, drone_dimensions, pause);

subplot(2,2,4)
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
title('3D view');
drone_fig2(3, 1, traj, ang, drone_dimensions, pause);

%% Non linear system scheme
% 
% F = @(t,x,u) [
%              x(4);
%              x(5);
%              x(6);
%              (k_f/m)*(sin(x(9))*sin(x(7)) - cos(x(9))*sin(x(8))*cos(x(7)))*(u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2);
%              (k_f/m)*(-cos(x(9))*sin(x(7)) - sin(x(9))*sin(x(8))*cos(x(7)))*(u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2);
%              (k_f/m)*(cos(x(8))*cos(x(7)))*(u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2) - g;
%              x(10);
%              x(11);
%              x(12);
%              (L*k_f/a)*(u(4)^2 - u(3)^2) + ((b-c)/a)*x(11)*x(12);
%              (L*k_f/b)*(u(2)^2 - u(1)^2) + ((c-a)/b)*x(10)*x(12);
%              (k_m/c)*(u(1)^2 + u(2)^2 - u(3)^2 - u(4)^2) + ((a-b)/c)*x(10)*x(11);
%              ];
% 
% % Initial conditions
% x0 = zeros(12, 1);
% 
% % Time interval
% tspan = [0, 10];    % [s]
% 
% % Ode system solver
% [t,x] = ode45(F, tspan, x0);