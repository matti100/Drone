function drone_fig2(nD, plane, traj, ang, drone_dimensions, wait)
%% Input: 
% - nD:                 2 -> 2D
%                       3 -> 3D
% - plane:              1: XY plane
%                       2: XZ plane
%                       3: YZ plane
% - traj:               trajectory of the drone
% - ang:                orientation of the drone
% - drone_dimentions:   .armLength
%                       .bodySize
%                       .rotorRadius
% - wait:               wait time (s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nD == 2) % 2D view
    if (plane == 1) % XY plane
        az = 0;
        el = 90;
    elseif (plane == 2) % XZ plane
        az = 90;
        el = 0;
    elseif (plane == 3) % YZ plane
        az = 0;
        el = 0;
    end

    view(az, el); 
else
    view(nD) % 3D view
end
axis equal;
hold on;
grid on;
xlim('auto');
ylim('auto');
zlim('auto');

%% Drone COORDINATES
L = drone_dimensions.armLength;
B = drone_dimensions.bodySize;
r = drone_dimensions.rotRad;

% Body (Square)
X_b = B .* [-1 1 1 -1];
Y_b = B .* [-1 -1 1 1];

% Arms (Rectangle)
X_a1 = B .* [1 1 1 1];
X_a1(2) = X_a1(2) + L; X_a1(3) = X_a1(3) + L;
Y_a1 = Y_b / 5;

X_a2 = -B .* [1 1 1 1]; 
X_a2(2) = X_a2(2) - L; X_a2(3) = X_a2(3) - L;
Y_a2 = Y_b / 5;

X_a3 = X_b / 5;
Y_a3 = B .* [1 1 1 1];
Y_a3(3) = Y_a3(3) + L; Y_a3(4) = Y_a3(4) + L;

X_a4 = X_b / 5;
Y_a4 = -B .* [1 1 1 1];
Y_a4(3) = Y_a4(3) - L; Y_a4(4) = Y_a4(4) - L;

% Rotors (Circle)
X_r1 = B+L; Y_r1 = 0;
X_r2 = -B-L; Y_r2 = 0;
X_r3 = 0; Y_r3 = B+L;
X_r4 = 0; Y_r4 = -B-L;
thetaC = linspace(0, 2*pi, 100);

%% PLOT
% Draw drone on initial position
Z = zeros(4,1);

Drone_body = fill3(X_b, Y_b, Z, 'red');
Drone_arm1 = fill3(X_a1, Y_a1, Z, 'blue');
Drone_arm2 = fill3(X_a2, Y_a2, Z, 'blue');
Drone_arm3 = fill3(X_a3, Y_a3, Z, 'blue');
Drone_arm4 = fill3(X_a4, Y_a4, Z, 'blue');

Drone_rotor1 = fill3(X_r1 + r*cos(thetaC), Y_r1 + r*sin(thetaC), ...
    zeros(size(thetaC)), 'y');
Drone_rotor2 = fill3(X_r2 + r*cos(thetaC), Y_r2 + r*sin(thetaC), ...
    zeros(size(thetaC)), 'y');
Drone_rotor3 = fill3(X_r3 + r*cos(thetaC), Y_r3 + r*sin(thetaC), ...
    zeros(size(thetaC)), 'y');
Drone_rotor4 = fill3(X_r4 + r*cos(thetaC), Y_r3 + r*sin(thetaC), ...
    zeros(size(thetaC)), 'y');

%% TRAJECTORY and ROTATION
x = traj(:, 1);
y = traj(:, 2);
z = traj(:, 3);

phi = ang(:, 1);
theta = ang(:, 2);
psi = ang(:, 3);

traj = plot3(x(1), y(1), z(1), '--k', 'LineWidth', 2);

%% UPDATE GRAPH
for k = 1:length(x)

    % Rotation matrix
    % R_phi = [cos(phi(k)),       0,                  sin(phi(k));
    %          0,                 1,                  0;
    %          -sin(phi(k)),      0,                  cos(phi(k))];
    % R_theta = [1,               0,                  0;
    %            0,               cos(theta(k)),      -sin(theta(k));
    %            0,               sin(theta(k)),      cos(theta(k))];
    % R_psi = [cos(psi(k)),       -sin(psi(k)),       0;
    %          sin(psi(k)),       cos(psi(k)),        0;
    %          0,                 0,                  1];
    % 

    R_phi =     [1,                 0,                  0;
                 0,                 cos(phi(k)),        -sin(phi(k));
                 0,                 sin(phi(k)),        cos(phi(k))];
    R_theta =   [cos(theta(k)),     0,                  sin(theta(k));
                 0,                 1,                  0;
                 -sin(theta(k)),    0,                  cos(theta(k))];
    R_psi =     [cos(psi(k)),       -sin(psi(k)),       0;
                 sin(psi(k)),       cos(psi(k)),        0;
                 0,                 0,                  1];

    % Full rotation matrix
    R = R_psi * R_theta * R_phi;

    % Apply rotation to the drone's body and arms
    rotated_body = R * [X_b; Y_b; Z'];
    rotated_arm1 = R * [X_a1; Y_a1; Z'];
    rotated_arm2 = R * [X_a2; Y_a2; Z'];
    rotated_arm3 = R * [X_a3; Y_a3; Z'];
    rotated_arm4 = R * [X_a4; Y_a4; Z'];

    % Update the drone's body and arms positions
    set(Drone_body, "XData", rotated_body(1, :) + x(k), "YData", rotated_body(2, :) + y(k), "ZData", rotated_body(3, :) + z(k));
    set(Drone_arm1, "XData", rotated_arm1(1, :) + x(k), "YData", rotated_arm1(2, :) + y(k), "ZData", rotated_arm1(3, :) + z(k));
    set(Drone_arm2, "XData", rotated_arm2(1, :) + x(k), "YData", rotated_arm2(2, :) + y(k), "ZData", rotated_arm2(3, :) + z(k));
    set(Drone_arm3, "XData", rotated_arm3(1, :) + x(k), "YData", rotated_arm3(2, :) + y(k), "ZData", rotated_arm3(3, :) + z(k));
    set(Drone_arm4, "XData", rotated_arm4(1, :) + x(k), "YData", rotated_arm4(2, :) + y(k), "ZData", rotated_arm4(3, :) + z(k));

    % Apply rotation to the rotors
    rotated_rotor1 = R * [(X_r1 + r*cos(thetaC)); (Y_r1 + r*sin(thetaC)); zeros(size(thetaC))];
    rotated_rotor2 = R * [(X_r2 + r*cos(thetaC)); (Y_r2 + r*sin(thetaC)); zeros(size(thetaC))];
    rotated_rotor3 = R * [(X_r3 + r*cos(thetaC)); (Y_r3 + r*sin(thetaC)); zeros(size(thetaC))];
    rotated_rotor4 = R * [(X_r4 + r*cos(thetaC)); (Y_r4 + r*sin(thetaC)); zeros(size(thetaC))];

    % Update the rotor positions
    
    set(Drone_rotor1, "XData", rotated_rotor1(1, :) + x(k), "YData", rotated_rotor1(2, :) + y(k), "ZData", rotated_rotor1(3, :) + z(k));
    set(Drone_rotor2, "XData", rotated_rotor2(1, :) + x(k), "YData", rotated_rotor2(2, :) + y(k), "ZData", rotated_rotor2(3, :) + z(k));
    set(Drone_rotor3, "XData", rotated_rotor3(1, :) + x(k), "YData", rotated_rotor3(2, :) + y(k), "ZData", rotated_rotor3(3, :) + z(k));
    set(Drone_rotor4, "XData", rotated_rotor4(1, :) + x(k), "YData", rotated_rotor4(2, :) + y(k), "ZData", rotated_rotor4(3, :) + z(k));

    % Update trajectory
    set(traj, "XData", x(1:k), "YData", y(1:k), "ZData", z(1:k));

    pause(wait);
end
