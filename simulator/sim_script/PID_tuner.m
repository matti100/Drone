function tunedGains = PID_tuner(Drone, k0, maxIter, tol, alpha)

% Start Parallel Computing
parpool("Processes", 8);

% Initial k values
kP = k0(:, 1);
kI = k0(:, 2);
kD = k0(:, 3);

k = [kP; kI; kD];

dJ_dk = zeros(18,1);

% Gradient optimization method
iter = 0;
err_grad = tol + 1;
dk = 0.0001;
while iter < maxIter && err_grad > tol
    iter = iter +1;

    % Compute gradient of the cost function
    J = cost_function(kP, kI, kD, Drone);

    parfor (j = 1:length(k))
        k2 = k;
        k2(j) = k2(j) + dk;
        kP2 = k2(1:6);
        kI2 = k2(7:12);
        kD2 = k2(13:18);

        J2 = cost_function(kP2, kI2, kD2, Drone);
        dJ = J2 - J;

        dJ_dk(j) = dJ / dk;
    end

    % Update k
    k = k - alpha.*dJ_dk;

    % Update gains structure
    kP = k(1:6);
    kI = k(7:12);
    kD = k(13:18);

    % Error gradient method
    % err_grad = max(abs(k - k_prev));
    err_grad = J;

    logger(iter, k, err_grad);

end

gains = gainBuilder(kP, kI, kD);

tunedGains = gains;

end

%% ----------- Functions --------------

function J = cost_function(kP, kI, kD, Drone)
g = gainBuilder(kP, kI, kD);

% Initialize drone
Drone.updateGains(g);

% Simulation
for i = 1:length(Drone.tvec)-1
    Drone.updateState();
end

% Mean Square Error (MSE)
err_x = Drone.err_x(~isnan(Drone.err_x) & ~isinf(Drone.err_x));
err_y = Drone.err_y(~isnan(Drone.err_y) & ~isinf(Drone.err_y));
err_z = Drone.err_z(~isnan(Drone.err_z) & ~isinf(Drone.err_z));
err_phi = Drone.err_phi(~isnan(Drone.err_phi) & ~isinf(Drone.err_phi));
err_theta = Drone.err_theta(~isnan(Drone.err_theta) & ~isinf(Drone.err_theta));
err_psi = Drone.err_psi(~isnan(Drone.err_psi) & ~isinf(Drone.err_psi));

% error = [Drone.err_x; Drone.err_y; Drone.err_z; Drone.err_phi; Drone.err_theta; Drone.err_psi];
error = [err_x; err_y; err_z; err_phi; err_theta; err_psi];
error = error.^2;
MSE = mean(error);

% Lyapunov Function derivative
dV = Drone.dV(~isnan(Drone.dV) & ~isinf(Drone.dV));
% dV = Drone.dV

V_dot = max(dV, zeros(size(dV)));
V_dot = sum(V_dot);
V_dot = 0;

% Cost function
J = MSE + V_dot;
end

function logger(iter, k, err_grad)
fprintf("Iteration n. %d\t| ", iter);
fprintf("Error = %f | ", err_grad);

fprintf("kP = [%f, %f, %f, %f, %f, %f] | ", k(1:6));
fprintf("kI = [%f, %f, %f, %f, %f, %f] | ", k(7:12));
fprintf("kD = [%f, %f, %f, %f, %f, %f]\n", k(13:18));
end