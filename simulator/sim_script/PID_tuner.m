function tunedGains = PID_tuner(Drone, k0, maxIter, tol, alpha)

% Start Parallel Computing
% parpool("Processes", 8);

% Initial k values
kP = k0(:, 1);
kI = k0(:, 2);
kD = k0(:, 3);

k = [kP; kI; kD];

dJ_dk = zeros(18,1);

% Gradient optimization method
iter = 0;
err_grad = tol + 1;
dk = 0.01;
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
    err_grad = norm(dJ_dk);

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
parfor i = 1:length(Drone.tvec)-1
    Drone.updateState();
end

% Integral of Square Error (ISE)

err_x = Drone.Err_x;
err_y = Drone.Err_y;
err_z = Drone.Err_z;
err_phi = Drone.Err_phi;
err_theta = Drone.Err_theta;
err_psi = Drone.Err_psi;

err_x(isnan(err_x)) = eps(0);
err_y(isnan(err_y)) = eps(0);
err_z(isnan(err_z)) = eps(0);
err_phi(isnan(err_phi)) = eps(0);
err_theta(isnan(err_theta)) = eps(0);
err_psi(isnan(err_psi)) = eps(0);

err_x(isinf(err_x)) = realmax;
err_y(isinf(err_y)) = realmax;
err_z(isinf(err_z)) = realmax;
err_phi(isinf(err_phi)) = realmax;
err_theta(isinf(err_theta)) = realmax;
err_psi(isinf(err_psi)) = realmax;

error = [err_x; err_y; err_z; err_phi; err_theta; err_psi];
error = error.^2;

ISE = norm(trapz(error));

% Cost function
J = ISE;

end

function logger(iter, k, err_grad)
fprintf("Iteration n. %d\t| ", iter);
fprintf("Error = %f | ", err_grad);

fprintf("kP = [%f, %f, %f, %f, %f, %f] | ", k(1:6));
fprintf("kI = [%f, %f, %f, %f, %f, %f] | ", k(7:12));
fprintf("kD = [%f, %f, %f, %f, %f, %f]\n", k(13:18));
end