function tunedGains = PID_tuner(Drone, k0, maxIter, tol, alpha)
% Initial k values
kP = k0(:, 1);
kI = k0(:, 2);
kD = k0(:, 3);

k = [kP; kI; kD];

dJ_dk = zeros(18,1);

% Gradient Conjugate optimization method
iter = 0;
err_grad = tol + 1;
dk = 0.001;
while iter < maxIter && err_grad > tol
    iter = iter +1;
    
    % Compute gradient of the cost function
    for (j = 1:length(k))
        k2 = k;
        k2(j) = k2(j) + dk;
        kP2 = k2(1:6);
        kI2 = k2(7:12);
        kD2 = k2(13:18);

        dJ = ( cost_function(kP2, kI2, kD2) - cost_function(kP, kI, kD) );
        dJ_dk(j) = dJ / dk;
    end

    % Update k
    k_prev = k;
    k = k - alpha.*dJ_dk;

    % Error gradient method
    err_grad = max(abs(k - k_prev));

    logger(iter, k, err_grad);

end

% Update gains structure
kP = k(1:6);
kI = k(7:12);
kD = k(13:18);

gains = struct();
gains = gainBuilder(kP, kI, kD, gains);

tunedGains = gains;

%% ----------- Functions --------------

    function J = cost_function(kP, kI, kD)
        g = struct();
        g = gainBuilder(kP, kI, kD, g);

        % Initialize drone
        Drone.updateGains(g);
        
        % Initialize cost function
        v = [];

        % Simulation
        for i = 1:length(Drone.tvec)-1
            Drone.updateState();

            % Cost function computation
            % Integral of Squared Error (ISE)
            % J = integral( sum_i ( err_i^2 ) ), i = 1...6 pid

            Err = [Drone.err_x; Drone.err_y; Drone.err_z; Drone.err_phi; Drone.err_theta; Drone.err_psi];
            Err = Err.^2;
            err = sum(Err);

            v = [v; err];

        end

        % Trapezium integral method
        J = 0.5*Drone.dt*( v(1) + v(end) ) + Drone.dt*sum( v(2:end-1) );
    end

    function logger(iter, k, err_grad)
        fprintf("Iteration n. %d\t| ", iter);
        fprintf("Error = %f | ", err_grad);

        fprintf("kP = [%f, %f, %f, %f, %f, %f] | ", k(1:6));
        fprintf("kI = [%f, %f, %f, %f, %f, %f] | ", k(7:12));
        fprintf("kD = [%f, %f, %f, %f, %f, %f]\n", k(13:18));
    end

end