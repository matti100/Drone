classdef Drone < handle
    % References:
    % https://youtu.be/7F9cG64kRxI?si=X26RSqmpmWHZq-e0

    properties
        %% Params
        m               % [Kg]              % Mass

        Ix              % [Kg / m^2]         % Principal Moment of Inertia X
        Iy              % [Kg / m^2]         % Principal Moment of Inertia Y
        Iz              % [Kg / m^2]         % Principal Moment of Inertia Z
        I               % [3x3] [kg / m^2]   % Principal Inertia Matrix

        l               % [m]                % Arm length
        b               % [m]                % Body size (assume a cube)
        rotRad          % [m]                % Rotor radius

        k_f             % [N / rpm]          % Lift Coefficient
        %k_f = (k_f)*60/6.28;                % [N/rad/s]
        k_m             % [N*m / rpm]        % Torque Coefficient
        %k_m = (k_m)*60/6.28;                % [N/rad/s]

        g               % [m/s^2]            % Gravity acceleration (module)

        t               % [s]                % Time
        t0              % [s]                % Intial simulation time
        tf              % [s]                % Final simulation time
        dt              % [s]                % Time interval
        tvec                                 % Time vector

        sampleTime      % [s]                % sample time for the descrete system
        counter
        
        %% State Space
        x               % [x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi]
        r               % [x, y, z]
        dr              % [dx, dy, dz]
        att             % [phi, theta, psi]
        w               % [dphi, dtheta, dpsi]
        dx
        x_old;

        u               % [4x1] Inputs
        u_old
        u1              
        u2              
        u3              
        u4     
        U               % [tx4] Input Matrix

        % Commands
        T               
        R
        P
        Y

        % Initial conditions
        x0             
        r0
        dr0
        att0
        w0

        % Desidered states
        rDes
        attDes
        xDes

        % Errors
        error;
        err_x; err_y; err_z; err_phi; err_theta; err_psi;

        Err_x; Err_y; Err_z; Err_phi; Err_theta; Err_psi;

        err_x_prev; err_x_prevprev;
        err_y_prev; err_y_prevprev;
        err_z_prev; err_z_prevprev;
        err_phi_prev; err_phi_prevprev;
        err_theta_prev; err_theta_prevprev;
        err_psi_prev; err_psi_prevprev;

        Serr_x; Derr_x;
        Serr_y; Derr_y;
        Serr_z; Derr_z;
        Serr_phi; Derr_phi;
        Serr_theta; Derr_theta;
        Serr_psi; Derr_psi;

        error_est;
        err_x_est; err_y_est; err_z_est; err_phi_est; err_theta_est; err_psi_est;

        Err_x_est; Err_y_est; Err_z_est; Err_phi_est; Err_theta_est; Err_psi_est;

        %% PID gains
        % Position controller
        % Altitude controller
        kP_T
        kI_T
        kD_T

        % Phi controller
        kP_phi
        kI_phi
        kD_phi

        % Theta controller
        kP_theta
        kI_theta
        kD_theta

        % Attitude controller
        % Roll controller
        kP_R
        kI_R 
        kD_R

        % Pitch controller
        kP_P
        kI_P
        kD_P

        % Yaw controller
        kP_Y
        kI_Y
        kD_Y


        %% Dynamics
        F               % [3x1] [N]         External Force
        Fx                   
        Fy
        Fz

        M               % [3x1] [Nm]        External Torque
        Mx
        My
        Mz

        traj            % [tx12]            Time evolution of the state

        f               % [function hanlde] System dynamics
        Force          
        Torque          
        rotMat          
        R_phi           
        R_theta         
        R_psi           
        func            

        % Linearization

        f_lin;          % [function handle] Linearized System dynamics

        A;              % [12x12]           State space matrix
        B;              % [12x4]            Control matrix

        K;              % [4x4]             LQR gain matrix
        u_lin;          % [4x1]             LQR control 

        %% Control 
        control         % control flag

        %% Guidance

        %% Navigation
        estimation       % estimator flag

        sigma_acc;
        sigma_gyro;
        noise_acc;
        noise_gyro;

        acc_meas;
        gyro_meas;

        x_pred;
        P_pred;
       
        y_pred;

        x_est;
        r_est;
        dr_est;
        att_est;
        w_est;

        traj_est;

        P_est;

        y;
        
        Kg;
        h;
        Q;
        R_cov;
    
        %% Lyapunov
        V
        dV
       
    end

    methods

        %% Constructor
        function obj = Drone(params, initialCondition, desideredState, gains, tspan, control, estimation, sampleTime)
            
            % Params
            obj.m = params.m;
            obj.I = params.I;
            obj.Ix = obj.I(1,1); obj.Iy = obj.I(2,2); obj.Iz = obj.I(3,3);

            obj.l = params.armLength;
            obj.b = params.bodySize;
            obj.rotRad = params.rotRad;

            obj.k_f = params.k_f;
            obj.k_m = params.k_m;

            obj.g = params.g;

            obj.t = 0;
            obj.t0 = tspan(1);
            obj.tf = tspan(end);
            obj.dt = params.dt;
            obj.tvec = obj.t0:obj.dt:obj.tf;

            obj.sampleTime = sampleTime;
            obj.counter = 0;

            % State Space
            % Initial conditions
            obj.x0 = initialCondition;
            obj.r0 = obj.x0(1:3);
            obj.dr0 = obj.x0(4:6);
            obj.att0 = obj.x0(7:9);
            obj.w0 = obj.x0(10:12);

            obj.x = obj.x0;
            obj.r = obj.r0;
            obj.dr = obj.dr0;
            obj.att = obj.att0;
            obj.w = obj.w0;
            obj.dx = zeros(12,1);
            obj.x_old = obj.x;

            obj.u = zeros(4,1);
            obj.u_old = obj.u;
            obj.U = obj.u';

            % Desidered State
            obj.rDes = desideredState.rDes;
            obj.attDes = desideredState.attDes;
            obj.xDes = [obj.rDes; 0; 0; 0; obj.attDes; 0; 0; 0];
           
            % Errors
            obj.error = obj.xDes - obj.x;

            obj.err_x = obj.error(1);
            obj.err_y = obj.error(2);
            obj.err_z = obj.error(3);
            obj.err_phi = obj.error(7);
            obj.err_theta = obj.error(8);
            obj.err_psi = obj.error(9);

            obj.err_x_prev = 0; obj.err_x_prevprev = 0; obj.Serr_x = 0; obj.Derr_x = 0;
            obj.err_y_prev = 0; obj.err_y_prevprev = 0; obj.Serr_y = 0; obj.Derr_y = 0;
            obj.err_z_prev = 0; obj.err_z_prevprev = 0; obj.Serr_z = 0; obj.Derr_z = 0;
            obj.err_phi_prev = 0; obj.err_phi_prevprev = 0; obj.Serr_phi = 0; obj.Derr_phi = 0;
            obj.err_theta_prev = 0; obj.err_theta_prevprev = 0; obj.Serr_theta = 0; obj.Derr_theta = 0;
            obj.err_psi_prev = 0; obj.err_psi_prevprev = 0; obj.Serr_psi = 0; obj.Derr_psi = 0;

            obj.Err_x = obj.err_x;
            obj.Err_y = obj.err_y;
            obj.Err_z = obj.err_z;
            obj.Err_phi = obj.err_phi;
            obj.Err_theta = obj.err_theta;
            obj.Err_psi = obj.err_psi;

            % PID gains
            updateGains(obj, gains);

            % Dynamics

            obj.R_psi = @(x) [cos(x(9)),    -sin(x(9)),     0;
                              sin(x(9)),     cos(x(9)),     0;
                                      0,             0,     1];

            obj.R_theta = @(x) [cos(x(8)),      0,      sin(x(8));
                                        0,      1,              0;
                                -sin(x(8)),     0,       cos(x(8))];

            obj.R_phi = @(x) [1,        0,              0;
                              0,        cos(x(7)),      -sin(x(7));
                              0,        sin(x(7)),      cos(x(7))];

            obj.rotMat = @(x) obj.R_psi(x) * obj.R_theta(x) * obj.R_phi(x);

            obj.Force = @(u) [0;
                              0;
                              u(1)];

            obj.Torque = @(u) [u(2);
                               u(3);
                               u(4)];

            obj.f = @(x, u) [x(4:6);
                (1/obj.m).*obj.rotMat(x)*obj.Force(u) + [0; 0; -obj.g];
                x(10:12);
                obj.I \ (obj.Torque(u) - cross(x(10:12), obj.I*x(10:12)))
                ];

            obj.f_lin = @(x, u) obj.A*x + obj.B*u;

            if (abs(obj.control) == 1)
                obj.func = obj.f_lin;
            else
                obj.func = obj.f;
            end

            obj.traj = obj.x0';

            % Navigation
            obj.estimation = estimation;

            obj.sigma_gyro = 0.005 * sqrt(1/obj.sampleTime) * pi/180;
            obj.sigma_acc = 400*(1e-6) * sqrt(1/obj.sampleTime) / obj.g;

            obj.x_est = obj.x;
            obj.r_est = obj.x_est(1:3);
            obj.dr_est = obj.x_est(4:6);
            obj.att_est = obj.x_est(7:9);
            obj.w_est = obj.x_est(10:12);
            
            obj.traj_est = obj.x_est';

            obj.error_est = obj.xDes - obj.x_est;

            obj.err_x_est = obj.error_est(1);
            obj.err_y_est = obj.error_est(2);
            obj.err_z_est = obj.error_est(3);
            obj.err_phi_est = obj.error_est(7);
            obj.err_theta_est = obj.error_est(8);
            obj.err_psi_est = obj.error_est(9);

            obj.Err_x_est = obj.err_x_est;
            obj.Err_y_est = obj.err_y_est;
            obj.Err_z_est = obj.err_z_est;
            obj.Err_phi_est = obj.err_phi_est;
            obj.Err_theta_est = obj.err_theta_est;
            obj.Err_psi_est = obj.err_psi_est;

            % obj.h = @(x, u) [obj.rotMat(x)' * ((1/obj.m).*obj.rotMat(x)*[0; 0; u(1)] + [0;0;-obj.g]);
            %     x(10);
            %     x(11);
            %     x(12)
            %     ];

            obj.h = @(x, u) [
                (1/obj.m).*[0; 0; u(1)] + obj.rotMat(x)'*[0;0;-obj.g];
                x(10:12);
                ];

            obj.P_est = 0.*eye(12);
            obj.Q = 0.*eye(12).*1e-0;
            obj.R_cov = diag([(obj.sigma_acc^2).*ones(3,1); ...
                              (obj.sigma_gyro^2).*ones(3,1)]);

            % Control
            obj.control = control;

            % Linearization
            if (obj.control ~= 0 && obj.control ~= 3)
                obj.linearizeHovering();
            end

            % Lyapunov function
            Ek0 = 0.5 * obj.m * norm(obj.dr)^2;
            Ep0 = obj.m * obj.g * obj.x(3);

            V0 = Ek0 + Ep0;

            [obj.V, obj.dV] = lyapunov(obj, V0);

        end

        function obj = updateGains(obj, gains) 
            % Controllers gains
            obj.kP_T = gains.kP_T;
            obj.kI_T = gains.kI_T;
            obj.kD_T = gains.kD_T;

            obj.kP_phi = gains.kP_phi;
            obj.kI_phi = gains.kI_phi;
            obj.kD_phi = gains.kD_phi;

            obj.kP_theta = gains.kP_theta;
            obj.kI_theta = gains.kI_theta;
            obj.kD_theta = gains.kD_theta;

            obj.kP_R = gains.kP_R;
            obj.kI_R = gains.kI_R;
            obj.kD_R = gains.kD_R;

            obj.kP_P = gains.kP_P;
            obj.kI_P = gains.kI_P;
            obj.kD_P = gains.kD_P;

            obj.kP_Y = gains.kP_Y;
            obj.kI_Y = gains.kI_Y;
            obj.kD_Y = gains.kD_Y;   
        end

        function updateDesiredState(obj, x, y, z, psi) 
            obj.rDes(1) = x;
            obj.rDes(2) = y;
            obj.rDes(3) = z;
            obj.attDes(3) = psi;

            obj.xDes = [obj.rDes; 0; 0; 0; obj.attDes; 0; 0; 0];
        end

        %% Get State 
        function state = getState(obj)
            state = obj.x;
        end

        %% CMD computation
        function obj = motorCMD(obj)

            if (obj.control == 3) 
                % MPC
                obj.u = obj.MPC();
            else
                if (obj.control > 0)
                    % LQR
                    obj.u_lin = obj.K * (obj.xDes - obj.x);
                    obj.u1 = obj.u_lin(1);
                    obj.u2 = obj.u_lin(2);
                    obj.u3 = obj.u_lin(3);
                    obj.u4 = obj.u_lin(4);
                else
                    % PID
                    obj.u1 = obj.altitudeCtrl();         % -> T
                    obj.attDes(1) = obj.phiCtrl();       % -> phi_des
                    obj.xDes(7) = obj.attDes(1);
                    obj.attDes(2) = obj.thetaCtrl();     % -> theta_des
                    obj.xDes(8) = obj.attDes(2);
                    obj.u2 = obj.RCtrl();                % -> U2 = Mx
                    obj.u3 = obj.PCtrl();                % -> U3 = My
                    obj.u4 = obj.YCtrl();                % -> U4 = Mz
                end

                obj.u = [obj.u1; obj.u2; obj.u3; obj.u4];
            end
                
                % Saturator
                obj.u(1) = min(100, max(0, obj.u(1)));
                obj.u(2) = max(-100, min(100, obj.u(2)));
                obj.u(3) = max(-100, min(100, obj.u(3)));
                obj.u(4) = max(-100, min(100, obj.u(4)));
        end

        function obj = integrateSystem(obj, state, cmd)

            % Integrate system of odes
            obj.dx = obj.func(state, cmd);

            % Explicit Euler integration method
            obj.x = state + ( obj.dx .* obj.dt );

            % % Runge-Kutta 4
            % obj.x = obj.rk4(obj.func, state, cmd, obj.dt);

            % % ODE45
            % [~, y] = ode45(@(t, x) obj.func(x, cmd), [obj.t(end), obj.t(end) + obj.dt], state);
            % obj.x = y(end, :)';

            % Update external force, torque 
            obj.F = obj.Force(cmd);
            obj.Fx = obj.F(1);
            obj.Fy = obj.F(2);
            obj.Fz = obj.F(3);

            obj.M = obj.Torque(cmd);
            obj.Mx = obj.M(1);
            obj.My = obj.M(2);
            obj.Mz = obj.M(3);
        end
        
        %% Update State
        function obj = updateState(obj)

            % Update time 
            obj.t = [obj.t; obj.t(end) + obj.dt];

            % SampleTime counter
            obj.counter = obj.counter + 1;


            if obj.counter >= 1/(obj.dt/obj.sampleTime)

                % State Estimation
                if (obj.estimation)
                    obj.Estimator();
                else
                    obj.x_est = obj.x;
                end

                % Error estimation
                obj.error_est = obj.xDes - obj.x_est;
                obj.err_x_est = obj.error_est(1);
                obj.err_y_est = obj.error_est(2);
                obj.err_z_est = obj.error_est(3);
                obj.err_phi_est = obj.error_est(7);
                obj.err_theta_est = obj.error_est(8);
                obj.err_psi_est = obj.error_est(9);

                obj.Err_x_est = [obj.Err_x_est, obj.err_x_est];
                obj.Err_y_est = [obj.Err_y_est, obj.err_y_est];
                obj.Err_z_est = [obj.Err_z_est, obj.err_z_est];
                obj.Err_phi_est = [obj.Err_phi_est, obj.err_phi_est];
                obj.Err_theta_est = [obj.Err_theta_est, obj.err_theta_est];
                obj.Err_psi_est = [obj.Err_psi_est, obj.err_psi_est];

                % Motor Commands
                obj.u_old = obj.u;
                obj.motorCMD();

                obj.counter = 0;
            end
            obj.traj_est = [obj.traj_est; obj.x_est'];
            obj.U = [obj.U; obj.u'];

            
            % Integrate system of odes.
            obj.integrateSystem(obj.x, obj.u);

            % Update each state
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.att = obj.x(7:9);
            obj.w = obj.x(10:12);

            % Update trajectory
            obj.traj = [obj.traj; obj.x'];

            % Update Lyapunov function
            [V, dV] = lyapunov(obj, obj.V(end));

            obj.V = [obj.V, V];
            obj.dV = [obj.dV, dV];
            
            % Update Error
            obj.error = obj.xDes - obj.x;
            obj.err_x = obj.error(1);
            obj.err_y = obj.error(2);
            obj.err_z = obj.error(3);
            obj.err_phi = obj.error(7);
            obj.err_theta = obj.error(8);
            obj.err_psi = obj.error(9);

            obj.Err_x = [obj.Err_x, obj.err_x];
            obj.Err_y = [obj.Err_y, obj.err_y];
            obj.Err_z = [obj.Err_z, obj.err_z];
            obj.Err_phi = [obj.Err_phi, obj.err_phi];
            obj.Err_theta = [obj.Err_theta, obj.err_theta];
            obj.Err_psi = [obj.Err_psi, obj.err_psi];

        end

        %% Linearization (around Hovering)
        function obj = linearizeHovering(obj)

            % Compute A-matrix
            obj.A = zeros(12,12);
            obj.A(1, 4) = 1;
            obj.A(2, 5) = 1;
            obj.A(3, 6) = 1;
            obj.A(4, 7) = obj.g*sin(obj.attDes(3));
            obj.A(4, 8) = obj.g*cos(obj.attDes(3));
            obj.A(5, 7) = -obj.g*cos(obj.attDes(3));
            obj.A(5, 8) = obj.g*sin(obj.attDes(3));
            obj.A(7, 10) = 1;
            obj.A(8, 11) = 1;
            obj.A(9, 12) = 1;

            % Compute B-matrix
            obj.B = zeros(12, 4);
            obj.B(6, 1) = 1/obj.m;
            obj.B(10, 2) = 1/obj.Ix;
            obj.B(11, 3) = 1/obj.Iy;
            obj.B(12, 4) = 1/obj.Iz;

            % LQR controller
            % Verifying modes stability
            eigs = eig(obj.A);
            stability = 1;
            for i = 1:length(eigs)
                if (real(eigs(i)) >= 0)
                    fprintf("unstable mode: lambda %d = %f\n", i, eigs(i));
                    stability = 0;
                end
            end

            if (~stability)
                fprintf("! Linearized system is unstable !\n");
            end

            % Verifying controllability
            Kc = ctrb(obj.A, obj.B);
            if (rank(Kc) < 12)
                disp("Linear system not fully controllable");
                fprintf("rank(Kc) = %d\n", rank(Kc));
            else
                disp("Linear system fully controllable");
                fprintf("rank(Kc) = %d\n", rank(Kc));

                % LQR control design
                % State variables weigths;
                d = zeros(12, 1);
                d(1:3) = 10;        % x, y, z
                d(3) = 1e3;
                d(4:6) = 1;        % x_dot, y_dot, z_dot
                d(7:9) = 10;        % phi, theta, psi
                d(10:end) = 1;     % phi_dot, theta_dot, psi_dot
                % Q matrix computation
                Q_mat = diag(d);

                % Input variables weights
                d = 1.*ones(4,1);
                d(1) = 0.01;
                % R matrix computation
                R_mat = diag(d);

                % Control matrix computation
                obj.K = lqr(obj.A, obj.B, Q_mat, R_mat);

            end
        end

        %% Guidance
        
        % Estimator
        function obj = Estimator(obj)
            obj.noise_acc = obj.sigma_acc .* randn(3, 1);
            obj.acc_meas = obj.rotMat(obj.x)'*( obj.dx(4:6) + [0;0;-obj.g] ) + obj.noise_acc;
            
            obj.noise_gyro = obj.sigma_gyro .* randn(3, 1);
            obj.gyro_meas = obj.dx(7:9) + obj.noise_gyro;

            obj.y = [obj.acc_meas; obj.gyro_meas];

            obj.KalmanFilter();

            obj.r_est = obj.x_est(1:3);
            obj.dr_est = obj.x_est(4:6);
            obj.att_est = obj.x_est(7:9);
            obj.w_est = obj.x_est(10:12);
            % obj.traj_est = [obj.traj_est; obj.x_est'];
        end

        % Kalman filter
        function obj = KalmanFilter(obj) 

            % Prediction
            % obj.x_pred = obj.x;
            % obj.x_pred = obj.x_est;
            obj.x_pred = obj.x_est + obj.func(obj.x_est, obj.u).*obj.sampleTime;
            % obj.x_pred = obj.func(obj.x_est, obj.u);

            % AJ = obj.JacobianA(obj.func, obj.x_est);
            AJ = obj.JacobianA(obj.func, obj.x_pred);

            obj.P_pred = AJ * obj.P_est * AJ' + obj.Q;

            % Correction
            obj.y_pred = obj.h(obj.x_pred, obj.u);
            HJ = obj.JacobianH(obj.h, obj.x_pred);

            % Kalman Gain
            obj.Kg = obj.P_pred * HJ' / (HJ * obj.P_pred * HJ' + obj.R_cov);

            % Estimation
            obj.x_est = obj.x_pred + ( obj.Kg * (obj.y - obj.y_pred));

            obj.P_est = (eye(12) - obj.Kg * HJ) * obj.P_pred;
        end

        function J = JacobianA(obj, f, x)
            dX = 1e-3;
            J = zeros(12, 12);

            for i = 1:12 % Row (derive with respect to the i-th component)
                for k = 1:12  % Column
                    xEst1 = x;
                    xEst2 = x;
                    xEst1(k) = x(k) + dX;
                    xEst2(k) = x(k) - dX;

                    j1 = f(xEst1, obj.u);
                    j2 = f(xEst2, obj.u);

                    J(i, k) = (j1(i) - j2(i)) / (2 * dX);
                end
            end
        end


        function J = JacobianH(obj, f, x)
            dX = 1e-3;
            J = zeros(6, 12);  % 6 osservazioni, 12 variabili di stato

            for i = 1:6 % Row (derive with respect to the i-th observation)
                for k = 1:12  % Column
                    xEst1 = x;
                    xEst2 = x;
                    xEst1(k) = x(k) + dX;
                    xEst2(k) = x(k) - dX;

                    j1 = f(xEst1, obj.u);
                    j2 = f(xEst2, obj.u);

                    J(i, k) = (j1(i) - j2(i)) / (2 * dX);
                end
            end
        end

        %% Controllers

        % Model Predictive Controller (non-linear)
        function u_mpc = MPC(obj)

            % Define parameters
            dT = 0.1;                % time interval
            N = 25;                     % Prediction horizon
            % alpha = 0.001;
            % du = 0.01;
            % grad_iters = 100;

            % Initial conditions
            x_0 = obj.x;

            % Desired state
            x_des = obj.xDes;

            % Inputs limits
            u_min = repmat([0; -100; -100; -100], N, 1);  % Repeat min limits N times
            u_max = repmat([100; 100; 100; 100], N, 1);   % Repeat max limits N times

            % Weigthing matrix
            Q = eye(6)*10;
            % Q(3,3) = Q(3,3);
            Q(3,3) = 100;
            Q(6,6) = 50;
            R = eye(4)*5;

            % % Dynamics
            % f = @(x, u) [x(4);
            %     x(5);
            %     x(6);
            %     (1/obj.m)*( sin(x(9))*sin(x(7)) + cos(x(9))*sin(x(8))*cos(x(7)) )*u(1);
            %     (1/obj.m)*( -cos(x(9))*sin(x(7)) + sin(x(9))*sin(x(8))*cos(x(7)) )*u(1);
            %     (1/obj.m)*cos(x(8))*cos(x(7))*u(1) - obj.g;
            %     x(10);
            %     x(11);
            %     x(12);
            %     (u(2)/obj.Ix) + (obj.Iy-obj.Iz)*x(11)*x(12)/obj.Ix;
            %     (u(3)/obj.Iy) + (obj.Iz-obj.Ix)*x(10)*x(12)/obj.Iy;
            %     (u(4)/obj.Iz) + (obj.Ix-obj.Iy)*x(10)*x(11)/obj.Iz
            %     ];

            % Optimal u
            options = optimoptions('fmincon', ...
                'Algorithm', 'interior-point', ...                 % Algoritmo scelto
                'MaxFunctionEvaluations', 50000, ...     % Limite massimo delle valutazioni
                'MaxIterations', 1000, ...               % Limite massimo delle iterazioni
                'Display', 'final', ...                   % Mostra il progresso
                'UseParallel',false,...
                'FiniteDifferenceType', 'central');  % Differenze finite più accurate
            [u_opt, fval] = fmincon(@(u) cost_function(x_0, u, x_des, Q, R, N, obj.f, dT), ...
                zeros(4*N, 1), [], [], [], [], u_min, u_max, [], options);

            u_mpc = u_opt(1:4);

            % Cost function
            function J = cost_function(x0, u, xref, Q, R, N, f, dt)
                J = 0;
                x = x0;
                for k = 1:N
                    u_k = u((k-1)*4+1:k*4); % Prendi l'input di controllo per il k-esimo passo
                    x = f(x, u_k) * dt + x; % Propaga lo stato con la dinamica non lineare
                    err_lin = x(1:3) - xref(1:3);
                    err_ang = x(7:9) - xref(7:9);
                    err = [err_lin; err_ang];
                    J = J + err' * Q * err + u_k' * R * u_k; % Funzione di costo
                end
            end

            % % Gradient optimization
            % for i = 1:grad_iters
            %
            % J = 0;
            % J1 = zeros(4,1);
            % grad_J = zeros(4,1);
            %
            %     for k = 1:N
            %
            %         u_k = U(k, :)';
            %         x_prev = x;
            %         x = x + f(x, u_k) * dT;
            %
            %         error = x_des - x;
            %
            %         J = J + (error'*Q*error) + u_k'*R*u_k;
            %
            %         for j = 1:4
            %             u1 = u_k;
            %             u1(j) = u1(j) + du;
            %             x1 = x_prev + f(x_prev, u1);
            %
            %             error1 = x_des - x1;
            %             J1(j) = J1(j) + error1'*Q*error1 + u1'*R*u1;
            %         end
            %
            %         grad_J = (J1 - J.*ones(4,1))./du;
            %
            %         U(k, :) = U(k, :) - alpha.*grad_J';
            %
            %     end
            %
            %     % U(1, :)
            %
            % end
            % u_mpc = U(1, :);

        end

        % Altitude controller
        function T = altitudeCtrl(obj)
            % Error computation
            obj.err_z_prev = obj.Err_z_est(end-1);

            % Integral term computation
            if (min(size(obj.Err_z_est)) > 1)
                obj.err_z_prevprev = obj.Err_z_est(end-2);
                obj.Serr_z = obj.Serr_z + ...
                    obj.simpson(obj.err_z, obj.err_z_prev, obj.err_z_prevprev);
            else
                obj.Serr_z = obj.Serr_z + ...
                                   obj.trap(obj.err_z, obj.err_z_prev);
            end

            % Derivative term computation
            obj.Derr_z = obj.bkwFD(obj.err_z, obj.err_z_prev);
            
            % PID controller
            T = (obj.kP_T * obj.err_z_est + ...
                     obj.kI_T * obj.Serr_z + ...
                     obj.kD_T * obj.Derr_z);
        end
        
        % Phi desidered controller
        function phi_des = phiCtrl(obj)
            % Error computation
            obj.err_y_prev = obj.Err_y_est(end-1);

            % Integral term computation
            if (min(size(obj.Err_y_est)) > 1)
                obj.err_y_prevprev = obj.Err_y_est(end-2);
                obj.Serr_y = obj.Serr_y + ...
                    obj.simpson(obj.err_y, obj.err_y_prev, obj.err_y_prevprev);
            else
                obj.Serr_y = obj.Serr_y + ...
                                   obj.trap(obj.err_y, obj.err_y_prev);
            end
            
            % Derivative term computation
            obj.Derr_y = obj.bkwFD(obj.err_y, obj.err_y_prev);

            % PID controller
            phi_des = obj.kP_phi * obj.err_y_est + ...
                      obj.kI_phi * obj.Serr_y + ...
                      obj.kD_phi * obj.Derr_y;
        end

        % Theta desidered controller
        function theta_des = thetaCtrl(obj)
            % Error computation
            obj.err_x_prev = obj.Err_x_est(end-1);

            % Integral term computation
            if (min(size(obj.Err_x_est)) > 1)
                obj.err_x_prevprev = obj.Err_x_est(end-2);
                obj.Serr_x = obj.Serr_x + ...
                    obj.simpson(obj.err_x, obj.err_x_prev, obj.err_x_prevprev);
            else
                obj.Serr_x = obj.Serr_x + ...
                                   obj.trap(obj.err_x, obj.err_x_prev);
            end
            
            % Derivative term computation
            obj.Derr_x = obj.bkwFD(obj.err_x, obj.err_x_prev);

            % PID controller
            theta_des = obj.kP_theta * obj.err_x_est + ...
                        obj.kI_theta * obj.Serr_x + ...
                        obj.kD_theta * obj.Derr_x;
        end

        % Roll controller - phi_error based
        % Control Y position
        function R = RCtrl(obj) % U2
            % Error computation
            obj.err_phi_prev = obj.Err_phi_est(end-1);

            % Integral term computation
            if (min(size(obj.Err_phi_est)) > 1)
                obj.err_phi_prevprev = obj.Err_phi_est(end-2);
                obj.Serr_phi = obj.Serr_phi + ...
                    obj.simpson(obj.err_phi, obj.err_phi_prev, obj.err_phi_prevprev);
            else
                obj.Serr_phi = obj.Serr_phi + ...
                                   obj.trap(obj.err_phi, obj.err_phi_prev);
            end

            % Derivative term computation
            obj.Derr_phi = obj.bkwFD(obj.err_phi, obj.err_phi_prev);

            % PID controller
            R = obj.kP_R * obj.err_phi_est + ...
                obj.kI_R * obj.Serr_phi + ...
                obj.kD_R * obj.Derr_phi;
        end

        % Pitch controller - theta_error based
        % Control X position
        function P = PCtrl(obj)
            % Error computation
            obj.err_theta_prev = obj.Err_theta_est(end-1);

            % Integral term computation
            if (min(size(obj.Err_theta_est)) > 1)
                obj.err_theta_prevprev = obj.Err_theta_est(end-2);
                obj.Serr_theta = obj.Serr_theta + ...
                    obj.simpson(obj.err_theta, obj.err_theta_prev, obj.err_theta_prevprev);
            else
                obj.Serr_theta = obj.Serr_theta + ...
                                   obj.trap(obj.err_theta, obj.err_theta_prev);
            end

            % Derivative term computation
            obj.Derr_theta = obj.bkwFD(obj.err_theta, obj.err_theta_prev);

            % PID controller
            P = obj.kP_P * obj.err_theta_est + ...
                obj.kI_P * obj.Serr_theta + ...
                obj.kD_P * obj.Derr_theta;
        end

        % Yaw controller
        function Y = YCtrl(obj)
            % Error computation
            obj.err_psi_prev = obj.Err_psi_est(end-1);
            
            % Integral term computation
            if (min(size(obj.Err_psi_est)) > 1)
                obj.err_psi_prevprev = obj.Err_psi_est(end-2);
                obj.Serr_psi = obj.Serr_psi + ...
                               obj.simpson(obj.err_psi, obj.err_psi_prev, obj.err_psi_prevprev);
            else 
                obj.Serr_psi = obj.Serr_psi + ...
                    obj.trap(obj.err_psi, obj.err_psi_prev);
            end

            % Derivative term computation
            obj.Derr_psi = obj.bkwFD(obj.err_psi, obj.err_psi_prev);
            
            % PID controller
            Y = obj.kP_Y * obj.err_psi_est + ...
                obj.kI_Y * obj.Serr_psi + ...
                obj.kD_Y * obj.Derr_psi;
        end

        %% Backward Finite Differences
        function d = bkwFD(obj, err, err_prev)
            d = (err - err_prev)/obj.dt;
        end

        %% Runge-Kutta 4
        function y = rk4(~, func, x, u, h)
            
            k1 = func(x, u);
            k2 = func(x + h*k1/2, u);
            k3 = func(x + h*k2/2, u);
            k4 = func(x + h*k3, u);

            y = x + h * (k1 + 2*k2 + 2*k3 + k4) / 6;
        end

        %% Simpson integration
        function s = simpson(obj, err, err_prev, err_prevprev)
            s = (err_prevprev + 4*err_prev + err)*obj.dt/3;
        end

        %% Trapezium integration
        function s = trap(obj, err, err_prev)
            s = (err + err_prev)*obj.dt/2;
        end

        %% Lyapunov Function
        function [V, dV] = lyapunov(obj, V_prev)
            % Lyapunov function definition
            Ek = 0.5 * obj.m * norm(obj.dr)^2;
            Ep = obj.m * obj.g * obj.x(3);

            V = Ek + Ep;

            dV = (V - V_prev)/obj.dt;
        end
        %% Logger
        function logger(obj)
            % progress = 100 - (100 * (abs(obj.t(end) - obj.tf))/obj.tf);
            % fprintf('Progress:\t %f [perc] |\t', progress);
            fprintf('Time:\t %f [s] |\t', obj.t(end));
            fprintf('Inputs:\t [%f, %f, %f, %f] |\t', obj.u);
            % fprintf('Thrust:\t %f [N] |\t', obj.Fz);
            % fprintf('R: \t %f\t|\t', obj.R);
            % fprintf('phi_des: \t %f [deg]\t|\t', obj.attDes(1));
            fprintf('Err (x, y, z):\t [%f, %f, %f] [m] |\t', obj.err_x, obj.err_y, obj.err_z);
            % fprintf('Attitude:\t [%f, %f, %f] |\t', obj.att);
            fprintf('Altitude:\t %f [m] |\t', obj.x(3));

            % fprintf('W:\t [%f, %f, %f]', obj.w);
            % fprintf('F proj: [%f, %f, %f] [N]\t|\t', obj.o_R_b'*obj.F);
            % fprintf('Fz:\t %f [N]\t|\t', obj.dx(6)*obj.m);
            fprintf('\n');
        end
    end
end