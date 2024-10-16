classdef Drone2 < handle
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
        
        %% State Space
        x               % [x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi]
        r               % [x, y, z]
        dr              % [dx, dy, dz]
        att             % [phi, theta, psi]
        w               % [dphi, dtheta, dpsi]

        u               % Inputs
        u1              % Rotor speed 1
        u2              % Rotor speed 2
        u3              % Rotor speed 3
        u4              % Rotor speed 4

        % Commands
        T               
        R
        P
        Y

        dx              % System of odes

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
        err_x; err_x_prev; err_x_prevprev; Serr_x; Derr_x;
        err_y; err_y_prev; err_y_prevprev; Serr_y; Derr_y;
        err_z; err_z_prev; err_z_prevprev; Serr_z; Derr_z;

        err_phi; err_phi_prev; err_phi_prevprev; Serr_phi; Derr_phi;
        err_theta; err_theta_prev; err_theta_prevprev; Serr_theta; Derr_theta;
        err_psi; err_psi_prev; err_psi_prevprev; Serr_psi; Derr_psi;

        Err_x; Err_y; Err_z; Err_phi; Err_theta; Err_psi;

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
        F               % [3x1] [N]      External Force
        Fx                   
        Fy
        Fz

        T1              % [N]            Rotor Thrust
        T2
        T3
        T4

        Weight          % [3x1]          Weight force

        C1              % [Nm]           Rotor torque
        C2
        C3
        C4

        M               % [3x1] [Nm]     External Torque
        Mx
        My
        Mz

        o_R_b           % [3x3]          Rotation Matrix

        traj            % [tx12]         Time evolution of the state

        %% Linearization
        linear;         % flag

        A; 
        B;
        C;
        D;

        K;
        u_lin;

        %% Lyapunov
        V
        dV
       
    end

    methods

        %% Constructor
        function obj = Drone2(params, initialCondition, desideredState, gains, tspan, linear)
            
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

            % Desidered State
            obj.rDes = desideredState.rDes;
            obj.attDes = desideredState.attDes;
            obj.xDes = [obj.rDes; 0; 0; 0; obj.attDes; 0; 0; 0];
           
            % Errors
            obj.err_x = obj.rDes(1) - obj.x(1);
            obj.err_y = obj.rDes(2) - obj.x(2);
            obj.err_z = obj.rDes(3) - obj.x(3);
            obj.err_phi = obj.attDes(1) - obj.x(7);
            obj.err_theta = obj.attDes(2) - obj.x(8);
            obj.err_psi = obj.attDes(3) - obj.x(9);

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

            % % Controllers gains
            % obj.kP_T = gains.kP_T;
            % obj.kI_T = gains.kI_T;
            % obj.kD_T = gains.kD_T;
            % 
            % obj.kP_phi = gains.kP_phi;
            % obj.kI_phi = gains.kI_phi;
            % obj.kD_phi = gains.kD_phi;
            % 
            % obj.kP_theta = gains.kP_theta;
            % obj.kI_theta = gains.kI_theta;
            % obj.kD_theta = gains.kD_theta;
            % 
            % obj.kP_R = gains.kP_R;
            % obj.kI_R = gains.kI_R;
            % obj.kD_R = gains.kD_R;
            % 
            % obj.kP_P = gains.kP_P;
            % obj.kI_P = gains.kI_P;
            % obj.kD_P = gains.kD_P;
            % 
            % obj.kP_Y = gains.kP_Y;
            % obj.kI_Y = gains.kI_Y;
            % obj.kD_Y = gains.kD_Y;

            updateGains(obj, gains);

            % Dynamics
            obj.dx = zeros(12,1);
            obj.u = zeros(4,1);

            obj.traj = obj.x0';

            % Linearization
            obj.linear = linear;
            if (obj.linear ~= 0)
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
    
        %% Dynamics
        function o_R_b = evalRotationMatrix(obj, angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);

            % Rotation around Z axis (yaw)
            R_psi = [cos(psi),       -sin(psi),        0;
                     sin(psi),        cos(psi),        0;
                            0,               0,        1];

            % Rotation around Y' axis (pitch)
            R_theta = [cos(theta),     0,      sin(theta);
                                0,     1,               0;
                      -sin(theta),     0,      cos(theta)];

            % Rotation around X'' axis (roll)
            R_phi = [1,                0,                0;
                     0,         cos(phi),        -sin(phi);
                     0,         sin(phi),        cos(phi)];
            
            % Rotation matrix from o -> b
            o_R_b = R_psi * R_theta * R_phi;
            % Rotation matrix from b -> o
            % o_R_b = o_R_b';
        end

        function obj = ODEs(obj)
            % Non-linear ODE system
            % F = @(t,x,u) [
            %              x(4);
            %              x(5);
            %              x(6);
            %              (k_f/m)*(sin(x(9))*sin(x(7)) + cos(x(9))*sin(x(8))*cos(x(7)))*u1;
            %              (k_f/m)*(-cos(x(9))*sin(x(7)) + sin(x(9))*sin(x(8))*cos(x(7)))*u1);
            %              (k_f/m)*(cos(x(8))*cos(x(7)))*u1 - g;
            %              x(10);
            %              x(11);
            %              x(12);
            %              u2/a + ((b-c)/a)*x(11)*x(12);
            %              u3/b + ((c-a)/b)*x(10)*x(12);
            %              u4/c + ((a-b)/c)*x(10)*x(11);
            %              ];
            
            if (abs(obj.linear) == 1)
                obj.dx = obj.A*obj.x + obj.B*obj.u;
            else
                obj.o_R_b = obj.evalRotationMatrix(obj.att);
                obj.dx(1:3) = obj.dr;
                obj.dx(4:6) = ( (1/obj.m) .* obj.o_R_b * obj.F ) + [0; 0; -obj.g];
                obj.dx(7:9) = obj.w;
                obj.dx(10:12) = obj.I \ (obj.M - cross(obj.w, obj.I*obj.w));
            end

        end

        %% CMD computation
        function obj = motorCMD(obj)

            if (true)
                if (mod(length(obj.t), 1) == 0)
                    obj.u = obj.MPC();
                    % obj.u(1) = obj.u(1) + obj.m*obj.g;
                else
                    obj.u = obj.u;
                end
            else
                if (obj.linear > 0)
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
        end

        function obj = cmdForceTorque(obj)

            obj.motorCMD();

            obj.F = [0;
                     0;
                     obj.u(1)];
            obj.Fx = obj.F(1);
            obj.Fy = obj.F(2);
            obj.Fz = obj.F(3);

            obj.M = [obj.u(2);
                     obj.u(3);
                     obj.u(4)];
            obj.Mx = obj.M(1);
            obj.My = obj.M(2);
            obj.Mz = obj.M(3);
        end
        
        %% Update State
        function obj = updateState(obj)

            % Update time 
            obj.t = [obj.t; obj.t(end) + obj.dt];

            % Compute motor cmd
            obj.cmdForceTorque();
 
            % Build system of odes
            obj.ODEs();
            
            % Integrate system of odes.
            obj.x = obj.x + (obj.dx .* obj.dt);

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
        
        %% Controllers

        % Model Predictive Controller (non-linear)
        function u_mpc = MPC(obj)

            % Define parameters
            dT = obj.dt;                % time interval
            N = 200;                     % Prediction horizon
            % alpha = 0.001;
            % du = 0.01;
            % grad_iters = 100;
            
            % Initial conditions
            x_0 = obj.x;

            % Desired state
            x_des = obj.xDes;

            % Inputs limits
            u_min = [0; -100; -100; -100];
            u_max = [100; 100; 100; 100];

            % Weigthing matrix
            Q = eye(12).*100;
            Q(3,3) = 1e5;
            R = eye(4).*100;

            % Dynamics
            f = @(x, u) [x(4);
                x(5);
                x(6);
                (1/obj.m)*( sin(x(9))*sin(x(7)) + cos(x(9))*sin(x(8))*cos(x(7)) )*u(1);
                (1/obj.m)*( -cos(x(9))*sin(x(7)) + sin(x(9))*sin(x(8))*cos(x(7)) )*u(1);
                (1/obj.m)*cos(x(8))*cos(x(7))*u(1) - obj.g;
                x(10);
                x(11);
                x(12);
                (u(2)/obj.Ix) + (obj.Iy-obj.Iz)*x(11)*x(12)/obj.Ix;
                (u(3)/obj.Iy) + (obj.Iz-obj.Ix)*x(10)*x(12)/obj.Iy;
                (u(4)/obj.Iz) + (obj.Ix-obj.Iy)*x(10)*x(11)/obj.Iz
                ];


            % Optimal u
            [u_opt, fval] = fmincon(@(u) cost_function(x_0, u, x_des, Q, R, N, f, dT), ...
                zeros(4*N, 1), [], [], [], [], u_min, u_max);

            u_mpc = u_opt(1:4);

            % Cost function
            function J = cost_function(x0, u, xref, Q, R, N, f, dt)
                J = 0;
                x = x0;
                for k = 1:N
                    u_k = u((k-1)*4+1:k*4); % Prendi l'input di controllo per il k-esimo passo
                    x = f(x, u_k) * dt + x; % Propaga lo stato con la dinamica non lineare
                    J = J + (x - xref)' * Q * (x - xref) + u_k' * R * u_k; % Funzione di costo
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
            if (length(obj.t) > 1)          % err_z_prevprev available
                obj.err_z_prevprev = obj.err_z_prev;
                obj.err_z_prev = obj.err_z;
                obj.err_z = obj.rDes(3) - obj.x(3); % z_des - z
                
                obj.Serr_z = obj.Serr_z + ...
                               obj.simpson(obj.err_z, obj.err_z_prev, obj.err_z_prevprev);
            else    
                obj.err_z_prev = obj.err_z;
                obj.err_z = obj.rDes(3) - obj.x(3); % z_des - z

                obj.Serr_z = obj.Serr_z + ...
                             obj.trap(obj.err_z, obj.err_z_prev);
            end

            obj.Derr_z = obj.bkwFD(obj.err_z, obj.err_z_prev);
            
            % PID controller
            % T = sqrt((obj.m*obj.g)/(4*obj.k_f)); % minimum required to balance weight
            T = (obj.kP_T * obj.err_z + ...
                     obj.kI_T * obj.Serr_z + ...
                     obj.kD_T * obj.Derr_z);
        end
        
        % Phi desidered controller
        function phi_des = phiCtrl(obj)
            if (length(obj.t) > 1)      % err_x_prevprev available  
                % Error computation
                obj.err_x_prevprev = obj.err_x_prev;
                obj.err_x_prev = obj.err_x;
                obj.err_x = obj.rDes(1) - obj.x(1); % x_des - x

                obj.Serr_x = obj.Serr_x + ...
                               obj.simpson(obj.err_x, obj.err_x_prev, obj.err_x_prevprev);
            else
                obj.err_x_prev = obj.err_x;
                obj.err_x = obj.rDes(1) - obj.x(1); 

                obj.Serr_x = obj.Serr_x + ...
                             obj.trap(obj.err_x, obj.err_x_prev);
            end

            obj.Derr_x = obj.bkwFD(obj.err_x, obj.err_x_prev);

            % PID controller
            phi_des = obj.kP_phi * obj.err_x + ...
                      obj.kI_phi * obj.Serr_x + ...
                      obj.kD_phi * obj.Derr_x;
        end

        % Theta desidered controller
        function theta_des = thetaCtrl(obj)
            % Error computation
            if (length(obj.t) > 1)       % err_y_prevprev available
                obj.err_y_prevprev = obj.err_y_prev;
                obj.err_y_prev = obj.err_y;
                obj.err_y = obj.rDes(2) - obj.x(2); % y_des - y

                obj.Serr_y = obj.Serr_y + ...
                               obj.simpson(obj.err_y, obj.err_y_prev, obj.err_y_prevprev);
            else
                obj.err_y_prev = obj.err_y;
                obj.err_y = obj.rDes(2) - obj.x(2); % y_des - y

                obj.Serr_y = obj.Serr_y + ...
                               obj.trap(obj.err_y, obj.err_y_prev);
            end

            obj.Derr_y = obj.bkwFD(obj.err_y, obj.err_y_prev);

            % PID controller
            theta_des = obj.kP_theta * obj.err_y + ...
                        obj.kI_theta * obj.Serr_y + ...
                        obj.kD_theta * obj.Derr_y;
        end

        % Roll controller - phi_error based
        % Control Y position
        function R = RCtrl(obj) % U2
            % Error computation
            if (length(obj.t) > 1)      % err_phi_prevprev available
                obj.err_phi_prevprev = obj.err_phi_prev;
                obj.err_phi_prev = obj.err_phi;
                obj.err_phi = obj.attDes(1) - obj.att(1); % phi_des - phi

                obj.Serr_phi = obj.Serr_phi + ...
                               obj.simpson(obj.err_phi, obj.err_phi_prev, obj.err_phi_prevprev);
            else
                obj.err_phi_prev = obj.err_phi;
                obj.err_phi = obj.attDes(1) - obj.att(1); % phi_des - phi;

                obj.Serr_phi = obj.Serr_phi + ...
                               obj.trap(obj.err_phi, obj.err_phi_prev);
            end

            obj.Derr_phi = obj.bkwFD(obj.err_phi, obj.err_phi_prev);

            % PID controller
            R = obj.kP_R * obj.err_phi + ...
                obj.kI_R * obj.Serr_phi + ...
                obj.kD_R * obj.Derr_phi;
        end

        % % Roll controller - y_error based 
        % % Control Y position
        % function R = RCtrl(obj)
        %     % Error computation
        %     if (length(obj.t) > 1)      % err_y_prevprev available  
        %         % Error computation
        %         obj.err_y_prevprev = obj.err_y_prev;
        %         obj.err_y_prev = obj.err_y;
        %         obj.err_y = obj.rDes(2) - obj.x(2); % y_des - y
        % 
        %         obj.Serr_y = obj.Serr_y + ...
        %                        obj.simpson(obj.err_y, obj.err_y_prev, obj.err_y_prevprev);
        %     else
        %         obj.err_y_prev = obj.err_y;
        %         obj.err_y = obj.rDes(2) - obj.x(2); 
        % 
        %         obj.Serr_y = obj.Serr_y + ...
        %                      obj.trap(obj.err_y, obj.err_y_prev);
        %     end
        % 
        %     obj.Derr_y = obj.bkwFD(obj.err_y, obj.err_y_prev);
        % 
        %     % PID controller
        %     R = obj.kP_R * obj.err_y + ...
        %         obj.kI_R * obj.Serr_y + ...
        %         obj.kD_R * obj.Derr_y;
        % end

        % Pitch controller - theta_error based
        % Control X position
        function P = PCtrl(obj)
            % Error computation
            if (length(obj.t) > 1)      % err_theta_prevprev available
                obj.err_theta_prevprev = obj.err_theta_prev;
                obj.err_theta_prev = obj.err_theta;
                obj.err_theta = obj.attDes(2) - obj.att(2); % theta_des - theta

                obj.Serr_theta = obj.Serr_theta + ...
                               obj.simpson(obj.err_theta, obj.err_theta_prev, obj.err_theta_prevprev);
            else
                obj.err_theta_prev = obj.err_theta;
                obj.err_theta = obj.attDes(2) - obj.att(2); % theta_des - theta

                obj.Serr_theta = obj.Serr_theta + ...
                               obj.trap(obj.err_theta, obj.err_theta_prev);
            end

            obj.Derr_theta = obj.bkwFD(obj.err_theta, obj.err_theta_prev);

            % PID controller
            P = obj.kP_P * obj.err_theta + ...
                obj.kI_P * obj.Serr_theta + ...
                obj.kD_P * obj.Derr_theta;
        end

        % % Pitch controller - x_error based
        % % Control X position
        % function P = PCtrl(obj)
        %     % Error computation
        %     if (length(obj.t) > 1)       % err_y_prevprev available
        %         obj.err_x_prevprev = obj.err_x_prev;
        %         obj.err_x_prev = obj.err_x;
        %         obj.err_x = obj.rDes(1) - obj.x(1); % x_des - x
        % 
        %         obj.Serr_x = obj.Serr_x + ...
        %                        obj.simpson(obj.err_x, obj.err_x_prev, obj.err_x_prevprev);
        %     else
        %         obj.err_x_prev = obj.err_x;
        %         obj.err_x = obj.rDes(2) - obj.x(2); % x_des - x
        % 
        %         obj.Serr_x = obj.Serr_x + ...
        %                        obj.trap(obj.err_x, obj.err_x_prev);
        %     end
        % 
        %     obj.Derr_x = obj.bkwFD(obj.err_x, obj.err_x_prev);
        % 
        %     % PID controller
        %     P = obj.kP_P * obj.err_x + ...
        %         obj.kI_P * obj.Serr_x + ...
        %         obj.kD_P * obj.Derr_x;
        % end

        % Yaw controller
        function Y = YCtrl(obj)
            % Error computation
            if (length(obj.t) > 1)      % err_psi_prevprev available
                obj.err_psi_prevprev = obj.err_psi_prev;
                obj.err_psi_prev = obj.err_psi;
                obj.err_psi = obj.attDes(3) - obj.att(3); % phi - phi_des
                
                obj.Serr_psi = obj.Serr_psi + ...
                               obj.simpson(obj.err_psi, obj.err_psi_prev, obj.err_psi_prevprev);
            else
                obj.err_psi_prev = obj.err_psi;
                obj.err_psi = obj.attDes(3) - obj.att(3); % phi_des - phi

                obj.Serr_psi = obj.Serr_psi + ...
                               obj.trap(obj.err_psi, obj.err_psi_prev);
            end

            obj.Derr_psi = obj.bkwFD(obj.err_psi, obj.err_psi_prev);
            
            % PID controller
            Y = obj.kP_Y * obj.err_psi + ...
                obj.kI_Y * obj.Serr_psi + ...
                obj.kD_Y * obj.Derr_psi;
        end

        %% Backward Finite Differences
        function d = bkwFD(obj, err, err_prev)
            d = (err - err_prev)/obj.dt;
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
            progress = 100 - (100 * (abs(obj.t(end) - obj.tf))/obj.tf);
            fprintf('Progress:\t %f [perc] |\t', progress);
            fprintf('Thrust:\t %f [N] |\t', obj.Fz);
            % fprintf('R: \t %f\t|\t', obj.R);
            % fprintf('phi_des: \t %f [deg]\t|\t', obj.attDes(1));
            fprintf('Err (x, y, z):\t [%f, %f, %f] [m] |\t', obj.err_x, obj.err_y, obj.err_z);
            fprintf('Attitude:\t [%f, %f, %f] |\t', obj.att);
            fprintf('Altitude:\t %f [m] |\t', obj.x(3));

            % fprintf('W:\t [%f, %f, %f]', obj.w);
            % fprintf('F proj: [%f, %f, %f] [N]\t|\t', obj.o_R_b'*obj.F);
            % fprintf('Fz:\t %f [N]\t|\t', obj.dx(6)*obj.m);
            fprintf('\n');
        end
    end
end