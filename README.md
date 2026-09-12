# Autonomous Quadrotor Project

A multidisciplinary quadrotor project developed to study and implement the main engineering aspects of a small unmanned aerial vehicle, combining **flight dynamics, simulation, guidance and control, state estimation, electronics, embedded systems, hardware testing and IoT-based control**.

The project combines a MATLAB/Simulink simulation environment with the supporting embedded software, electronics, documentation, testing activities and web interface required for a complete quadrotor system.

---

## Overview

The project was developed to apply aerospace engineering concepts to a real-world autonomous flight system.

The main areas investigated throughout the project are:

* 6-DOF quadrotor flight dynamics
* Nonlinear system modeling
* Linearization and system analysis
* PID control
* Linear Quadratic Regulator (LQR)
* Nonlinear Model Predictive Control (NMPC)
* Kalman-filter-based state estimation
* Sensor fusion
* Trajectory tracking
* Controller tuning
* Embedded systems and Arduino
* Electronics and PCB design
* Hardware testing
* IoT-based drone control

The repository is organized into separate directories according to the different aspects of the project, from theoretical modeling and simulation to embedded implementation and system testing.

---

# Repository Structure

```text
Drone/
│
├── simulator/
│   ├── MATLAB and Simulink simulation files
│   └── Flight dynamics, control and state-estimation models
│
├── arduino_sketches/
│   ├── Main Arduino code
│   ├── Supporting functions
│   └── Required libraries and modules
│
├── electronics/
│   ├── Electrical schematics
│   └── PCB designs
│
├── docs/
│   ├── Project documentation
│   └── Component datasheets
│
├── testing/
│   └── Hardware and system testing
│
├── WebApp/
│   └── drone-web-app/
│       └── Web application for IoT-based drone control
│
└── README.md
```

Each directory addresses a different part of the overall system.

---

# Simulator

The `simulator/` directory contains the **MATLAB and Simulink simulation environment** developed to model, analyze and control the quadrotor before its physical implementation.

The simulator provides a model-based environment in which the vehicle dynamics, control algorithms, state-estimation techniques and trajectory-tracking performance can be developed and evaluated independently of the physical hardware.

The main components investigated within the simulator are:

* Nonlinear quadrotor dynamics
* Linearized hover model
* Stability and controllability analysis
* PID control
* LQR control
* Nonlinear Model Predictive Control
* Kalman-filter-based state estimation
* Controller tuning
* Closed-loop trajectory tracking
* Simulation visualization and animation

---

## Running the Simulator

The main entry point for the simulation is `simulator.m`, located in the `simulator/` directory.

Before running the simulation, open `simulator.m` and configure the vehicle parameters, initial conditions, desired state, simulation settings, controller, state estimator and visualization options.

Once the parameters have been configured, run:

```matlab
simulator
```

The simulation initializes the quadrotor model, executes the selected control and estimation architecture and generates the requested plots and animation.

---

### 1. Vehicle Parameters

The physical parameters of the quadrotor can be modified at the beginning of `simulator.m`.

```matlab
params.m = 0.8;                 % [kg] Mass

params.Ix = 15.67e-3;           % [kg m^2] Moment of inertia about X
params.Iy = 15.67e-3;           % [kg m^2] Moment of inertia about Y
params.Iz = 28.34e-3;           % [kg m^2] Moment of inertia about Z

params.armLength = 0.3;         % [m] Arm length
params.bodySize = 0.2;          % [m] Body size
params.rotRad = 0.1;             % [m] Rotor radius

params.k_f = 1.4e-5 / 9.54929658;  % Lift coefficient
params.k_m = 1.78e-6 / 9.54929658; % Torque coefficient

params.g = 9.81;                % [m/s^2] Gravitational acceleration
```

These parameters define the physical model used by the simulator.

The parameters that can be modified include:

| Parameter          | Description                               | Unit  |
| ------------------ | ----------------------------------------- | ----- |
| `params.m`         | Vehicle mass                              | kg    |
| `params.Ix`        | Principal moment of inertia about X       | kg·m² |
| `params.Iy`        | Principal moment of inertia about Y       | kg·m² |
| `params.Iz`        | Principal moment of inertia about Z       | kg·m² |
| `params.armLength` | Distance from the center to the rotor arm | m     |
| `params.bodySize`  | Approximate body size                     | m     |
| `params.rotRad`    | Rotor radius                              | m     |
| `params.k_f`       | Rotor lift coefficient                    | —     |
| `params.k_m`       | Rotor torque coefficient                  | —     |
| `params.g`         | Gravitational acceleration                | m/s²  |

The inertia matrix is then constructed as:

```matlab
params.I = diag([params.Ix, params.Iy, params.Iz]);
```

---

### 2. Numerical Integration

The simulation integration step is controlled by:

```matlab
params.dt = 0.001;       % [s]
```

The current implementation uses **explicit Euler integration** to propagate the nonlinear equations of motion.

Therefore, `params.dt` controls the numerical integration step and directly affects both the simulation resolution and computational cost.

A smaller value generally provides a finer temporal resolution at the expense of increased computational cost.

---

### 3. Initial Conditions

The initial state of the quadrotor is defined through the 12-state vector:

```matlab
x0 = zeros(12,1);
```

The state follows the convention:

```text
x0 = [x y z vx vy vz phi theta psi p q r]ᵀ
```

Individual initial conditions can be modified directly:

```matlab
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
```

This allows the user to investigate different initial flight conditions and attitude perturbations.

---

### 4. Desired State

The target position and attitude are defined through the `desideredState` structure:

```matlab
desideredState = struct();

desideredState.rDes = [-0.5, 1, 1]';    % [m]
desideredState.attDes = [0, 0, 0]';     % [rad]
```

The desired state consists of:

* Desired position: `rDes = [x, y, z]`
* Desired attitude: `attDes = [phi, theta, psi]`

For example, to command the quadrotor to a different position:

```matlab
desideredState.rDes = [1, 2, 1.5]';
```

The desired yaw angle can also be modified:

```matlab
desideredState.attDes = [0, 0, pi/2]';
```

---

### 5. Simulation Time

The simulation interval is defined using:

```matlab
t0 = 0;          % [s]
tmax = 10;       % [s]

tspan = [t0, tmax];
```

Modify `tmax` to change the total simulation duration.

For example:

```matlab
tmax = 20;
```

runs the simulation for 20 seconds.

---

### 6. Control Mode

The controller is selected using the `control` flag:

```matlab
control = 2;
```

The available modes are:

| Value | Configuration            |
| ----: | ------------------------ |
|  `-1` | PID + linear dynamics    |
|   `0` | PID + nonlinear dynamics |
|   `1` | LQR + linear dynamics    |
|   `2` | LQR + nonlinear dynamics |
|   `3` | Nonlinear MPC            |

For example:

```matlab
control = 0;
```

runs the PID controller using the nonlinear quadrotor dynamics.

To evaluate the LQR controller on the nonlinear plant:

```matlab
control = 2;
```

To use nonlinear MPC:

```matlab
control = 3;
```

The different configurations allow the effect of model linearization and nonlinear dynamics on closed-loop performance to be investigated.

---

### 7. State Estimation

The state-estimation architecture is selected using:

```matlab
estimation = 0;
```

The available options are:

| Value | Configuration       |
| ----: | ------------------- |
|   `0` | No state estimation |
|   `1` | Kalman filter       |

When `estimation = 0`, the controller receives the simulated state directly.

When `estimation = 1`, simulated accelerometer and gyroscope measurements are generated with noise and processed by the Kalman filter before being provided to the control architecture.

For example:

```matlab
estimation = 1;
```

enables the Kalman-filter-based state estimator.

---

### 8. Discrete-Time Sampling

The controller and estimator sampling time is controlled by:

```matlab
sampleTime = 0.005;       % [s]
```

The integration step `params.dt` and the controller/estimator sampling time are independent.

This allows the simulator to represent a system where the continuous-time dynamics are integrated at a finer resolution than the controller and estimator update rate.

Setting:

```matlab
sampleTime = params.dt;
```

makes the controller and estimator update at every integration step.

---

### 9. Visualization

Two flags control the graphical output:

```matlab
plot_flag = 1;
anim_flag = 1;
```

Set:

```matlab
plot_flag = 1;
```

to generate the simulation plots or:

```matlab
plot_flag = 0;
```

to disable plotting.

The animation can similarly be enabled or disabled using:

```matlab
anim_flag = 1;
```

or:

```matlab
anim_flag = 0;
```

When enabled, the simulator generates 2D views of the XY, YZ and XZ planes as well as a 3D visualization of the quadrotor trajectory.

---

### 10. PID Gain Tuning

When using PID control, the controller gains can be manually specified through the `kP`, `kI` and `kD` vectors.

The six controlled quantities correspond to:

```text
1. Altitude
2. Desired roll
3. Desired pitch
4. Roll
5. Pitch
6. Yaw
```

For example:

```matlab
kP = [20;
      -0.4;
       0.4;
       1;
       1;
       1.2];

kI = [10;
      -0.09;
       0.09;
       0.13;
       0.13;
       0.03];

kD = [10;
      -0.25;
       0.25;
       0.66;
       0.66;
       0.9];
```

These values are passed to `gainBuilder()` and then used by the PID controllers.

---

### 11. Automatic PID Tuning

The simulator also provides two automated PID tuning methods through the `tuner_flag`.

```matlab
tuner_flag = 0;
```

The available options are:

| Value | Tuning method       |
| ----: | ------------------- |
|   `0` | No automatic tuning |
|   `1` | Gradient Descent    |
|   `2` | Genetic Algorithm   |

#### Gradient Descent

```matlab
tuner_flag = 1;
```

The current implementation uses:

```matlab
maxIter = 10000;
tol = 1e-6;
alpha = 10;
```

The initial PID gains are randomly generated and then optimized using the PID tuning routine.

The resulting gains are saved to:

```text
tunedGains_gradient.mat
```

#### Genetic Algorithm

```matlab
tuner_flag = 2;
```

The current implementation uses:

```matlab
pop_size = 100;
maxGen = 2000;
tol = 1;
mutation_rate = 0.6;
kMax = 10;
```

The optimized gains are saved to:

```text
tunedGains_ga.mat
```

When automatic tuning is enabled, plotting is automatically disabled during the optimization process to avoid generating unnecessary graphical output.

---

### 12. Nonlinear MPC Parameters

When:

```matlab
control = 3;
```

the nonlinear MPC controller is used.

The main MPC parameters are defined inside the `MPC()` method in `Drone3.m`.

The prediction horizon is:

```matlab
N = 25;
```

and the MPC prediction time step is:

```matlab
dT = 0.1;
```

The control inputs are constrained by:

```text
0 ≤ u₁ ≤ 100
-100 ≤ u₂ ≤ 100
-100 ≤ u₃ ≤ 100
-100 ≤ u₄ ≤ 100
```

The MPC cost function uses weighting matrices for state-tracking error and control effort:

```matlab
Q = eye(6)*10;
Q(3,3) = 100;
Q(6,6) = 50;

R = eye(4)*5;
```

The optimization problem is solved using MATLAB's `fmincon` with the interior-point algorithm.

---

### 13. Running a Basic Simulation

A simple way to run the simulator is to use the default configuration:

```matlab
control = 2;       % LQR + nonlinear dynamics
estimation = 0;    % No state estimation

plot_flag = 1;
anim_flag = 1;

tuner_flag = 0;
```

Then run:

```matlab
simulator
```

The simulator will:

1. Initialize the vehicle parameters.
2. Define the initial and desired states.
3. Select the controller and state-estimation architecture.
4. Initialize the `Drone3` object.
5. Propagate the quadrotor dynamics.
6. Compute the control inputs at the selected sampling rate.
7. Log the system trajectory and control inputs.
8. Generate the requested plots and animation.

---

### 14. Experimenting with Different Configurations

The simulator is designed to make controller and model comparisons straightforward.

For example, the following configurations can be compared:

```matlab
% PID + nonlinear dynamics
control = 0;

% LQR + linear dynamics
control = 1;

% LQR + nonlinear dynamics
control = 2;

% Nonlinear MPC
control = 3;
```

The effect of state estimation can then be investigated independently:

```matlab
% Perfect state information
estimation = 0;

% Kalman-filter-based estimation
estimation = 1;
```

This makes it possible to perform controlled comparisons between different guidance, control and navigation configurations without modifying the underlying quadrotor model.

---

## Quadrotor State Representation

The quadrotor is represented using a 12-state vector:

$$
\mathbf{x} =
\begin{bmatrix}
x & y & z &
v_x & v_y & v_z &
\phi & \theta & \psi &
p & q & r
\end{bmatrix}^T
$$

where:

* $x,y,z$ — inertial position
* $v_x,v_y,v_z$ — inertial velocity
* $\phi,\theta,\psi$ — roll, pitch and yaw angles
* $p,q,r$ — body-frame angular velocities

The model describes the coupled translational and rotational motion of the vehicle.

---

## Nonlinear Dynamics

The simulator implements a nonlinear 6-DOF rigid-body model of the quadrotor.

The dynamics account for:

* Translational motion
* Rotational motion
* Gravity
* Rotor-generated thrust
* Control torques
* Attitude kinematics
* Coupling between translational and rotational dynamics

The rotor inputs are converted into the corresponding total thrust and body moments acting on the vehicle.

The resulting nonlinear equations of motion are numerically integrated during the simulation to obtain the time evolution of the vehicle state.

---

## Linearization

To enable linear control design, the nonlinear model is linearized around a **hover equilibrium condition**.

The resulting model is expressed in state-space form:

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

where:

* $A$ is the state matrix
* $B$ is the input matrix
* $\mathbf{x}$ is the vehicle state
* $\mathbf{u}$ represents the control inputs

The linearized model provides the basis for system analysis and LQR controller design.

---

## System Analysis

The simulator includes analysis of the linearized system before controller design.

### Stability Analysis

The eigenvalues of the state matrix are evaluated to investigate the stability characteristics of the linearized system.

### Controllability Analysis

The controllability matrix is computed to verify whether the system states can be controlled through the available inputs.

These analyses provide the theoretical basis for the subsequent controller design.

---

# Control Architectures

Several control strategies are implemented to evaluate different approaches to quadrotor flight control.

The simulator allows controllers to be tested using both the **linearized** and **nonlinear** plant models where applicable.

---

## PID Control

A PID-based controller is implemented for the main flight-control variables, including:

* Altitude
* Roll
* Pitch
* Yaw

The controller provides a relatively simple baseline for evaluating closed-loop flight performance.

The implementation includes numerical integration and differentiation of the tracking error for the integral and derivative terms.

---

## Linear Quadratic Regulator

An LQR controller is designed using the linearized hover model.

The controller minimizes a quadratic cost function:

$$
J =
\int_0^\infty
\left(
\mathbf{x}^T Q \mathbf{x}
+
\mathbf{u}^T R \mathbf{u}
\right)dt
$$

where:

* $Q$ weights deviations of the system states
* $R$ penalizes control effort

The weighting matrices can be modified to investigate the trade-off between tracking performance and control effort.

The LQR controller can be evaluated both on the linearized model and on the nonlinear quadrotor dynamics.

---

## Nonlinear Model Predictive Control

A nonlinear Model Predictive Controller is also implemented.

At each control step, the controller predicts the future evolution of the nonlinear quadrotor over a finite prediction horizon and solves an optimization problem to determine the control inputs.

The implemented formulation accounts for:

* Position tracking error
* Attitude tracking error
* Control effort

The prediction horizon used in the implementation is:

$$
N = 25
$$

This provides a framework for comparing nonlinear predictive control with conventional PID and LQR approaches.

---

# State Estimation

The simulator includes a **Kalman-filter-based state-estimation framework**.

The estimator uses simulated measurements from:

* Accelerometers
* Gyroscopes

to estimate the vehicle state in the presence of measurement noise.

The estimation process follows a prediction/correction structure:

### Prediction

The current state estimate is propagated using the system dynamics.

### Correction

The predicted state is corrected using the available sensor measurements.

The implementation uses the relevant system and measurement Jacobians required by the nonlinear estimation framework.

The estimated state can then be used by the controller, allowing the effect of imperfect state information on closed-loop performance to be investigated.

---

# Controller Tuning

The simulator includes automated controller-tuning capabilities.

Two optimization approaches are available:

### Gradient Descent

Gradient-based optimization can be used to iteratively adjust controller parameters according to the selected performance objective.

### Genetic Algorithm

A genetic-algorithm-based approach is also available for controller parameter optimization.

These methods provide an alternative to manually selecting controller gains.

---

# Simulation Configurations

The simulation framework allows different combinations of plant models, controllers and state-estimation configurations to be evaluated.

### Plant Model

```text
Plant Model
├── Linear dynamics
└── Nonlinear dynamics
```

### Controller

```text
Controller
├── PID
├── LQR
└── Nonlinear MPC
```

### State Estimation

```text
State Estimation
├── No estimation
└── Kalman filter
```

The simulator can therefore be used to investigate questions such as:

* How does a controller designed from a linearized model perform on the nonlinear plant?
* How do nonlinearities affect closed-loop trajectory tracking?
* How does state estimation influence controller performance?
* How do PID, LQR and nonlinear MPC compare?
* How does controller tuning affect the resulting response?

---

# Visualization

The simulator provides visualization tools for analyzing the resulting flight behavior.

The available outputs include:

* Position and trajectory evolution
* Vehicle attitude
* Control inputs
* Estimated trajectory
* 2D trajectory visualization
* 3D trajectory visualization
* Flight animation

These tools allow the closed-loop response of the vehicle to be inspected and different control architectures to be compared.

---

# Other Repository Components

## `arduino_sketches/`

The `arduino_sketches/` directory contains the **embedded software** developed for the physical quadrotor.

It includes the main Arduino code together with supporting functions, libraries and modules required by the onboard system.

This part of the project connects the mathematical and simulation environment with the constraints of a real embedded implementation.

---

## `electronics/`

The `electronics/` directory contains the hardware design associated with the quadrotor.

It includes:

* Electrical schematics
* PCB designs

This section documents the electronic architecture used to support the physical implementation of the vehicle.

---

## `docs/`

The `docs/` directory contains the technical documentation developed throughout the project together with the datasheets of the components used in the system.

The documentation provides the theoretical and engineering background behind the design and implementation choices made throughout the project.

---

## `testing/`

The `testing/` directory contains material related to the **testing and validation of the physical system**.

Testing provides the connection between the simulated models and the real-world implementation, allowing the behavior of the developed hardware and software to be evaluated.

---

## `WebApp/drone-web-app/`

The `WebApp/drone-web-app/` directory contains the **web application developed as an IoT interface for the drone**.

The application provides a higher-level interface through which the user can interact with the quadrotor system.

---

# Project Workflow

The overall development process can be summarized as:

```text
                 THEORETICAL MODEL
                        │
                        ▼
               Nonlinear Dynamics
                        │
                        ▼
                  Linearization
                        │
              ┌─────────┴─────────┐
              ▼                   ▼
       System Analysis      Controller Design
       • Stability          • PID
       • Controllability    • LQR
                            • NMPC
              │                   │
              └─────────┬─────────┘
                        ▼
                 State Estimation
                 • Kalman Filter
                        │
                        ▼
                  MATLAB/Simulink
                     Simulation
                        │
                        ▼
               Embedded Implementation
                        │
              ┌─────────┴─────────┐
              ▼                   ▼
          Electronics           Arduino
              │                   │
              └─────────┬─────────┘
                        ▼
                  Physical Drone
                        │
                        ▼
                     Testing
                        │
                        ▼
                 IoT Web Interface
```

The workflow connects the different components of the project, from mathematical modeling and simulation to embedded implementation, hardware testing and user interaction.

---

# Technologies

### Simulation & Control

* MATLAB
* Simulink
* Nonlinear dynamics
* PID
* LQR
* Nonlinear MPC
* Kalman filtering
* Numerical optimization
* Monte Carlo methods

### Embedded Systems

* Arduino
* Embedded C/C++
* Sensors

### Electronics

* Electrical schematics
* PCB design

### Software & IoT

* Web application
* IoT-based drone control

---

# Project Goals

The main goals of the project are to:

1. Develop a mathematical model of a quadrotor.
2. Analyze its nonlinear 6-DOF flight dynamics.
3. Linearize the system around hover and analyze its properties.
4. Develop and compare different control architectures.
5. Investigate state estimation and sensor fusion.
6. Optimize controller parameters.
7. Implement the developed concepts on embedded hardware.
8. Design and document the supporting electronics.
9. Test and validate the physical system.
10. Develop an IoT-based interface for interaction with the drone.

The project is intended as a complete engineering workflow connecting **aerospace dynamics, control, simulation, estimation, embedded systems, electronics and hardware implementation**.

---

# Author

**Matteo Colussi**

M.S. Aerospace Engineering
Georgia Institute of Technology

B.S. Aerospace Engineering
Politecnico di Milano

---

# License

This project is intended for educational and research purposes.

