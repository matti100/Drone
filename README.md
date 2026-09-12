# Nonlinear Quadrotor Flight Simulator

A MATLAB/Simulink-based quadrotor simulation framework for modeling nonlinear 6-DOF flight dynamics and evaluating different flight-control and state-estimation architectures.

The project was developed to study the interaction between **nonlinear aircraft dynamics, linearization, feedback control, state estimation and trajectory tracking** in a quadrotor drone.

---

## Overview

This project implements a **12-state nonlinear quadrotor simulator** capable of simulating translational and rotational motion and evaluating multiple control architectures.

The simulator includes:

* Nonlinear 6-DOF quadrotor dynamics
* Hover linearization
* Controllability and stability analysis
* PID control
* LQR control
* Nonlinear Model Predictive Control (NMPC)
* Kalman-filter-based state estimation
* Gradient-descent and genetic-algorithm-based controller tuning
* Closed-loop trajectory tracking
* Numerical simulation and trajectory visualization
* 3D flight animation

The simulator can be configured to compare controllers operating with either the **linearized** or **nonlinear** plant dynamics.

---

## State-Space Model

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
* $\phi,\theta,\psi$ — roll, pitch, and yaw angles
* $p,q,r$ — body-frame angular velocities

The simulator models both translational and rotational dynamics, including the relationship between rotor-generated forces/torques and the resulting vehicle motion.

---

## Nonlinear Dynamics

The nonlinear model describes the quadrotor's coupled translational and rotational dynamics.

The translational equations account for:

* Gravity
* Total rotor thrust
* Vehicle attitude
* Translational acceleration

The rotational dynamics include:

* Body-frame angular rates
* Moments of inertia
* Control torques
* Coupling between rotational axes

Attitude kinematics are modeled using the roll, pitch, and yaw representation implemented in the simulator.

The rotor inputs are converted into the corresponding total thrust and body torques through the quadrotor actuator model.

---

## Linearization

To enable linear control design, the nonlinear model is linearized around a **hovering equilibrium condition**.

The linearized system is represented in the standard state-space form:

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

where:

* $A$ is the state matrix
* $B$ is the input matrix
* $\mathbf{x}$ is the 12-state vector
* $\mathbf{u}$ represents the control inputs

The project includes analysis of the resulting linear model, including:

### Stability

The eigenvalues of the state matrix are evaluated to assess the stability properties of the linearized system.

### Controllability

The controllability matrix is computed to verify whether the linearized quadrotor model can be controlled through the available inputs.

---

## Control Architectures

The simulator supports several control strategies, allowing their performance to be compared under different plant and estimation configurations.

### PID Control

A PID-based architecture is implemented for quadrotor position and attitude control.

The controller uses feedback to regulate:

* Altitude
* Roll
* Pitch
* Yaw

The implementation includes numerical integration and differentiation of the tracking error for the integral and derivative components.

PID control can be evaluated both with the nonlinear and linearized plant models.

---

### Linear Quadratic Regulator (LQR)

An LQR controller is designed using the linearized hover model.

The controller minimizes a quadratic cost of the form:

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

* $Q$ weights the state deviations
* $R$ penalizes control effort

Different weighting matrices can be evaluated to study the trade-off between tracking performance and control effort.

The LQR controller can be tested against both the linearized and nonlinear quadrotor dynamics.

---

### Nonlinear Model Predictive Control

A nonlinear Model Predictive Controller is also implemented.

At each control step, the controller predicts the future system evolution over a finite horizon and solves an optimization problem to determine the control sequence.

The implementation uses a prediction horizon of:

$$
N = 25
$$

The optimization objective accounts for:

* Position tracking error
* Attitude tracking error
* Control effort

The nonlinear dynamics are used directly within the prediction and optimization process.

The optimization is performed using MATLAB's constrained nonlinear optimization capabilities.

---

## State Estimation

The project also includes a **Kalman filter** for state estimation.

The estimator uses simulated sensor measurements and accounts for measurement noise.

The implemented estimation architecture uses:

* Accelerometer measurements
* Gyroscope measurements

The Kalman filter follows the standard prediction/correction structure:

### Prediction

The state estimate is propagated using the system dynamics.

### Correction

The predicted state is corrected using the available sensor measurements.

The implementation also uses the relevant system and measurement Jacobians required by the nonlinear estimation framework.

This allows the effect of imperfect state information on closed-loop control performance to be investigated.

---

## Controller Tuning

The simulator includes automated tuning options for controller parameters.

Two optimization approaches are available:

### Gradient Descent

Gradient-based optimization can be used to iteratively adjust controller parameters according to the selected performance objective.

### Genetic Algorithm

A genetic-algorithm-based approach is also available for controller parameter optimization.

These tools allow controller performance to be evaluated without relying exclusively on manually selected gains.

---

## Simulation Configurations

The simulator can be configured to evaluate different combinations of:

### Plant dynamics

* Nonlinear dynamics
* Linearized dynamics

### Controller

* PID
* LQR
* Nonlinear MPC

### State estimation

* No estimation
* Kalman filter

This makes it possible to study questions such as:

* How does a controller designed from a linearized model perform on the nonlinear plant?
* How does state estimation affect trajectory tracking?
* How does nonlinear MPC compare with PID and LQR?
* How does controller tuning influence tracking performance?
* What are the effects of model nonlinearities on closed-loop behavior?

---

## Simulation Workflow

A typical simulation follows the workflow:

```text
Reference Trajectory
        │
        ▼
   State Error
        │
        ▼
 ┌─────────────────┐
 │    Controller   │
 │                 │
 │ PID / LQR / MPC │
 └─────────────────┘
        │
        ▼
 Control Inputs
        │
        ▼
 ┌─────────────────┐
 │ Quadrotor Model │
 │                 │
 │ Linear / Nonlinear
 └─────────────────┘
        │
        ▼
   System State
        │
        ├──────────────► Visualization
        │
        ▼
 ┌─────────────────┐
 │ State Estimator │
 │ Kalman Filter   │
 └─────────────────┘
        │
        ▼
  Estimated State
        │
        └──────────────► Controller
```

---

## Project Structure

The main MATLAB scripts/functions are organized around the simulation, dynamics, control, and estimation components.

A typical project structure is:

```text
Drone-Simulator/
│
├── main simulation script
├── Drone3.m
├── controller functions
├── dynamics functions
├── estimation functions
├── plotting / visualization functions
└── README.md
```

`Drone3.m` contains the main implementation of the nonlinear dynamics, linearization, LQR design, Kalman filtering, PID control, nonlinear MPC, and related analysis.

---

## Requirements

The project requires:

* MATLAB
* Optimization Toolbox for the nonlinear MPC optimization
* Simulink, if the Simulink components of the project are used

The simulator was developed and tested in MATLAB.

---

## Running the Simulator

1. Clone or download the repository.

2. Open MATLAB and set the repository folder as the working directory.

3. Open the main simulation script.

4. Select the desired controller and simulation configuration.

The simulator allows the user to select between different controller/plant configurations and enable or disable state estimation.

For example, the available control modes include:

```text
PID + nonlinear dynamics
LQR + linear dynamics
LQR + nonlinear dynamics
PID + linear dynamics
Nonlinear MPC
```

The Kalman-filter-based state estimator can be enabled separately.

---

## Results and Visualization

The simulator provides tools for analyzing the closed-loop response, including:

* 3D vehicle trajectory
* Position tracking
* Attitude evolution
* Control inputs
* Estimated trajectory
* Simulation animation

These outputs can be used to evaluate both the transient response and overall trajectory-tracking performance.

---

## Main Concepts Explored

This project combines several topics in aerospace guidance, navigation, and control:

**Flight Dynamics**

* 6-DOF rigid-body dynamics
* Translational and rotational motion
* Attitude kinematics
* Rotor force and torque modeling

**Control**

* PID control
* LQR
* Nonlinear MPC
* Controller tuning
* Closed-loop trajectory tracking

**Estimation**

* Kalman filtering
* Sensor noise
* State estimation
* Prediction and correction

**System Analysis**

* Nonlinear system modeling
* Linearization
* Eigenvalue analysis
* Controllability

---

## Future Development

Potential extensions of the simulator include:

* More advanced nonlinear attitude representations
* Improved aerodynamic modeling
* Disturbance and uncertainty modeling
* More realistic sensor and actuator models
* Comparison with additional nonlinear control techniques
* More advanced trajectory-generation methods
* Hardware-in-the-loop simulation

---

## Author

**Matteo Colussi**

M.S. Aerospace Engineering
Georgia Institute of Technology

B.S. Aerospace Engineering
Politecnico di Milano

---

## License

This project is intended for educational and research purposes.
