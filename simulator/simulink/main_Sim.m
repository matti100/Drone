%% Main Simulation Script
% Script used to manage all the parameters of the simulation
% All the below parameters can be changed in order to perform different
% actions and operations. 
% 
% Date:         5/07/2024
% Authors:      Colussi Matteo

clear
clc
close all

%% Number of simulations
sim_iter = 10;

%% Time interval
t0 = 0;                 % [s]
tf = 10;                % [s]

%% Initial Condition
% 1: zeros
% 2: random conditions 

% init_cond = 1;

init_cond = [0, 0, 0; 0, 0, 0];

ang_des = [10, 10, 10];

%% Tuning procedure
% Select the tuning mode:
% - 0   # --- NO TUNING ---
% - 1:  # --- Altitude tuner ---
%       # Tune the altitude PID controller based on the only T_cmd
%       # contribute
%       # Set R_cmd = 0; P_cmd = 0; Y_cmd = 0;
% - 2:  # --- Roll tuner ---
%       # Tune the roll PID controller based on the T_cmd and R_cmd
%       # contribute
%       # Set P_cmd = 0; Y_cmd = 0;
% - 3: ...

flag_tuner = 0;

%% Organize all parameters in a structure data type
% Init_params : ' Initialization parameters '
Init_params = struct();

Init_params.iter = sim_iter;
Init_params.time = [t0, tf];
Init_params.cond = init_cond;

%% Run scripts
run('Data.m');
% run('Control.m');

