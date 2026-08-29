clear;
clc;
close all;

% SPACAR Path
addpath('spacar');
addpath('spacar\spalight-1.38');

% Shared helper functions
addpath('utils');

% Load parameters
params = parameters();

% Load reference
ref = reference(params.spec.driving_speed_nom_mps, params.spec.weeding_time_s, params.spec.return_time_s, 1e-4);

% Nominal plant sizing - Deliverable b
nominal_plant_sizing = nominal_plant_sizing(params, ref);

% Store nominal plant sizes in params structure under mech
params.mech.r_arm_m       = nominal_plant_sizing.r_arm_m;
params.mech.J_kgm2        = nominal_plant_sizing.J_kgm2;
params.mech.k_Nm_per_rad  = nominal_plant_sizing.k_Nm_per_rad;
params.mech.d_Nms_per_rad = nominal_plant_sizing.d_Nms_per_rad;

% Define Laplace variable s
s_var = tf('s');

% Nominal Continuous plant: Voltage to Mirror Angle
P_nom = nominal_plant_tf(params.mech, params.actuator, s_var);

% Controller design
controller_design = controller_design(params, nominal_plant_sizing, ref);

% Simulation output
spacar_sim_out = simulation(params, nominal_plant_sizing, controller_design);

% Discretization
discretized_system = discretisation(params, controller_design, spacar_sim_out);

% Simulink parameters
ts = 5e-4;
r_sensor = 0.052;

equivalent = second_order_params(spacar_sim_out.plant_voltage_to_sensor_reduced);
J_eq = equivalent.m_eq;   % s^2 coeff -- rotary inertia [kg*m^2]
d_eq = equivalent.d_eq;   % s^1 coeff -- rotary damping [N*m*s/rad]
k_eq = equivalent.k_eq;
