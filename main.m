clear; clc; close all 

% SPACAR Path 
spacarRoot = fullfile(fileparts(mfilename('fullpath')), 'spacar');
spacarLight = fullfile(spacarRoot, 'spalight-1.38');

for pathToAdd = {spacarRoot, spacarLight}
    if ~isfolder(pathToAdd{1})
        error('laserWeeding:missingSpacar', ...
            'Required SPACAR directory not found: %s', pathToAdd{1});
    end
    addpath(pathToAdd{1});
end

if isempty(which('spacarlight'))
    error('laserWeeding:missingSpacar', ...
        'spacarlight is not on the MATLAB path after adding %s.', spacarLight);
end

% Load parameters 
params = parameters(); 

% Load reference 
ref = reference(params.spec.driving_speed_nom_mps, params.spec.weeding_time_s, params.spec.return_time_s, 1e-4);

% Nominal plant sizing - Deliverable b 
nominal_sizing = nominal_plant_sizing(params, ref);

% Store nominal plant sizes in params structure under mech
params.mech.r_arm_m       = nominal_sizing.r_arm_m;
params.mech.J_kgm2        = nominal_sizing.J_kgm2;
params.mech.k_Nm_per_rad  = nominal_sizing.k_Nm_per_rad;
params.mech.d_Nms_per_rad = nominal_sizing.d_Nms_per_rad;

% Define Laplace variable s
s_var = tf('s');

% Nominal Continuous plant: Voltage to Mirror Angle 
P_nom = (params.mech.r_arm_m * params.actuator.Kf_N_per_A / params.actuator.R25_ohm) / ...
    (params.mech.J_kgm2 * s_var^2 + params.mech.d_Nms_per_rad * s_var + params.mech.k_Nm_per_rad);

% Controller design 
controller = controller_design(params, nominal_sizing, ref);

% Simulation output
spacar_sim_out = simulation(params, nominal_sizing, controller);

% Discretization 
discretized_system = discretisation(params, controller, spacar_sim_out);

% Simulink parameters 
ts = 5e-4;
r_sensor = 0.052;

[num_a, den_a] = tfdata(spacar_sim_out.plant_voltage_to_sensor_reduced, 'v');

if numel(den_a) < 3
    error('laserWeeding:plantOrder', ...
        ['Reduced plant has denominator order %d; at least a second-order model is ', ...
         'required to extract J_eq, d_eq and k_eq.'], numel(den_a) - 1);
end

if ~isfinite(num_a(end)) || num_a(end) == 0
    error('laserWeeding:plantGain', ...
        ['Reduced plant has a zero or non-finite DC numerator coefficient (%g); ', ...
         'the equivalent inertia, damping and stiffness cannot be derived.'], num_a(end));
end

J_eq = abs(den_a(end-2) / num_a(end));    % s^2 coeff -- rotary inertia [kg*m^2]
d_eq = abs(den_a(end-1) / num_a(end));    % s^1 coeff -- rotary damping [N*m*s/rad]
k_eq = abs(den_a(end)   / num_a(end));
