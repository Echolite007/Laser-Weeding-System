function simulation = simulation(p, nominal_sizing, controller)
% Run SPACAR simulation, add actuator dynamics, reduce the plant,
% and analyse controller stability margins.

%% Geometry
w_mm = 15;
t_mm = 0.2;
youngModulus_GPa = 200;
springLength_mm = 88;
projectedSpringLength_mm = springLength_mm*sind(45);

Ixx_kgm2 = 316583.69e-9;
Ixy_kgm2 = 13049.31e-9;
Ixz_kgm2 = -14606.73e-9;
Iyy_kgm2 = 360033.12e-9;
Iyz_kgm2 = -3013.47e-9;
Izz_kgm2 = 538452.59e-9;

%% Nodes
nodes_m = 1e-3 * [
     0                         w_mm*1.5625     0;
     projectedSpringLength_mm  w_mm*1.5625     projectedSpringLength_mm;
     0                         w_mm*10.9375    0;
     projectedSpringLength_mm  w_mm*10.9375    projectedSpringLength_mm;
     0                         w_mm*6.25       projectedSpringLength_mm;
     projectedSpringLength_mm  w_mm*6.25       0;
     projectedSpringLength_mm  w_mm*6.25       projectedSpringLength_mm;
     87.56                     93.66           projectedSpringLength_mm;
     49.12                     93.09           63.81;
     projectedSpringLength_mm/2 w_mm*6.25      projectedSpringLength_mm/2;
     22                        103.5           82.5];

%% Elements
elements = [
    1  2;
    3  4;
    5  6;
    7  5;
    2  7;
    4  7;
    8  7;
    9  7;
    10 7;
    5  11];

%% Node properties
clear nodeProperties

nodeProperties(1).fix = true;
nodeProperties(3).fix = true;
nodeProperties(6).fix = true;

nodeProperties(9).mass = 0.26104;
nodeProperties(9).mominertia = [
    Ixx_kgm2 Ixy_kgm2 Ixz_kgm2 Iyy_kgm2 Iyz_kgm2 Izz_kgm2];

nodeProperties(8).transfer_in  = 'force_z';
nodeProperties(8).transfer_out = 'veloc_z';
nodeProperties(11).transfer_out = 'displ_x';

actuatorVelocityOutputIndex = 1;
sensorDisplacementOutputIndex = 2;

%% Element properties
clear elementProperties

elementProperties(1).elems = [1 2];
elementProperties(1).emod = youngModulus_GPa*1e9;
elementProperties(1).smod = 79e9;
elementProperties(1).dens = 7800;
elementProperties(1).cshape = 'rect';
elementProperties(1).dim = [w_mm*3.125e-3 t_mm*1e-3];
elementProperties(1).orien = [0 1 0];
elementProperties(1).nbeams = 2;
elementProperties(1).flex = 1:6;
elementProperties(1).color = 'grey';
elementProperties(1).opacity = 0.7;
elementProperties(1).warping = true;

elementProperties(2).elems = 3;
elementProperties(2).emod = youngModulus_GPa*1e9;
elementProperties(2).smod = 79e9;
elementProperties(2).dens = 7800;
elementProperties(2).cshape = 'rect';
elementProperties(2).dim = [w_mm*6.25e-3 t_mm*1e-3];
elementProperties(2).orien = [0 1 0];
elementProperties(2).nbeams = 2;
elementProperties(2).flex = 1:6;
elementProperties(2).color = 'grey';
elementProperties(2).opacity = 0.7;
elementProperties(2).warping = true;

elementProperties(3).elems = 4;
elementProperties(3).cshape = 'rect';
elementProperties(3).dim = [w_mm*12.5e-3 0.6e-3];
elementProperties(3).orien = [0 1 0];
elementProperties(3).nbeams = 1;
elementProperties(3).color = 'darkblue';
elementProperties(3).warping = true;

elementProperties(4).elems = [5 6 7 8 9 10];
elementProperties(4).orien = [1 0 0];

%% Run SPACAR
spacarOptions.gravity = [0 -9.81 0];
spacarOptions.transfer = {true, p.ctrl.ts_s};
spacarOptions.customvis = {'modeshape','maximum 20'};

spacarOutput = spacarlight(nodes_m, elements, nodeProperties, elementProperties, spacarOptions);

% spacarlight returns a partially filled struct when an analysis step does not
% complete, so check what came back before using it.
if ~isstruct(spacarOutput) || ~isfield(spacarOutput, 'statespace') || isempty(spacarOutput.statespace)
    error('simulation:noStateSpace', ...
        ['SPACAR returned no state-space model. Check spacar_file.log and that ', ...
         'opt.transfer is enabled with valid transfer inputs/outputs.']);
end

if ~isfield(spacarOutput, 'step') || isempty(spacarOutput.step) || ...
        ~isfield(spacarOutput.step(end), 'freq') || isempty(spacarOutput.step(end).freq)
    error('simulation:noEigenFrequencies', ...
        'SPACAR returned no eigenfrequencies; the model reduction split frequency cannot be set.');
end

eigenFreq_Hz = spacarOutput.step(end).freq;
plant_force_to_outputs_mech = spacarOutput.statespace;

%% Mechanical Bode diagram: actuator force to sensor displacement
opts = bodeoptions;
opts.PhaseWrapping = 'off';
figure('Name','Deliverable g: Force to Sensor Displacement Frequency Response');
bodeplot(-plant_force_to_outputs_mech(sensorDisplacementOutputIndex,1));
title('Deliverable g - Mechanical plant: Force to Sensor Displacement Frequency Response');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Add actuator dynamics (Voltage to sensor displacement) 
coilInductance_H = p.actuator.Lcoil_H;
coilResistance_ohm = p.actuator.R25_ohm;
km = p.actuator.Kf_N_per_A;

actuator_current_per_voltage = tf(1,[coilInductance_H coilResistance_ohm]); % Maps current to voltage 

plant_voltage_to_outputs_noBackEmf = plant_force_to_outputs_mech * km * actuator_current_per_voltage; % Voltage to Sensor Displacement (No Back EMF) 

plant_voltage_to_outputs_withBackEmf = feedback(plant_voltage_to_outputs_noBackEmf, km, 1, actuatorVelocityOutputIndex); % Voltage to Sensor Displacement (Back EMF)

plant_voltage_to_sensor_em = plant_voltage_to_outputs_withBackEmf(sensorDisplacementOutputIndex,1); % Sign flip because a positive voltage gives a negative sensor displacement

plant_voltage_to_sensor_em = -tf(plant_voltage_to_sensor_em);

figure('Name','Deliverable h: Plant with/without actuator dynamics');
bodeplot(-tf(plant_force_to_outputs_mech(sensorDisplacementOutputIndex,1)), plant_voltage_to_sensor_em);
legend('Without actuator dynamics','With actuator dynamics');
title('Deliverable h - Voltage/Force -> sensor displacement');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Model reduction 
reductionSplitFreq_rad_s = eigenFreq_Hz(1)*40*2*pi;

[plant_voltage_to_sensor_reduced, ~] = freqsep(plant_voltage_to_sensor_em, reductionSplitFreq_rad_s); % Separate model in slow and fast dynamics

plant_voltage_to_sensor_reduced = tf(plant_voltage_to_sensor_reduced); % Transfer function of slow dynamics part only 

figure('Name','Deliverable h: Model reduction via freqsep');
bode(plant_voltage_to_sensor_em, plant_voltage_to_sensor_reduced);
legend('Full electromechanical plant','2nd-order approximation (freqsep)');
title('Deliverable h - Model reduction via freqsep');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Extract equivalent second-order parameters (m_eq, w_n, zeta) 
% Extract numerator and denominator coefficient vectors.
[reducedNumerator, reducedDenominator] = tfdata(plant_voltage_to_sensor_reduced, 'v'); 

% The second-order fit below indexes three denominator coefficients and divides
% by the numerator DC gain; both assumptions can silently produce Inf/NaN.
if numel(reducedDenominator) < 3
    error('simulation:reductionOrder', ...
        ['Model reduction returned a denominator of order %d; freqsep did not keep a ', ...
         'second-order slow subsystem (split frequency %.1f rad/s).'], ...
        numel(reducedDenominator) - 1, reductionSplitFreq_rad_s);
end

if ~isfinite(reducedNumerator(end)) || reducedNumerator(end) == 0
    error('simulation:reductionGain', ...
        'Reduced plant DC numerator coefficient is %g; m_eq cannot be derived.', ...
        reducedNumerator(end));
end

% Get parameters using standard second order transfer function form 
m_eq = abs(reducedDenominator(end-2)/reducedNumerator(end));
w_n = sqrt(reducedDenominator(end));
zeta = reducedDenominator(end-1)/(2*w_n);

% fprintf('\n=== SPACAR 2nd-order fit ===\n');
% fprintf('  wn   = %.2f rad/s (%.2f Hz)\n', ...
%     w_n, w_n/(2*pi));
% fprintf('  zeta = %.4f\n', zeta);
% fprintf('  m_eq = %.4g\n', m_eq);

%% Stability
s = tf('s');

%% Nominal plant (Voltage to Angle)
plant_voltage_to_angle_nominal = (nominal_sizing.r_arm_m*km/p.actuator.R25_ohm) / ...
    (nominal_sizing.J_kgm2*s^2 + nominal_sizing.d_Nms_per_rad*s + nominal_sizing.k_Nm_per_rad);

nominalCrossover_rad_s = controller.B.wc_rad_s;
nominalAlpha = controller.alpha;
beta = controller.beta;

controller_nominal = build_pid(nominal_sizing.J_kgm2 / ...
    (nominal_sizing.r_arm_m*km/p.actuator.R25_ohm), nominalCrossover_rad_s, nominalAlpha, beta, s);

openLoop_nominal = controller_nominal * plant_voltage_to_angle_nominal;


% phaseCrossFreq_nominal_rad_s can also be retrieved instead of ~
[gainMargin_nominal_abs, phaseMargin_nominal_deg, ~ , gainCrossFreq_nominal_rad_s] = ...
    margin(openLoop_nominal);

[gainMargin_nominal_abs, phaseMargin_nominal_deg, gainCrossFreq_nominal_rad_s] = ...
    checkMargins('nominal plant + nominal controller', ...
    gainMargin_nominal_abs, phaseMargin_nominal_deg, gainCrossFreq_nominal_rad_s);


%% SPACAR Model without actuator dynamics (force to sensor displacement)
plant_force_to_sensor_mech = -tf(plant_force_to_outputs_mech(sensorDisplacementOutputIndex,1));

openLoop_spacarMechanical = controller_nominal * plant_force_to_sensor_mech;


% phaseCrossFreq_spacarMechanical_rad_s can be retrieved instead of ~
[gainMargin_spacarMechanical_abs, phaseMargin_spacarMechanical_deg, ...
    ~, gainCrossFreq_spacarMechanical_rad_s] = ...
    margin(openLoop_spacarMechanical);

[gainMargin_spacarMechanical_abs, phaseMargin_spacarMechanical_deg, ...
    gainCrossFreq_spacarMechanical_rad_s] = ...
    checkMargins('SPACAR mechanical + nominal controller', ...
    gainMargin_spacarMechanical_abs, phaseMargin_spacarMechanical_deg, ...
    gainCrossFreq_spacarMechanical_rad_s);


%% SPACAR Model with actuator dynamics (Back EMF) (voltage to sensor displacement) 
openLoop_electromechanical_nominalController = ...
    controller_nominal * plant_voltage_to_sensor_em;

% phaseCrossFreq_electromechanical_rad_s can be retrieved instead of ~
[gainMargin_electromechanical_abs, phaseMargin_electromechanical_deg, ...
    ~, gainCrossFreq_electromechanical_rad_s] = ...
    margin(openLoop_electromechanical_nominalController);

[gainMargin_electromechanical_abs, phaseMargin_electromechanical_deg, ...
    gainCrossFreq_electromechanical_rad_s] = ...
    checkMargins('electromechanical + nominal controller', ...
    gainMargin_electromechanical_abs, phaseMargin_electromechanical_deg, ...
    gainCrossFreq_electromechanical_rad_s);

%% Retuned controller
targetCrossover_rad_s = p.ctrl.wc_rads;

[~, plantPhaseAtTarget_deg] = bode( ...
    plant_voltage_to_sensor_em, targetCrossover_rad_s);

plantPhaseAtTarget_deg = squeeze(plantPhaseAtTarget_deg);
requiredLeadPhase_deg = 45 - (180 + plantPhaseAtTarget_deg);

if requiredLeadPhase_deg <= 0
    retunedAlpha = controller.alpha;
else
    retunedAlpha = ...
        (1 - sind(requiredLeadPhase_deg)) / ...
        (1 + sind(requiredLeadPhase_deg));

    retunedAlpha = max(0.001, min(0.999, retunedAlpha));
end

controller_retunedPID = build_pid( ...
    m_eq, targetCrossover_rad_s, retunedAlpha, beta, s);


%% SPACAR Model with actuator dynamics (Back EMF) with retuned controller
openLoop_retuned = controller_retunedPID * plant_voltage_to_sensor_em;
closedLoop_retuned = feedback(openLoop_retuned, 1);

% phaseCrossFreq_retuned_rad_s instead of ~
[gainMargin_retuned_abs, phaseMargin_retuned_deg, ...
    ~, gainCrossFreq_retuned_rad_s] = ...
    margin(openLoop_retuned);

[gainMargin_retuned_abs, phaseMargin_retuned_deg, gainCrossFreq_retuned_rad_s] = ...
    checkMargins('electromechanical + retuned controller', ...
    gainMargin_retuned_abs, phaseMargin_retuned_deg, gainCrossFreq_retuned_rad_s);

gainMargin_retuned_dB = 20*log10(gainMargin_retuned_abs);

%% Check for stability using poles 
isClosedLoopStable_retuned = all(real(pole(closedLoop_retuned)) < 0);

if isClosedLoopStable_retuned
    fprintf('  Closed-loop Stable\n');
else
    warning('simulation:closedLoopUnstable', ...
        ['Retuned closed loop is unstable: %d of %d poles have non-negative real ', ...
         'part. Downstream discretisation results are not valid.'], ...
        sum(real(pole(closedLoop_retuned)) >= 0), numel(pole(closedLoop_retuned)));
end

%% Summary table
fprintf('\n--- Summary table ---\n');
fprintf('%-55s  %8s  %8s  %8s\n', ...
    'Configuration','wc(rad/s)','PM(deg)','GM(dB)');

fprintf('%-55s  %8.1f  %8.1f  %8.1f\n', ...
    'Nominal plant + nominal controller', ...
    gainCrossFreq_nominal_rad_s, ...
    phaseMargin_nominal_deg, ...
    20*log10(gainMargin_nominal_abs));

fprintf('%-55s  %8.1f  %8.1f  %8.1f\n', ...
    'SPACAR mechanical + nominal controller', ...
    gainCrossFreq_spacarMechanical_rad_s, ...
    phaseMargin_spacarMechanical_deg, ...
    20*log10(gainMargin_spacarMechanical_abs));

fprintf('%-55s  %8.1f  %8.1f  %8.1f\n', ...
    'Electromechanical + nominal controller', ...
    gainCrossFreq_electromechanical_rad_s, ...
    phaseMargin_electromechanical_deg, ...
    20*log10(gainMargin_electromechanical_abs));

fprintf('%-55s  %8.1f  %8.1f  %8.1f\n', ...
    'Electromechanical + retuned controller', ...
    gainCrossFreq_retuned_rad_s, ...
    phaseMargin_retuned_deg, ...
    gainMargin_retuned_dB);

%% Open-loop comparison plot
figure('Name','Deliverable i: Open-loop Bode - before and after');
bode( ...
    openLoop_nominal, ...
    openLoop_spacarMechanical, ...
    openLoop_electromechanical_nominalController, ...
    openLoop_retuned);

legend( ...
    'Nominal plant + C_{nom}', ...
    'SPACAR mechanical + C_{nom}', ...
    'Electromechanical + C_{nom}', ...
    'Electromechanical + C_{retuned}', ...
    'Location','southwest');

title('Deliverable i - Effect of parasitic and actuator dynamics on open-loop margins');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Retuned open-loop margin plot
figure('Name','Deliverable i: Retuned open-loop Bode with margins');
margin(openLoop_retuned);
title('Deliverable i - Retuned open-loop: C_{PID} P_{em}');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Print retuned PID parameters
fprintf('\n=== Retuned PID parameters ===\n');
fprintf('  alpha  = %.4f\n', retunedAlpha);
fprintf('  beta   = %.1f\n', beta);

retunedtau_z = sqrt(1/retunedAlpha)/targetCrossover_rad_s;
retunedtau_i = beta*retunedtau_z;
retunedPoleTimeConstant_s = ...
    1/(targetCrossover_rad_s*sqrt(1/retunedAlpha));

retunedProportionalGain = ...
    m_eq*targetCrossover_rad_s^2/sqrt(1/retunedAlpha);

fprintf('  kp     = %.4g\n', retunedProportionalGain);
fprintf('  tau_z  = %.4g s  (zero at %.1f rad/s)\n', ...
    retunedtau_z, 1/retunedtau_z);
fprintf('  tau_i  = %.4g s  (zero at %.1f rad/s)\n', ...
    retunedtau_i, 1/retunedtau_i);
fprintf('  tau_p  = %.4g s  (pole at %.1f rad/s)\n', ...
    retunedPoleTimeConstant_s, 1/retunedPoleTimeConstant_s);

%% Pack outputs
simulation.eigenFreq_Hz = eigenFreq_Hz;

simulation.plant_force_to_outputs_mech = plant_force_to_outputs_mech;
simulation.plant_force_to_sensor_mech = plant_force_to_sensor_mech;
simulation.plant_voltage_to_sensor_em = plant_voltage_to_sensor_em;
simulation.plant_voltage_to_sensor_reduced = plant_voltage_to_sensor_reduced;

simulation.m_eq = m_eq;
simulation.wn_rad_s = w_n;
simulation.zeta = zeta;

simulation.controller_nominal = controller_nominal;
simulation.controller_retunedPID = controller_retunedPID;

simulation.openLoop_nominal = openLoop_nominal;
simulation.openLoop_spacarMechanical = openLoop_spacarMechanical;
simulation.openLoop_electromechanical_nominalController = ...
    openLoop_electromechanical_nominalController;
simulation.openLoop_retuned = openLoop_retuned;
simulation.closedLoop_retuned = closedLoop_retuned;

simulation.wc_rad_s = gainCrossFreq_retuned_rad_s;
simulation.wc_target = targetCrossover_rad_s;
simulation.PM_deg = phaseMargin_retuned_deg;
simulation.GM_dB = gainMargin_retuned_dB;
simulation.alpha = retunedAlpha;
simulation.beta = beta;

simulation.PM_nom = phaseMargin_nominal_deg;
simulation.PM_mech = phaseMargin_spacarMechanical_deg;
simulation.PM_em = phaseMargin_electromechanical_deg;

simulation.GM_nom_dB = 20*log10(gainMargin_nominal_abs);
simulation.GM_mech_dB = 20*log10(gainMargin_spacarMechanical_abs);
simulation.GM_em_dB = 20*log10(gainMargin_electromechanical_abs);
simulation.isClosedLoopStable_retuned = isClosedLoopStable_retuned;

end

function [gainMargin_abs, phaseMargin_deg, gainCrossFreq_rad_s] = ...
    checkMargins(label, gainMargin_abs, phaseMargin_deg, gainCrossFreq_rad_s)
% margin() returns empty or NaN when no crossover exists. Without this check
% those values print as NaN in the summary table and are packed into the
% output struct as if they were valid margins.
if isempty(gainMargin_abs);     gainMargin_abs = NaN;     end
if isempty(phaseMargin_deg);    phaseMargin_deg = NaN;    end
if isempty(gainCrossFreq_rad_s); gainCrossFreq_rad_s = NaN; end

if ~isfinite(phaseMargin_deg) || ~isfinite(gainCrossFreq_rad_s)
    warning('simulation:noCrossover', ...
        ['No valid gain crossover found for %s (PM = %g deg, wc = %g rad/s); ', ...
         'the reported margins for this configuration are meaningless.'], ...
        label, phaseMargin_deg, gainCrossFreq_rad_s);
elseif phaseMargin_deg <= 0
    warning('simulation:nonPositivePhaseMargin', ...
        'Phase margin for %s is %.2f deg, i.e. the open loop is not stable.', ...
        label, phaseMargin_deg);
end
end

function controller_tf = build_pid(m_eq, wc_rad_s, alpha, beta, s)
% PID controller with lead filter:
% C(s) = kp*(tau_z*s + 1)*(tau_i*s + 1) / ((tau_p*s + 1)*tau_i*s)

tau_z = sqrt(1/alpha)/wc_rad_s;
tau_i = beta*tau_z;
poleTimeConstant_s = 1/(wc_rad_s*sqrt(1/alpha));

proportionalGain = m_eq*wc_rad_s^2/sqrt(1/alpha);

controller_tf = proportionalGain * ...
    (tau_z*s + 1) * ...
    (tau_i*s + 1) / ...
    ((poleTimeConstant_s*s + 1)*tau_i*s);

end