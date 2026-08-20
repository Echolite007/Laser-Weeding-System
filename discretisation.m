function discretized_system = discretisation(p, controller_design, sim_out)

requiredSimFields = {'wc_target', 'plant_voltage_to_sensor_reduced', ...
    'controller_retunedPID', 'm_eq', 'alpha'};
missing = requiredSimFields(~isfield(sim_out, requiredSimFields));
if ~isempty(missing)
    error('discretisation:missingSimulationOutput', ...
        'Simulation output is missing required field(s): %s.', strjoin(missing, ', '));
end

%% Setup
s = tf('s');

sampleTime_s = p.ctrl.ts_s;
samplingFrequency_Hz = 1/sampleTime_s;
nyquistFrequency_Hz = samplingFrequency_Hz/2;
nyquistFrequency_rad_s = pi/sampleTime_s;

targetCrossover_rad_s = sim_out.wc_target;

plant_voltage_to_sensor_reduced = sim_out.plant_voltage_to_sensor_reduced;
controller_continuous_retunedPID = sim_out.controller_retunedPID;

openLoop_continuous = controller_continuous_retunedPID * plant_voltage_to_sensor_reduced;

m_eq = sim_out.m_eq;
beta = controller_design.beta;

%% Step j.1: Discretise plant with ZOH and controller with Tustin
plant_discrete_zoh = c2d(plant_voltage_to_sensor_reduced, sampleTime_s, 'zoh');

controller_discrete_tustin = c2d( ...
    controller_continuous_retunedPID, ...
    sampleTime_s, ...
    'tustin');

openLoop_discrete_original = controller_discrete_tustin * plant_discrete_zoh;
closedLoop_discrete_original = feedback(openLoop_discrete_original, 1);

[gainMargin_cont_abs, phaseMargin_cont_deg, ...
    phaseCrossFreq_cont_rad_s, gainCrossFreq_cont_rad_s] = ...
    margin(openLoop_continuous);

[gainMargin_disc_abs, phaseMargin_disc_deg, ...
    phaseCrossFreq_disc_rad_s, gainCrossFreq_disc_rad_s] = ...
    margin(openLoop_discrete_original);

[gainMargin_cont_abs, phaseMargin_cont_deg, gainCrossFreq_cont_rad_s] = ...
    checkMargins('continuous open loop', ...
    gainMargin_cont_abs, phaseMargin_cont_deg, gainCrossFreq_cont_rad_s);

[gainMargin_disc_abs, phaseMargin_disc_deg, gainCrossFreq_disc_rad_s] = ...
    checkMargins('discrete open loop, original alpha', ...
    gainMargin_disc_abs, phaseMargin_disc_deg, gainCrossFreq_disc_rad_s);

phaseMarginLoss_deg = phaseMargin_cont_deg - phaseMargin_disc_deg;
crossoverShift_percent = ...
    (gainCrossFreq_disc_rad_s - gainCrossFreq_cont_rad_s)/gainCrossFreq_cont_rad_s*100;

numUnstablePoles_original = sum(abs(pole(closedLoop_discrete_original)) > 1);
isClosedLoopStable_original = (numUnstablePoles_original == 0);

if ~isClosedLoopStable_original
    warning('discretisation:closedLoopUnstable', ...
        ['Discrete closed loop with the original alpha has %d pole(s) outside the unit ', ...
         'circle.'], numUnstablePoles_original);
end

%% Step j.2: Quantify phase-lag source
zohHalfSampleLag_deg = sampleTime_s * gainCrossFreq_cont_rad_s / 2 * (180/pi);

tustinWarpedFreq_rad_s = ...
    2/sampleTime_s * tan(gainCrossFreq_cont_rad_s*sampleTime_s/2);

tustinWarp_percent = ...
    (tustinWarpedFreq_rad_s - gainCrossFreq_cont_rad_s)/gainCrossFreq_cont_rad_s*100;

%% Step j.3: Retune controller for discrete implementation
desiredDiscretePhaseMargin_deg = 30;

targetContinuousPhaseMargin_deg = ...
    desiredDiscretePhaseMargin_deg + phaseMarginLoss_deg;

currentLeadPhase_deg = asind( ...
    (1 - sim_out.alpha) / ...
    (1 + sim_out.alpha));

extraLeadRequired_deg = targetContinuousPhaseMargin_deg - phaseMargin_cont_deg;

if extraLeadRequired_deg <= 0
    retunedAlpha = sim_out.alpha;
else
    newLeadPhase_deg = min(currentLeadPhase_deg + extraLeadRequired_deg, 78);

    retunedAlpha = ...
        (1 - sind(newLeadPhase_deg)) / ...
        (1 + sind(newLeadPhase_deg));

    retunedAlpha = max(0.001, min(0.999, retunedAlpha));
end

retunedZeroTimeConstant_s = sqrt(1/retunedAlpha)/targetCrossover_rad_s;
retunedIntegralTimeConstant_s = beta*retunedZeroTimeConstant_s;
retunedPoleTimeConstant_s = ...
    1/(targetCrossover_rad_s*sqrt(1/retunedAlpha));

retunedProportionalGain = ...
    m_eq*targetCrossover_rad_s^2/sqrt(1/retunedAlpha);

controller_continuous_retunedForDiscretisation = ...
    retunedProportionalGain * ...
    (retunedZeroTimeConstant_s*s + 1) * ...
    (retunedIntegralTimeConstant_s*s + 1) / ...
    ((retunedPoleTimeConstant_s*s + 1)*retunedIntegralTimeConstant_s*s);

openLoop_continuous_retunedForDiscretisation = ...
    controller_continuous_retunedForDiscretisation * plant_voltage_to_sensor_reduced;

controller_discrete_retunedForDiscretisation = c2d( ...
    controller_continuous_retunedForDiscretisation, ...
    sampleTime_s, ...
    'tustin');

openLoop_discrete_retuned = ...
    controller_discrete_retunedForDiscretisation * plant_discrete_zoh;

closedLoop_discrete_retuned = feedback(openLoop_discrete_retuned, 1);

[gainMargin_contRetuned_abs, phaseMargin_contRetuned_deg, ...
    ~, gainCrossFreq_contRetuned_rad_s] = ...
    margin(openLoop_continuous_retunedForDiscretisation);

[gainMargin_discRetuned_abs, phaseMargin_discRetuned_deg, ...
    phaseCrossFreq_discRetuned_rad_s, gainCrossFreq_discRetuned_rad_s] = ...
    margin(openLoop_discrete_retuned);

[gainMargin_contRetuned_abs, phaseMargin_contRetuned_deg, gainCrossFreq_contRetuned_rad_s] = ...
    checkMargins('continuous open loop, retuned for discretisation', ...
    gainMargin_contRetuned_abs, phaseMargin_contRetuned_deg, gainCrossFreq_contRetuned_rad_s);

[gainMargin_discRetuned_abs, phaseMargin_discRetuned_deg, gainCrossFreq_discRetuned_rad_s] = ...
    checkMargins('discrete open loop, retuned alpha', ...
    gainMargin_discRetuned_abs, phaseMargin_discRetuned_deg, gainCrossFreq_discRetuned_rad_s);

numUnstablePoles_retuned = sum(abs(pole(closedLoop_discrete_retuned)) > 1);
isClosedLoopStable_retuned = (numUnstablePoles_retuned == 0);

if ~isClosedLoopStable_retuned
    warning('discretisation:closedLoopUnstable', ...
        ['Discrete closed loop with the retuned alpha has %d pole(s) outside the unit ', ...
         'circle; the retuned controller is not implementable at ts = %.4g s.'], ...
        numUnstablePoles_retuned, sampleTime_s);
end

if isfinite(gainCrossFreq_discRetuned_rad_s) && ...
        gainCrossFreq_discRetuned_rad_s > 0.3*nyquistFrequency_rad_s
    warning('discretisation:crossoverNearNyquist', ...
        ['Discrete crossover %.1f rad/s is above 30%% of the Nyquist frequency ', ...
         '%.1f rad/s; the sampled-data margins are unreliable.'], ...
        gainCrossFreq_discRetuned_rad_s, nyquistFrequency_rad_s);
end

%% Summary table
fprintf('\n--- Deliverable j summary table ---\n');
fprintf('%-48s  %8s  %8s  %8s\n', ...
    'Configuration','wc(rad/s)','PM(deg)','GM(dB)');

fprintf('%-48s  %8.1f  %8.1f  %8.1f\n', ...
    'Continuous, original alpha', ...
    gainCrossFreq_cont_rad_s, ...
    phaseMargin_cont_deg, ...
    20*log10(gainMargin_cont_abs));

fprintf('%-48s  %8.1f  %8.1f  %8.1f\n', ...
    'Discrete Tustin + ZOH, original alpha', ...
    gainCrossFreq_disc_rad_s, ...
    phaseMargin_disc_deg, ...
    20*log10(gainMargin_disc_abs));

fprintf('%-48s  %8.1f  %8.1f  %8.1f\n', ...
    'Discrete Tustin + ZOH, retuned alpha', ...
    gainCrossFreq_discRetuned_rad_s, ...
    phaseMargin_discRetuned_deg, ...
    20*log10(gainMargin_discRetuned_abs));

%% Figure j.1: Continuous vs discrete open-loop Bode

opts = bodeoptions;
opts.PhaseWrapping = 'off';
% opts.PhaseWrappingBranch = -180;
opts.XLim = [1 nyquistFrequency_rad_s];

figure('Name','Deliverable j: Continuous vs Discrete OL Bode');
bode( ...
    openLoop_continuous, ...
    openLoop_discrete_original, ...
    openLoop_discrete_retuned, opts);

legend( ...
    'Continuous C P_{em}', ...
    'Discrete, Tustin C + ZOH P, original alpha', ...
    'Discrete, Tustin C_{retuned} + ZOH P', ...
    'Location','southwest');

title('Deliverable j - Effect of discretisation on open-loop frequency response');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Figure j.2: Discrete open-loop margin plot, retuned
figure('Name','Deliverable j: Discrete OL margin, retuned');
margin(openLoop_discrete_retuned);
title('Deliverable j - Discrete open-loop C_z(retuned) P_z(ZOH)');
grid on;
set(findall(gcf,'Type','line'),'LineWidth',1.3);

%% Figure j.3: Discrete closed-loop pole-zero map
figure('Name','Deliverable j: Discrete CL pole-zero map');
pzmap(closedLoop_discrete_original, closedLoop_discrete_retuned);
zgrid;

legend( ...
    'Original alpha', ...
    'Retuned alpha', ...
    'Location','northwest');

title('Deliverable j - Discrete closed-loop poles, unit circle is stability boundary');

%% Pack outputs
discretized_system.sampleTime_s = sampleTime_s;
discretized_system.samplingFrequency_Hz = samplingFrequency_Hz;
discretized_system.nyquistFrequency_Hz = nyquistFrequency_Hz;
discretized_system.nyquistFrequency_rad_s = nyquistFrequency_rad_s;
discretized_system.targetCrossover_rad_s = targetCrossover_rad_s;
discretized_system.crossoverToNyquistRatio = targetCrossover_rad_s/nyquistFrequency_rad_s;

discretized_system.plant_discrete_zoh = plant_discrete_zoh;
discretized_system.controller_discrete_tustin = controller_discrete_tustin;

discretized_system.openLoop_continuous = openLoop_continuous;
discretized_system.openLoop_discrete_original = openLoop_discrete_original;
discretized_system.closedLoop_discrete_original = closedLoop_discrete_original;

discretized_system.alpha_original = sim_out.alpha;
discretized_system.alpha_retuned = retunedAlpha;

discretized_system.controller_continuous_retunedForDiscretisation = ...
    controller_continuous_retunedForDiscretisation;

discretized_system.controller_discrete_retunedForDiscretisation = ...
    controller_discrete_retunedForDiscretisation;

discretized_system.openLoop_continuous_retunedForDiscretisation = ...
    openLoop_continuous_retunedForDiscretisation;

discretized_system.openLoop_discrete_retuned = openLoop_discrete_retuned;
discretized_system.closedLoop_discrete_retuned = closedLoop_discrete_retuned;

discretized_system.phaseMargin_continuous_deg = phaseMargin_cont_deg;
discretized_system.phaseMargin_discrete_original_deg = phaseMargin_disc_deg;
discretized_system.phaseMargin_discrete_retuned_deg = phaseMargin_discRetuned_deg;
discretized_system.phaseMarginLoss_deg = phaseMarginLoss_deg;

discretized_system.gainMargin_continuous_dB = 20*log10(gainMargin_cont_abs);
discretized_system.gainMargin_discrete_original_dB = 20*log10(gainMargin_disc_abs);
discretized_system.gainMargin_discrete_retuned_dB = 20*log10(gainMargin_discRetuned_abs);

discretized_system.crossover_continuous_rad_s = gainCrossFreq_cont_rad_s;
discretized_system.crossover_discrete_original_rad_s = gainCrossFreq_disc_rad_s;
discretized_system.crossover_discrete_retuned_rad_s = gainCrossFreq_discRetuned_rad_s;
discretized_system.crossoverShift_percent = crossoverShift_percent;

discretized_system.phaseCrossFreq_continuous_rad_s = phaseCrossFreq_cont_rad_s;
discretized_system.phaseCrossFreq_discrete_original_rad_s = phaseCrossFreq_disc_rad_s;
discretized_system.phaseCrossFreq_discrete_retuned_rad_s = phaseCrossFreq_discRetuned_rad_s;

discretized_system.zohHalfSampleLag_deg = zohHalfSampleLag_deg;
discretized_system.tustinWarpedFreq_rad_s = tustinWarpedFreq_rad_s;
discretized_system.tustinWarp_percent = tustinWarp_percent;

discretized_system.numUnstablePoles_original = numUnstablePoles_original;
discretized_system.numUnstablePoles_retuned = numUnstablePoles_retuned;
discretized_system.isClosedLoopStable_original = isClosedLoopStable_original;
discretized_system.isClosedLoopStable_retuned = isClosedLoopStable_retuned;

discretized_system.desiredDiscretePhaseMargin_deg = desiredDiscretePhaseMargin_deg;
discretized_system.targetContinuousPhaseMargin_deg = targetContinuousPhaseMargin_deg;
discretized_system.currentLeadPhase_deg = currentLeadPhase_deg;
discretized_system.extraLeadRequired_deg = extraLeadRequired_deg;

discretized_system.retunedZeroTimeConstant_s = retunedZeroTimeConstant_s;
discretized_system.retunedIntegralTimeConstant_s = retunedIntegralTimeConstant_s;
discretized_system.retunedPoleTimeConstant_s = retunedPoleTimeConstant_s;
discretized_system.retunedProportionalGain = retunedProportionalGain;

%% Backward-compatible field names used by main.m
discretized_system.ts = sampleTime_s;
discretized_system.Pz_zoh = plant_discrete_zoh;
discretized_system.Cz_tustin = controller_discrete_tustin;
discretized_system.OL_disc = openLoop_discrete_original;
discretized_system.CL_disc = closedLoop_discrete_original;
discretized_system.alpha_orig = sim_out.alpha;
discretized_system.Cz_retuned = controller_discrete_retunedForDiscretisation;
discretized_system.OL_retuned = openLoop_discrete_retuned;
discretized_system.CL_retuned = closedLoop_discrete_retuned;
discretized_system.PM_cont = phaseMargin_cont_deg;
discretized_system.PM_disc = phaseMargin_disc_deg;
discretized_system.PM_retuned = phaseMargin_discRetuned_deg;
discretized_system.GM_cont_dB = 20*log10(gainMargin_cont_abs);
discretized_system.GM_disc_dB = 20*log10(gainMargin_disc_abs);
discretized_system.GM_retuned_dB = 20*log10(gainMargin_discRetuned_abs);
discretized_system.wc_cont = gainCrossFreq_cont_rad_s;
discretized_system.wc_disc = gainCrossFreq_disc_rad_s;
discretized_system.wc_retuned = gainCrossFreq_discRetuned_rad_s;
discretized_system.phase_lag_ZoH_deg = zohHalfSampleLag_deg;

end

function [gainMargin_abs, phaseMargin_deg, gainCrossFreq_rad_s] = ...
    checkMargins(label, gainMargin_abs, phaseMargin_deg, gainCrossFreq_rad_s)
% margin() returns empty or NaN when no crossover exists. Report that instead
% of packing meaningless numbers into the summary table and output struct.
if isempty(gainMargin_abs);      gainMargin_abs = NaN;      end
if isempty(phaseMargin_deg);     phaseMargin_deg = NaN;     end
if isempty(gainCrossFreq_rad_s); gainCrossFreq_rad_s = NaN; end

if ~isfinite(phaseMargin_deg) || ~isfinite(gainCrossFreq_rad_s)
    warning('discretisation:noCrossover', ...
        ['No valid gain crossover found for %s (PM = %g deg, wc = %g rad/s); ', ...
         'the reported margins for this configuration are meaningless.'], ...
        label, phaseMargin_deg, gainCrossFreq_rad_s);
elseif phaseMargin_deg <= 0
    warning('discretisation:nonPositivePhaseMargin', ...
        'Phase margin for %s is %.2f deg, i.e. the open loop is not stable.', ...
        label, phaseMargin_deg);
end
end