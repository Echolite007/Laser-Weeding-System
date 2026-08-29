function discretized_system = discretisation(p, controller_design, sim_out)

    %% Setup
    s = tf('s');

    sampleTime_s = p.ctrl.ts_s;
    samplingFrequency_Hz = 1 / sampleTime_s;
    nyquistFrequency_Hz = samplingFrequency_Hz / 2;
    nyquistFrequency_rad_s = pi / sampleTime_s;

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

    margins_cont = loop_margins(openLoop_continuous);
    margins_disc = loop_margins(openLoop_discrete_original);

    phaseMarginLoss_deg = margins_cont.PM_deg - margins_disc.PM_deg;
    crossoverShift_percent = ...
        (margins_disc.wcp_rad_s - margins_cont.wcp_rad_s) / margins_cont.wcp_rad_s * 100;

    [numUnstablePoles_original, isClosedLoopStable_original] = ...
        discrete_pole_stability(closedLoop_discrete_original);

    %% Step j.2: Quantify phase-lag source
    zohHalfSampleLag_deg = sampleTime_s * margins_cont.wcp_rad_s / 2 * (180 / pi);

    tustinWarpedFreq_rad_s = ...
        2 / sampleTime_s * tan(margins_cont.wcp_rad_s * sampleTime_s / 2);

    tustinWarp_percent = ...
        (tustinWarpedFreq_rad_s - margins_cont.wcp_rad_s) / margins_cont.wcp_rad_s * 100;

    %% Step j.3: Retune controller for discrete implementation
    desiredDiscretePhaseMargin_deg = 30;

    targetContinuousPhaseMargin_deg = ...
        desiredDiscretePhaseMargin_deg + phaseMarginLoss_deg;

    currentLeadPhase_deg = asind( ...
                                 (1 - sim_out.alpha) / ...
                                 (1 + sim_out.alpha));

    extraLeadRequired_deg = targetContinuousPhaseMargin_deg - margins_cont.PM_deg;

    if extraLeadRequired_deg <= 0
        retunedAlpha = sim_out.alpha;
    else
        newLeadPhase_deg = min(currentLeadPhase_deg + extraLeadRequired_deg, 78);

        retunedAlpha = alpha_from_phase_lead(newLeadPhase_deg, [0.001 0.999]);
    end

    [controller_continuous_retunedForDiscretisation, retunedGains] = ...
        build_pid_lead(m_eq, targetCrossover_rad_s, retunedAlpha, beta, s);

    retunedZeroTimeConstant_s = retunedGains.tau_z;
    retunedIntegralTimeConstant_s = retunedGains.tau_i;
    retunedPoleTimeConstant_s = retunedGains.tau_p;
    retunedProportionalGain = retunedGains.kp;

    openLoop_continuous_retunedForDiscretisation = ...
        controller_continuous_retunedForDiscretisation * plant_voltage_to_sensor_reduced;

    controller_discrete_retunedForDiscretisation = c2d( ...
                                                       controller_continuous_retunedForDiscretisation, ...
                                                       sampleTime_s, ...
                                                       'tustin');

    openLoop_discrete_retuned = ...
        controller_discrete_retunedForDiscretisation * plant_discrete_zoh;

    closedLoop_discrete_retuned = feedback(openLoop_discrete_retuned, 1);

    margins_contRetuned = loop_margins(openLoop_continuous_retunedForDiscretisation);
    margins_discRetuned = loop_margins(openLoop_discrete_retuned);

    [numUnstablePoles_retuned, isClosedLoopStable_retuned] = ...
        discrete_pole_stability(closedLoop_discrete_retuned);

    %% Summary table
    print_margin_table('--- Deliverable j summary table ---', 48, { ...
                                                                   'Continuous, original alpha', ...
                                                                   margins_cont.wcp_rad_s, margins_cont.PM_deg, margins_cont.GM_dB; ...
                                                                   'Discrete Tustin + ZOH, original alpha', ...
                                                                   margins_disc.wcp_rad_s, margins_disc.PM_deg, margins_disc.GM_dB; ...
                                                                   'Discrete Tustin + ZOH, retuned alpha', ...
                                                                   margins_discRetuned.wcp_rad_s, margins_discRetuned.PM_deg, margins_discRetuned.GM_dB});

    %% Figure j.1: Continuous vs discrete open-loop Bode

    % opts.PhaseWrappingBranch = -180;
    opts = bode_options('XLim', [1 nyquistFrequency_rad_s]);

    figure('Name', 'Deliverable j: Continuous vs Discrete OL Bode');
    bode( ...
         openLoop_continuous, ...
         openLoop_discrete_original, ...
         openLoop_discrete_retuned, opts);

    legend( ...
           'Continuous C P_{em}', ...
           'Discrete, Tustin C + ZOH P, original alpha', ...
           'Discrete, Tustin C_{retuned} + ZOH P', ...
           'Location', 'southwest');

    style_plot('Deliverable j - Effect of discretisation on open-loop frequency response');

    %% Figure j.2: Discrete open-loop margin plot, retuned
    figure('Name', 'Deliverable j: Discrete OL margin, retuned');
    margin(openLoop_discrete_retuned);
    style_plot('Deliverable j - Discrete open-loop C_z(retuned) P_z(ZOH)');

    %% Figure j.3: Discrete closed-loop pole-zero map
    figure('Name', 'Deliverable j: Discrete CL pole-zero map');
    pzmap(closedLoop_discrete_original, closedLoop_discrete_retuned);
    zgrid;

    legend( ...
           'Original alpha', ...
           'Retuned alpha', ...
           'Location', 'northwest');

    title('Deliverable j - Discrete closed-loop poles, unit circle is stability boundary');

    %% Pack outputs
    discretized_system.sampleTime_s = sampleTime_s;
    discretized_system.samplingFrequency_Hz = samplingFrequency_Hz;
    discretized_system.nyquistFrequency_Hz = nyquistFrequency_Hz;
    discretized_system.nyquistFrequency_rad_s = nyquistFrequency_rad_s;
    discretized_system.targetCrossover_rad_s = targetCrossover_rad_s;
    discretized_system.crossoverToNyquistRatio = targetCrossover_rad_s / nyquistFrequency_rad_s;

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

    discretized_system.phaseMargin_continuous_deg = margins_cont.PM_deg;
    discretized_system.phaseMargin_discrete_original_deg = margins_disc.PM_deg;
    discretized_system.phaseMargin_discrete_retuned_deg = margins_discRetuned.PM_deg;
    discretized_system.phaseMarginLoss_deg = phaseMarginLoss_deg;

    discretized_system.gainMargin_continuous_dB = margins_cont.GM_dB;
    discretized_system.gainMargin_discrete_original_dB = margins_disc.GM_dB;
    discretized_system.gainMargin_discrete_retuned_dB = margins_discRetuned.GM_dB;

    discretized_system.crossover_continuous_rad_s = margins_cont.wcp_rad_s;
    discretized_system.crossover_discrete_original_rad_s = margins_disc.wcp_rad_s;
    discretized_system.crossover_discrete_retuned_rad_s = margins_discRetuned.wcp_rad_s;
    discretized_system.crossoverShift_percent = crossoverShift_percent;

    discretized_system.phaseCrossFreq_continuous_rad_s = margins_cont.wcg_rad_s;
    discretized_system.phaseCrossFreq_discrete_original_rad_s = margins_disc.wcg_rad_s;
    discretized_system.phaseCrossFreq_discrete_retuned_rad_s = margins_discRetuned.wcg_rad_s;

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
    discretized_system.PM_cont = margins_cont.PM_deg;
    discretized_system.PM_disc = margins_disc.PM_deg;
    discretized_system.PM_retuned = margins_discRetuned.PM_deg;
    discretized_system.GM_cont_dB = margins_cont.GM_dB;
    discretized_system.GM_disc_dB = margins_disc.GM_dB;
    discretized_system.GM_retuned_dB = margins_discRetuned.GM_dB;
    discretized_system.wc_cont = margins_cont.wcp_rad_s;
    discretized_system.wc_disc = margins_disc.wcp_rad_s;
    discretized_system.wc_retuned = margins_discRetuned.wcp_rad_s;
    discretized_system.phase_lag_ZoH_deg = zohHalfSampleLag_deg;

end
