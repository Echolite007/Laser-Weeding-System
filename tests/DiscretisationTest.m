classdef DiscretisationTest < matlab.unittest.TestCase
    % Unit tests for discretisation.m (deliverable j).
    %
    % The function discretises the plant with ZOH and the controller with
    % Tustin, quantifies the phase-margin loss and retunes the lead ratio.
    % A synthetic second-order plant plus lead-lag controller is used so the
    % tests do not depend on the SPACAR simulation.

    properties
        params
        controllerDesign
        simOut
        result
    end

    methods (TestClassSetup)

        function requireControlToolbox(testCase)
            testCase.assumeTrue(logical(exist('c2d', 'file')), ...
                                'Control System Toolbox is required for discretisation.m.');
        end

    end

    methods (TestMethodSetup)

        function hideFigures(testCase)
            % discretisation.m draws Bode, margin and pole-zero figures.
            original = get(0, 'DefaultFigureVisible');
            set(0, 'DefaultFigureVisible', 'off');
            testCase.addTeardown(@() set(0, 'DefaultFigureVisible', original));
            testCase.addTeardown(@() close('all'));
        end

        function runDiscretisation(testCase)
            testCase.params = parameters();
            testCase.controllerDesign = struct('beta', 2);
            testCase.simOut = DiscretisationTest.makeSimOut();
            testCase.result = testCase.discretise(testCase.simOut);
        end

    end

    methods (Access = private)

        function verifySameResponse(testCase, actual, expected)
            % Compare two systems by their frequency response, which is
            % insensitive to how a product was expanded internally.
            w = [1 10 100 1000];
            testCase.verifyEqual(squeeze(freqresp(actual, w)), ...
                                 squeeze(freqresp(expected, w)), 'RelTol', 1e-9);
        end

        function out = discretise(testCase, simOut)
            % Swallow the summary table printed by discretisation.m.
            captured = evalc(['out = discretisation(testCase.params, ' ...
                              'testCase.controllerDesign, simOut);']); %#ok<NASGU>
        end

    end

    methods (Static)

        function simOut = makeSimOut(varargin)
            % Voltage-to-sensor plant of the nominal mechanism plus a lead-lag
            % controller of the shape used in simulation.m.
            p = parameters();
            sizing = nominal_plant_sizing(p, NominalPlantSizingTest.makeRef());
            s = tf('s');

            plant = (sizing.r_arm_m * p.actuator.Kf_N_per_A / p.actuator.R25_ohm) / ...
                (sizing.J_kgm2 * s^2 + sizing.d_Nms_per_rad * s + sizing.k_Nm_per_rad);

            alpha = 0.2;
            wc = 600;
            beta = 2;
            % Equivalent mass seen by the voltage input: theta/V has the gain
            % r_arm*Kf/R, so J is referred back through that gain.
            m_eq = sizing.J_kgm2 * p.actuator.R25_ohm / ...
                (sizing.r_arm_m * p.actuator.Kf_N_per_A);

            tz = sqrt(1 / alpha) / wc;
            ti = beta * tz;
            tp = 1 / (wc * sqrt(1 / alpha));
            Kp = m_eq * wc^2 / sqrt(1 / alpha);
            controller = Kp * (tz * s + 1) * (ti * s + 1) / ((tp * s + 1) * ti * s);

            simOut = struct( ...
                            'plant_voltage_to_sensor_reduced', plant, ...
                            'controller_retunedPID',           controller, ...
                            'wc_target',                       wc, ...
                            'alpha',                           alpha, ...
                            'm_eq',                            m_eq);
            for i = 1:2:numel(varargin)
                simOut.(varargin{i}) = varargin{i + 1};
            end
        end

    end

    methods (Test)

        function samplingQuantitiesFollowFromTheControllerSampleTime(testCase)
            d = testCase.result;
            ts = testCase.params.ctrl.ts_s;
            testCase.verifyEqual(d.sampleTime_s, ts, 'RelTol', 1e-12);
            testCase.verifyEqual(d.samplingFrequency_Hz, 1 / ts, 'RelTol', 1e-12);
            testCase.verifyEqual(d.nyquistFrequency_Hz, 1 / (2 * ts), 'RelTol', 1e-12);
            testCase.verifyEqual(d.nyquistFrequency_rad_s, pi / ts, 'RelTol', 1e-12);
        end

        function targetCrossoverIsTakenFromTheSimulationAndStaysBelowNyquist(testCase)
            d = testCase.result;
            testCase.verifyEqual(d.targetCrossover_rad_s, testCase.simOut.wc_target, ...
                                 'RelTol', 1e-12);
            testCase.verifyEqual(d.crossoverToNyquistRatio, ...
                                 d.targetCrossover_rad_s / d.nyquistFrequency_rad_s, 'RelTol', 1e-12);
            testCase.verifyLessThan(d.crossoverToNyquistRatio, 1);
        end

        function plantIsDiscretisedWithZeroOrderHold(testCase)
            d = testCase.result;
            expected = c2d(testCase.simOut.plant_voltage_to_sensor_reduced, ...
                           d.sampleTime_s, 'zoh');
            testCase.verifyEqual(d.plant_discrete_zoh.Ts, d.sampleTime_s, 'RelTol', 1e-12);
            testCase.verifySameResponse(d.plant_discrete_zoh, expected);
        end

        function controllerIsDiscretisedWithTustin(testCase)
            d = testCase.result;
            expected = c2d(testCase.simOut.controller_retunedPID, d.sampleTime_s, 'tustin');
            testCase.verifyEqual(d.controller_discrete_tustin.Ts, d.sampleTime_s, ...
                                 'RelTol', 1e-12);
            testCase.verifySameResponse(d.controller_discrete_tustin, expected);
        end

        function openLoopsAreControllerTimesPlant(testCase)
            d = testCase.result;
            expectedContinuous = testCase.simOut.controller_retunedPID * ...
                testCase.simOut.plant_voltage_to_sensor_reduced;
            testCase.verifySameResponse(d.openLoop_continuous, expectedContinuous);

            expectedDiscrete = d.controller_discrete_tustin * d.plant_discrete_zoh;
            testCase.verifyEqual(d.openLoop_discrete_original.Ts, d.sampleTime_s, ...
                                 'RelTol', 1e-12);
            testCase.verifySameResponse(d.openLoop_discrete_original, expectedDiscrete);
        end

        function closedLoopIsUnityFeedbackOfTheDiscreteOpenLoop(testCase)
            d = testCase.result;
            expected = feedback(d.openLoop_discrete_original, 1);
            testCase.verifyEqual(sort(pole(d.closedLoop_discrete_original)), ...
                                 sort(pole(expected)), 'AbsTol', 1e-9);
        end

        function marginsAreReportedInDecibelsAndDegrees(testCase)
            d = testCase.result;
            [gm, pm] = margin(d.openLoop_continuous);
            testCase.verifyEqual(d.gainMargin_continuous_dB, 20 * log10(gm), 'RelTol', 1e-9);
            testCase.verifyEqual(d.phaseMargin_continuous_deg, pm, 'RelTol', 1e-9);
        end

        function phaseMarginLossIsContinuousMinusDiscrete(testCase)
            d = testCase.result;
            testCase.verifyEqual(d.phaseMarginLoss_deg, ...
                                 d.phaseMargin_continuous_deg - d.phaseMargin_discrete_original_deg, ...
                                 'RelTol', 1e-9);
            % Sampling can only remove phase from the loop.
            testCase.verifyGreaterThan(d.phaseMarginLoss_deg, 0);
        end

        function crossoverShiftIsRelativeToTheContinuousCrossover(testCase)
            d = testCase.result;
            expected = (d.crossover_discrete_original_rad_s - ...
                        d.crossover_continuous_rad_s) / d.crossover_continuous_rad_s * 100;
            testCase.verifyEqual(d.crossoverShift_percent, expected, 'RelTol', 1e-9);
        end

        function zeroOrderHoldLagIsHalfASampleAtCrossover(testCase)
            d = testCase.result;
            expected = d.sampleTime_s * d.crossover_continuous_rad_s / 2 * (180 / pi);
            testCase.verifyEqual(d.zohHalfSampleLag_deg, expected, 'RelTol', 1e-9);
            testCase.verifyGreaterThan(d.zohHalfSampleLag_deg, 0);
        end

        function tustinFrequencyWarpingIsQuantified(testCase)
            d = testCase.result;
            wc = d.crossover_continuous_rad_s;
            ts = d.sampleTime_s;
            expectedWarped = 2 / ts * tan(wc * ts / 2);
            testCase.verifyEqual(d.tustinWarpedFreq_rad_s, expectedWarped, 'RelTol', 1e-9);
            testCase.verifyEqual(d.tustinWarp_percent, ...
                                 (expectedWarped - wc) / wc * 100, 'RelTol', 1e-9);
            % tan(x) > x, so Tustin always warps upwards.
            testCase.verifyGreaterThan(d.tustinWarp_percent, 0);
        end

        function retuningTargetsThirtyDegreesOfDiscretePhaseMargin(testCase)
            d = testCase.result;
            testCase.verifyEqual(d.desiredDiscretePhaseMargin_deg, 30);
            testCase.verifyEqual(d.targetContinuousPhaseMargin_deg, ...
                                 30 + d.phaseMarginLoss_deg, 'RelTol', 1e-9);
            testCase.verifyEqual(d.extraLeadRequired_deg, ...
                                 d.targetContinuousPhaseMargin_deg - d.phaseMargin_continuous_deg, ...
                                 'RelTol', 1e-9);
            testCase.verifyEqual(d.currentLeadPhase_deg, ...
                                 asind((1 - d.alpha_original) / (1 + d.alpha_original)), 'RelTol', 1e-9);
        end

        function retunedLeadRatioFollowsTheSelectedBranch(testCase)
            d = testCase.result;
            if d.extraLeadRequired_deg <= 0
                testCase.verifyEqual(d.alpha_retuned, d.alpha_original, 'RelTol', 1e-12);
            else
                newLead = min(d.currentLeadPhase_deg + d.extraLeadRequired_deg, 78);
                expected = (1 - sind(newLead)) / (1 + sind(newLead));
                expected = max(0.001, min(0.999, expected));
                testCase.verifyEqual(d.alpha_retuned, expected, 'RelTol', 1e-9);
            end
            testCase.verifyGreaterThanOrEqual(d.alpha_retuned, 0.001);
            testCase.verifyLessThanOrEqual(d.alpha_retuned, 0.999);
        end

        function retunedLeadRatioIsClampedForExcessiveLeadDemand(testCase)
            % An almost undamped original lead ratio drives the lead demand into
            % the 78 deg cap, which corresponds to the smallest allowed alpha.
            d = testCase.discretise(DiscretisationTest.makeSimOut('alpha', 1e-6));
            capAlpha = (1 - sind(78)) / (1 + sind(78));
            testCase.verifyEqual(d.alpha_retuned, capAlpha, 'RelTol', 1e-9);
        end

        function retunedControllerTimeConstantsFollowTheLeadLagLayout(testCase)
            d = testCase.result;
            wc = d.targetCrossover_rad_s;
            beta = testCase.controllerDesign.beta;
            testCase.verifyEqual(d.retunedZeroTimeConstant_s, ...
                                 sqrt(1 / d.alpha_retuned) / wc, 'RelTol', 1e-12);
            testCase.verifyEqual(d.retunedIntegralTimeConstant_s, ...
                                 beta * d.retunedZeroTimeConstant_s, 'RelTol', 1e-12);
            testCase.verifyEqual(d.retunedPoleTimeConstant_s, ...
                                 1 / (wc * sqrt(1 / d.alpha_retuned)), 'RelTol', 1e-12);
            testCase.verifyEqual(d.retunedProportionalGain, ...
                                 testCase.simOut.m_eq * wc^2 / sqrt(1 / d.alpha_retuned), 'RelTol', 1e-12);
            % Lead network: the zero must sit below the pole.
            testCase.verifyGreaterThan(d.retunedZeroTimeConstant_s, ...
                                       d.retunedPoleTimeConstant_s);
        end

        function retunedControllerIsDiscretisedWithTustinAtTheSameRate(testCase)
            d = testCase.result;
            expected = c2d(d.controller_continuous_retunedForDiscretisation, ...
                           d.sampleTime_s, 'tustin');
            testCase.verifyEqual(d.controller_discrete_retunedForDiscretisation.Ts, ...
                                 d.sampleTime_s, 'RelTol', 1e-12);
            testCase.verifySameResponse( ...
                                        d.controller_discrete_retunedForDiscretisation, expected);
        end

        function stabilityFlagsMatchThePoleLocations(testCase)
            d = testCase.result;
            testCase.verifyEqual(d.numUnstablePoles_original, ...
                                 sum(abs(pole(d.closedLoop_discrete_original)) > 1));
            testCase.verifyEqual(d.numUnstablePoles_retuned, ...
                                 sum(abs(pole(d.closedLoop_discrete_retuned)) > 1));
            testCase.verifyEqual(d.isClosedLoopStable_original, ...
                                 d.numUnstablePoles_original == 0);
            testCase.verifyEqual(d.isClosedLoopStable_retuned, ...
                                 d.numUnstablePoles_retuned == 0);
        end

        function backwardCompatibleAliasesMirrorTheCanonicalFields(testCase)
            d = testCase.result;
            testCase.verifyEqual(d.ts, d.sampleTime_s, 'RelTol', 1e-12);
            testCase.verifyEqual(d.alpha_orig, d.alpha_original, 'RelTol', 1e-12);
            testCase.verifyEqual(d.PM_cont, d.phaseMargin_continuous_deg, 'RelTol', 1e-12);
            testCase.verifyEqual(d.PM_disc, d.phaseMargin_discrete_original_deg, 'RelTol', 1e-12);
            testCase.verifyEqual(d.PM_retuned, d.phaseMargin_discrete_retuned_deg, 'RelTol', 1e-12);
            testCase.verifyEqual(d.GM_cont_dB, d.gainMargin_continuous_dB, 'RelTol', 1e-12);
            testCase.verifyEqual(d.GM_disc_dB, d.gainMargin_discrete_original_dB, 'RelTol', 1e-12);
            testCase.verifyEqual(d.GM_retuned_dB, d.gainMargin_discrete_retuned_dB, 'RelTol', 1e-12);
            testCase.verifyEqual(d.wc_cont, d.crossover_continuous_rad_s, 'RelTol', 1e-12);
            testCase.verifyEqual(d.wc_disc, d.crossover_discrete_original_rad_s, 'RelTol', 1e-12);
            testCase.verifyEqual(d.wc_retuned, d.crossover_discrete_retuned_rad_s, 'RelTol', 1e-12);
            testCase.verifyEqual(d.phase_lag_ZoH_deg, d.zohHalfSampleLag_deg, 'RelTol', 1e-12);
            testCase.verifyEqual(pole(d.Pz_zoh), pole(d.plant_discrete_zoh), 'AbsTol', 1e-12);
            testCase.verifyEqual(pole(d.Cz_tustin), pole(d.controller_discrete_tustin), ...
                                 'AbsTol', 1e-12);
            testCase.verifyEqual(pole(d.Cz_retuned), ...
                                 pole(d.controller_discrete_retunedForDiscretisation), 'AbsTol', 1e-12);
            testCase.verifyEqual(pole(d.OL_disc), pole(d.openLoop_discrete_original), ...
                                 'AbsTol', 1e-12);
            testCase.verifyEqual(pole(d.OL_retuned), pole(d.openLoop_discrete_retuned), ...
                                 'AbsTol', 1e-12);
            testCase.verifyEqual(pole(d.CL_disc), pole(d.closedLoop_discrete_original), ...
                                 'AbsTol', 1e-12);
            testCase.verifyEqual(pole(d.CL_retuned), pole(d.closedLoop_discrete_retuned), ...
                                 'AbsTol', 1e-12);
        end

    end
end
