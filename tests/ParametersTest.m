classdef ParametersTest < matlab.unittest.TestCase
    % Unit tests for parameters.m, the loader for all system parameters.

    properties
        params
    end

    methods (TestClassSetup)
        function loadParams(testCase)
            testCase.params = parameters();
        end
    end

    methods (Test)

        function returnsAllTopLevelGroups(testCase)
            expected = {'spec','actuator','amplifier','sensor','mech','ctrl','flex'};
            testCase.verifyClass(testCase.params, 'struct');
            for i = 1:numel(expected)
                testCase.verifyTrue(isfield(testCase.params, expected{i}), ...
                    sprintf('Missing parameter group "%s".', expected{i}));
            end
        end

        function angularAccuracyFollowsOpticalLeverRelation(testCase)
            spec = testCase.params.spec;
            % A mirror rotation of dtheta deflects the beam by 2*dtheta, so the
            % allowed angular error is the spot accuracy over twice the path length.
            testCase.verifyEqual(spec.angular_accuracy_rad, ...
                spec.spot_accuracy_m / (2*spec.optical_path_length_m), ...
                'RelTol', 1e-12);
        end

        function mirrorAngleMaxIsOneDegree(testCase)
            testCase.verifyEqual(testCase.params.spec.mirror_angle_max_rad, ...
                deg2rad(1), 'RelTol', 1e-12);
        end

        function specValuesArePositiveAndOrdered(testCase)
            spec = testCase.params.spec;
            positiveFields = {'spot_accuracy_m','optical_path_length_m', ...
                'mirror_angle_max_rad','spot_amplitude_m','driving_speed_nom_mps', ...
                'driving_speed_max_mps','weeding_time_s','return_time_s', ...
                'mirror_offset_min_m','mirror_offset_max_m'};
            for i = 1:numel(positiveFields)
                testCase.verifyGreaterThan(spec.(positiveFields{i}), 0, ...
                    sprintf('spec.%s must be positive.', positiveFields{i}));
            end
            testCase.verifyLessThan(spec.driving_speed_nom_mps, spec.driving_speed_max_mps);
            testCase.verifyLessThan(spec.mirror_offset_min_m, spec.mirror_offset_max_m);
            testCase.verifyLessThan(spec.return_time_s, spec.weeding_time_s);
        end

        function actuatorContinuousVoltageFollowsFromContinuousCurrent(testCase)
            act = testCase.params.actuator;
            testCase.verifyEqual(act.Umax_V, act.Ic_A * act.R25_ohm, 'RelTol', 1e-12);
            % The usable continuous voltage must stay inside the datasheet limit.
            testCase.verifyLessThan(act.Umax_V, act.Umax_datasheet_V);
        end

        function actuatorElectricalTimeConstantMatchesDatasheet(testCase)
            act = testCase.params.actuator;
            testCase.verifyEqual(act.tau_e_from_LR_s, act.Lcoil_H / act.R25_ohm, ...
                'RelTol', 1e-12);
            % L/R and the datasheet value should agree to within 10 %.
            testCase.verifyEqual(act.tau_e_from_LR_s, act.tau_e_s, 'RelTol', 0.1);
        end

        function actuatorStrokeHalfIsHalfOfStroke(testCase)
            act = testCase.params.actuator;
            testCase.verifyEqual(act.stroke_half_m, act.stroke_m/2, 'RelTol', 1e-12);
        end

        function actuatorPeakRatingsExceedContinuousRatings(testCase)
            act = testCase.params.actuator;
            testCase.verifyGreaterThan(act.Fpk_N, act.Fc_N);
            testCase.verifyGreaterThan(act.Ipk_A, act.Ic_A);
        end

        function samplingParametersAreConsistent(testCase)
            ctrl = testCase.params.ctrl;
            testCase.verifyEqual(ctrl.ts_s, 1/ctrl.fs_hz, 'RelTol', 1e-12);
            % Target crossover must stay well below the Nyquist frequency.
            nyquist_rad_s = pi/ctrl.ts_s;
            testCase.verifyLessThan(ctrl.wc_rads, nyquist_rad_s);
        end

        function unresolvedParametersAreExplicitPlaceholders(testCase)
            % These are documented as "to be filled in later" and must stay NaN /
            % empty rather than silently defaulting to a number.
            p = testCase.params;
            testCase.verifyTrue(all(isnan(cell2mat(struct2cell(p.amplifier)))));
            testCase.verifyTrue(isnan(p.sensor.resolution_um));
            testCase.verifyTrue(isnan(p.sensor.mounting_radius_m));
            testCase.verifyTrue(isnan(p.mech.J_kgm2));
            testCase.verifyTrue(isnan(p.mech.k_Nm_per_rad));
            testCase.verifyTrue(isnan(p.mech.r_arm_m));
            testCase.verifyTrue(isnan(p.flex.E_Pa));
            testCase.verifyTrue(isnan(p.flex.sigma_y_Pa));
            testCase.verifyEmpty(p.flex.thickness_options_m);
        end

        function sensorResolutionRangeIsOrdered(testCase)
            sensor = testCase.params.sensor;
            testCase.verifyLessThan(sensor.resolution_min_um, sensor.resolution_max_um);
            testCase.verifyLessThan(sensor.ride_height_min_mm, sensor.ride_height_max_mm);
        end

        function callIsDeterministic(testCase)
            testCase.verifyEqual(parameters(), testCase.params);
        end

    end
end
