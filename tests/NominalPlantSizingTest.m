classdef NominalPlantSizingTest < matlab.unittest.TestCase
    % Unit tests for nominal_plant_sizing.m (deliverable b).
    %
    % The function sizes the arm length, inertia and stiffness from the
    % actuator voltage budget, so the tests check the sizing relations rather
    % than hard-coded numbers.

    properties (Constant)
        SafetyMargin = 0.9   % hard-coded inside nominal_plant_sizing.m
    end

    properties
        params
        ref
    end

    methods (TestClassSetup)

        function loadFixtures(testCase)
            testCase.params = parameters();
            testCase.ref = NominalPlantSizingTest.makeRef();
        end

    end

    methods (Static)

        function ref = makeRef(varargin)
            % Minimal stand-in for the Simulink reference generator output.
            ref = struct( ...
                         'theta_max',    deg2rad(1), ...
                         'dtheta_max',   5, ...
                         'ddtheta_max',  500, ...
                         'dddtheta_max', 5e4);
            for i = 1:2:numel(varargin)
                ref.(varargin{i}) = varargin{i + 1};
            end
        end

    end

    methods (Test)

        function returnsAllDocumentedFields(testCase)
            out = nominal_plant_sizing(testCase.params, testCase.ref);
            expected = {'r_arm_m', 'J_kgm2', 'k_Nm_per_rad', 'd_Nms_per_rad', 'zeta', ...
                        'wn_rad_s', 'margin', 'Umax_V', 'stroke_check_m', 'stroke_half_m', ...
                        'stroke_ok', 'offset_ok', 'mirror_offset_min_m', 'mirror_offset_max_m', ...
                        'u_req_check'};
            for i = 1:numel(expected)
                testCase.verifyTrue(isfield(out, expected{i}), ...
                                    sprintf('Missing output field "%s".', expected{i}));
            end
            testCase.verifyEqual(out.margin, testCase.SafetyMargin, 'RelTol', 1e-12);
            testCase.verifyEqual(out.Umax_V, testCase.params.actuator.Umax_V, 'RelTol', 1e-12);
        end

        function inertiaFollowsAccelerationVoltageBudget(testCase)
            p = testCase.params;
            out = nominal_plant_sizing(p, testCase.ref);
            expectedJ = testCase.SafetyMargin * p.actuator.Umax_V * out.r_arm_m * ...
                p.actuator.Kf_N_per_A / (p.actuator.R25_ohm * testCase.ref.ddtheta_max);
            testCase.verifyEqual(out.J_kgm2, expectedJ, 'RelTol', 1e-12);
            testCase.verifyGreaterThan(out.J_kgm2, 0);
        end

        function stiffnessFollowsPositionVoltageBudget(testCase)
            p = testCase.params;
            out = nominal_plant_sizing(p, testCase.ref);
            expectedK = testCase.SafetyMargin * p.actuator.Umax_V * out.r_arm_m * ...
                p.actuator.Kf_N_per_A / (p.actuator.R25_ohm * testCase.ref.theta_max);
            testCase.verifyEqual(out.k_Nm_per_rad, expectedK, 'RelTol', 1e-12);
            testCase.verifyGreaterThan(out.k_Nm_per_rad, 0);
        end

        function dampingIsBackEmfDampingAndNotNeglected(testCase)
            p = testCase.params;
            out = nominal_plant_sizing(p, testCase.ref);
            expectedD = (p.actuator.Kf_N_per_A^2 / p.actuator.R25_ohm) * out.r_arm_m^2;
            testCase.verifyEqual(out.d_Nms_per_rad, expectedD, 'RelTol', 1e-12);
            testCase.verifyGreaterThan(out.d_Nms_per_rad, 0, ...
                                       'Back-EMF damping must not collapse to the zero placeholder in parameters.m.');
        end

        function naturalFrequencyAndDampingRatioAreConsistent(testCase)
            out = nominal_plant_sizing(testCase.params, testCase.ref);
            testCase.verifyEqual(out.wn_rad_s, sqrt(out.k_Nm_per_rad / out.J_kgm2), 'RelTol', 1e-12);
            testCase.verifyEqual(out.zeta, ...
                                 out.d_Nms_per_rad / (2 * sqrt(out.J_kgm2 * out.k_Nm_per_rad)), 'RelTol', 1e-12);
            % Underdamped second-order mechanism.
            testCase.verifyGreaterThan(out.zeta, 0);
            testCase.verifyLessThan(out.zeta, 1);
        end

        function accelerationAndPositionVoltageTermsSaturateTheBudget(testCase)
            % By construction J and k_eq are sized so that both terms consume
            % exactly margin*Umax.
            out = nominal_plant_sizing(testCase.params, testCase.ref);
            budget_V = testCase.SafetyMargin * out.Umax_V;
            testCase.verifyEqual(out.u_req_check.u_J_V, budget_V, 'RelTol', 1e-9);
            testCase.verifyEqual(out.u_req_check.u_k_V, budget_V, 'RelTol', 1e-9);
        end

        function backEmfVoltageTermStaysWithinBudget(testCase)
            % r_arm is fixed at 70 mm instead of the value solved from the
            % back-EMF budget, so u_v is only required to fit in the budget.
            out = nominal_plant_sizing(testCase.params, testCase.ref);
            testCase.verifyLessThanOrEqual(out.u_req_check.u_v_V, ...
                                           testCase.SafetyMargin * out.Umax_V);
            expected_u_v = testCase.params.actuator.Kf_N_per_A * out.r_arm_m * ...
                testCase.ref.dtheta_max;
            testCase.verifyEqual(out.u_req_check.u_v_V, expected_u_v, 'RelTol', 1e-12);
        end

        function armLengthIsIndependentOfReference(testCase)
            % r_arm is currently overridden with a fixed 70 mm design choice.
            slow = nominal_plant_sizing(testCase.params, ...
                                        NominalPlantSizingTest.makeRef('dtheta_max', 1));
            fast = nominal_plant_sizing(testCase.params, ...
                                        NominalPlantSizingTest.makeRef('dtheta_max', 20));
            testCase.verifyEqual(slow.r_arm_m, fast.r_arm_m, 'RelTol', 1e-12);
            testCase.verifyEqual(slow.r_arm_m, 70e-3, 'RelTol', 1e-12);
        end

        function inertiaScalesInverselyWithPeakAcceleration(testCase)
            base = nominal_plant_sizing(testCase.params, ...
                                        NominalPlantSizingTest.makeRef('ddtheta_max', 500));
            twice = nominal_plant_sizing(testCase.params, ...
                                         NominalPlantSizingTest.makeRef('ddtheta_max', 1000));
            testCase.verifyEqual(twice.J_kgm2, base.J_kgm2 / 2, 'RelTol', 1e-12);
            % Stiffness only depends on theta_max, so it must be unchanged.
            testCase.verifyEqual(twice.k_Nm_per_rad, base.k_Nm_per_rad, 'RelTol', 1e-12);
        end

        function stiffnessScalesInverselyWithStrokeAngle(testCase)
            base = nominal_plant_sizing(testCase.params, ...
                                        NominalPlantSizingTest.makeRef('theta_max', deg2rad(1)));
            wider = nominal_plant_sizing(testCase.params, ...
                                         NominalPlantSizingTest.makeRef('theta_max', deg2rad(2)));
            testCase.verifyEqual(wider.k_Nm_per_rad, base.k_Nm_per_rad / 2, 'RelTol', 1e-12);
        end

        function strokeCheckComparesArmTravelWithHalfStroke(testCase)
            p = testCase.params;
            out = nominal_plant_sizing(p, testCase.ref);
            testCase.verifyEqual(out.stroke_check_m, ...
                                 out.r_arm_m * p.spec.mirror_angle_max_rad, 'RelTol', 1e-12);
            testCase.verifyEqual(out.stroke_half_m, p.actuator.stroke_half_m, 'RelTol', 1e-12);
            testCase.verifyEqual(out.stroke_ok, ...
                                 out.stroke_check_m <= p.actuator.stroke_half_m);
            testCase.verifyTrue(out.stroke_ok, ...
                                'Nominal design must fit inside the VCM half-stroke.');
        end

        function strokeCheckFailsForAnOversizedMirrorAngle(testCase)
            p = testCase.params;
            % 70 mm arm with a 20 deg mirror angle needs far more than 7.5 mm.
            p.spec.mirror_angle_max_rad = deg2rad(20);
            out = nominal_plant_sizing(p, testCase.ref);
            testCase.verifyFalse(out.stroke_ok);
        end

        function offsetCheckReflectsTheMechanismOffsetWindow(testCase)
            p = testCase.params;
            out = nominal_plant_sizing(p, testCase.ref);
            testCase.verifyEqual(out.mirror_offset_min_m, p.spec.mirror_offset_min_m);
            testCase.verifyEqual(out.mirror_offset_max_m, p.spec.mirror_offset_max_m);
            testCase.verifyEqual(out.offset_ok, ...
                                 out.r_arm_m >= p.spec.mirror_offset_min_m && ...
                                 out.r_arm_m <= p.spec.mirror_offset_max_m);
        end

        function offsetCheckFailsWhenArmIsOutsideTheWindow(testCase)
            p = testCase.params;
            p.spec.mirror_offset_min_m = 100e-3;   % 70 mm arm now too short
            p.spec.mirror_offset_max_m = 150e-3;
            out = nominal_plant_sizing(p, testCase.ref);
            testCase.verifyFalse(out.offset_ok);
        end

    end
end
