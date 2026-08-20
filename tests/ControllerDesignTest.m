classdef ControllerDesignTest < matlab.unittest.TestCase
    % Unit tests for controller_design.m (deliverable d).
    %
    % controller_design.m turns the phase margin into a lead ratio alpha and
    % then sizes the crossover frequency and the error constants kj, ka, kv for
    % two approaches (A: weeding phase only, B: worst case over the whole
    % reference). The tests pin those relations down.

    properties (Constant)
        Beta = 2      % design choices hard-coded inside controller_design.m
        Gamma = 0.9   % g1 = g2 = g3
    end

    properties
        params
        sizing
        ref
        controller
    end

    methods (TestClassSetup)
        function loadFixtures(testCase)
            testCase.params = parameters();
            testCase.sizing = nominal_plant_sizing(testCase.params, ...
                NominalPlantSizingTest.makeRef());
            testCase.ref = ControllerDesignTest.makeRef();
        end
    end

    methods (TestMethodSetup)
        function hideFigures(testCase)
            % controller_design.m always draws the tracking-error figure.
            original = get(0, 'DefaultFigureVisible');
            set(0, 'DefaultFigureVisible', 'off');
            testCase.addTeardown(@() set(0, 'DefaultFigureVisible', original));
            testCase.addTeardown(@() close('all'));
        end
        function runDesign(testCase)
            testCase.controller = controller_design(testCase.params, ...
                testCase.sizing, testCase.ref);
        end
    end

    methods (Static)
        function ref = makeRef(varargin)
            % Deterministic stand-in for the reference generator output: a
            % smooth weeding ramp followed by a fast return.
            ts = 1e-4;
            tw = 0.75;
            tr = 0.03;
            t = (0:ts:(tw+tr))';
            w = 2*pi/(tw+tr);
            v = 2;                 % peak angular rate [rad/s]
            ref = struct();
            ref.t     = t;
            ref.r     = -v*cos(w*t)/w;
            ref.dr    = v*sin(w*t);
            ref.ddr   = v*w*cos(w*t);
            ref.dddr  = -v*w^2*sin(w*t);
            ref.a2p         = 0.1;
            ref.theta_max   = max(abs(ref.r));
            ref.dtheta_max  = max(abs(ref.dr));
            ref.ddtheta_max = max(abs(ref.ddr));
            ref.dddtheta_max = max(abs(ref.dddr));
            for i = 1:2:numel(varargin)
                ref.(varargin{i}) = varargin{i+1};
            end
        end
    end

    methods (Test)

        function returnsBothApproachesAndRecommendation(testCase)
            c = testCase.controller;
            testCase.verifyTrue(isfield(c, 'A'));
            testCase.verifyTrue(isfield(c, 'B'));
            testCase.verifyEqual(c.recommended, 'B');
            for name = {'wc_rad_s','kj','ka','kv','max_abs_err_rad','spec_ok','e_t'}
                testCase.verifyTrue(isfield(c.A, name{1}));
                testCase.verifyTrue(isfield(c.B, name{1}));
            end
        end

        function designChoicesArePassedThrough(testCase)
            c = testCase.controller;
            testCase.verifyEqual(c.beta, testCase.Beta);
            testCase.verifyEqual(c.zeta, testCase.sizing.zeta, 'RelTol', 1e-12);
            testCase.verifyEqual(c.w1_rad_s, testCase.sizing.wn_rad_s, 'RelTol', 1e-12);
            testCase.verifyEqual(c.emax_rad, testCase.params.spec.angular_accuracy_rad, ...
                'RelTol', 1e-12);
            testCase.verifyEqual(c.t, testCase.ref.t);
        end

        function phaseMarginIsDerivedFromTheDampingRatio(testCase)
            testCase.verifyEqual(testCase.controller.PM_deg, ...
                testCase.sizing.zeta*100, 'RelTol', 1e-12);
        end

        function leadRatioFollowsEquation129(testCase)
            c = testCase.controller;
            expectedAlpha = (1 - sind(45 - c.PM_deg)) / (1 + sind(45 - c.PM_deg));
            testCase.verifyEqual(c.alpha, expectedAlpha, 'RelTol', 1e-12);
            % alpha is a lead ratio and must stay a physical fraction.
            testCase.verifyGreaterThan(c.alpha, 0);
            testCase.verifyLessThan(c.alpha, 1);
        end

        function weedingVelocityUsesTheAngleToPositionRatio(testCase)
            testCase.verifyEqual(testCase.controller.v_weeding_rad_s, ...
                testCase.params.spec.driving_speed_nom_mps / testCase.ref.a2p, ...
                'RelTol', 1e-12);
        end

        function approachACrossoverFollowsTheVelocityErrorTerm(testCase)
            c = testCase.controller;
            expected = (c.w1_rad_s^2 * c.beta * (1 - testCase.Gamma) * ...
                c.v_weeding_rad_s / (c.alpha * c.emax_rad))^(1/3);
            testCase.verifyEqual(c.A.wc_rad_s, expected, 'RelTol', 1e-12);
        end

        function approachBCrossoverFollowsTheWorstCaseSum(testCase)
            c = testCase.controller;
            g = testCase.Gamma;
            rhs = (1-g)*testCase.ref.dddtheta_max + ...
                (1-g)*2*c.zeta*c.w1_rad_s*testCase.ref.ddtheta_max + ...
                (1-g)*c.w1_rad_s^2*testCase.ref.dtheta_max;
            expected = (c.beta * rhs / (c.alpha * c.emax_rad))^(1/3);
            testCase.verifyEqual(c.B.wc_rad_s, expected, 'RelTol', 1e-12);
        end

        function worstCaseApproachAsksForAHigherCrossover(testCase)
            % Approach B also covers the fast return transient, so it can never
            % need a lower bandwidth than the weeding-only approach A.
            c = testCase.controller;
            testCase.verifyGreaterThan(c.B.wc_rad_s, c.A.wc_rad_s);
        end

        function errorConstantsFollowTheLeadCompensatorRelations(testCase)
            c = testCase.controller;
            for name = {'A','B'}
                approach = c.(name{1});
                testCase.verifyEqual(approach.kj, ...
                    c.beta/(c.alpha*approach.wc_rad_s^3), 'RelTol', 1e-12);
                testCase.verifyEqual(approach.ka, ...
                    2*c.zeta*c.w1_rad_s*approach.kj, 'RelTol', 1e-12);
                testCase.verifyEqual(approach.kv, ...
                    c.w1_rad_s^2*approach.kj, 'RelTol', 1e-12);
            end
        end

        function approachAExactlySpendsTheErrorBudgetDuringWeeding(testCase)
            % wc_A is solved from kv*(1-g3)*v_weeding == emax, so the steady
            % weeding-phase error must land exactly on the spec.
            c = testCase.controller;
            testCase.verifyEqual(c.A.kv*(1 - testCase.Gamma)*c.v_weeding_rad_s, ...
                c.emax_rad, 'RelTol', 1e-9);
        end

        function errorTraceIsTheWeightedSumOfReferenceDerivatives(testCase)
            c = testCase.controller;
            g = testCase.Gamma;
            expected = c.A.kj*(1-g)*testCase.ref.dddr + ...
                c.A.ka*(1-g)*testCase.ref.ddr + ...
                c.A.kv*(1-g)*testCase.ref.dr;
            testCase.verifyEqual(c.A.e_t, expected, 'RelTol', 1e-12);
            testCase.verifySize(c.A.e_t, size(testCase.ref.t));
        end

        function maxAbsErrorAndSpecFlagAreConsistent(testCase)
            c = testCase.controller;
            for name = {'A','B'}
                approach = c.(name{1});
                testCase.verifyEqual(approach.max_abs_err_rad, ...
                    max(abs(approach.e_t)), 'RelTol', 1e-12);
                testCase.verifyEqual(approach.spec_ok, ...
                    approach.max_abs_err_rad <= c.emax_rad);
            end
        end

        function higherBandwidthApproachTracksTheReferenceBetter(testCase)
            % Approach B has the larger crossover, hence smaller error constants
            % and a smaller worst-case tracking error over the full reference.
            c = testCase.controller;
            testCase.verifyLessThan(c.B.kj, c.A.kj);
            testCase.verifyLessThan(c.B.max_abs_err_rad, c.A.max_abs_err_rad);
        end

        function zeroReferenceMotionGivesZeroTrackingError(testCase)
            zeroRef = ControllerDesignTest.makeRef();
            zeroRef.dr = zeros(size(zeroRef.t));
            zeroRef.ddr = zeros(size(zeroRef.t));
            zeroRef.dddr = zeros(size(zeroRef.t));
            c = controller_design(testCase.params, testCase.sizing, zeroRef);
            testCase.verifyEqual(c.A.max_abs_err_rad, 0);
            testCase.verifyTrue(c.A.spec_ok);
            testCase.verifyTrue(c.B.spec_ok);
        end

    end
end
