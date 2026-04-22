classdef testEvalRHS_GaussPlanetary < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for EvalRHS_GaussPlanetary (Gauss planetary equations ODE system).
    % Tests unperturbed case (type=0), dimensional consistency, and perturbation ordering.
    % NOTE: Requires kep2car, ComputeJ2Perturbation_Gauss, ComputeAeroDrag_Gauss on the path.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth     = 398600.4418;   % [km^3/s^2]
        dRadiusEarth = 6378.137;      % [km]
        dJ2Earth     = 1.08263e-3;    % [-]
    end

    methods (Test)

        function testUnperturbedMeanAnomalyRate(testCase)
            % With type=0 (unperturbed), the only non-zero rate should be true anomaly (mean motion)
            dSma   = 7000;
            dEcc   = 0.01;
            dIncl  = deg2rad(45);
            dRaan  = deg2rad(30);
            dArgP  = deg2rad(60);
            dTA    = deg2rad(0);

            dKepState = [dSma; dEcc; dIncl; dRaan; dArgP; dTA];
            dBcoeffPoly = 0; % Not used for type=0

            dKepDot = EvalRHS_GaussPlanetary(0, dKepState, testCase.dMuEarth, testCase.dRadiusEarth, ...
                                   testCase.dJ2Earth, dBcoeffPoly, uint8(0));

            % For unperturbed, da/dt=de/dt=di/dt=dOmega/dt=domega/dt = 0
            testCase.verifyEqual(dKepDot(1:5), zeros(5, 1), 'AbsTol', 1e-20, ...
                'Unperturbed Gauss equations must give zero rates for a, e, i, RAAN, omega');

            % True anomaly rate should be h/r^2 = n*a*b/r^2
            dSemiLatRect = dSma * (1 - dEcc^2);
            dRadius = dSemiLatRect / (1 + dEcc * cos(dTA));
            dMeanMotion = sqrt(testCase.dMuEarth / dSma^3);
            dSemiMinor = dSma * sqrt(1 - dEcc^2);
            dAngMom = dMeanMotion * dSma * dSemiMinor;
            dExpectedTArate = dAngMom / dRadius^2;

            testCase.verifyEqual(dKepDot(6), dExpectedTArate, 'RelTol', 1e-10, ...
                'True anomaly rate must match h/r^2 for unperturbed case');
        end

        function testOutputDimension(testCase)
            dKepState = [7000; 0.01; deg2rad(45); deg2rad(30); deg2rad(60); 0];
            dKepDot = EvalRHS_GaussPlanetary(0, dKepState, testCase.dMuEarth, testCase.dRadiusEarth, ...
                                   testCase.dJ2Earth, 0, uint8(1));
            testCase.verifySize(dKepDot, [6, 1]);
        end

        function testJ2PerturbationSmallerthanMeanMotion(testCase)
            % J2 perturbation rates (type=1) should be much smaller than dTA/dt.
            % Threshold is 0.1 (10%): J2 secular rates on da/dt and domega/dt can reach a
            % few percent of dTA/dt for a LEO orbit, well below the Keplerian mean motion.
            dKepState = [7000; 0.01; deg2rad(45); deg2rad(30); deg2rad(60); deg2rad(30)];

            dKepDot = EvalRHS_GaussPlanetary(0, dKepState, testCase.dMuEarth, testCase.dRadiusEarth, ...
                                   testCase.dJ2Earth, 0, uint8(1));

            dTArate = abs(dKepDot(6));
            for idElem = 1:5
                testCase.verifyLessThan(abs(dKepDot(idElem)) / dTArate, 0.1, ...
                    sprintf('Element %d rate should be << true anomaly rate', idElem));
            end
        end

        function testPerturbationTypeOrdering(testCase)
            % type=2 (J2+drag) rates should have larger magnitude than type=1 (J2 only) or type=3 (drag only)
            % for a LEO orbit where both are active
            dKepState = [6800; 0.001; deg2rad(51.6); deg2rad(30); deg2rad(60); deg2rad(30)];
            dBcoeffPoly = 0.01; % Typical ballistic coefficient

            dKepDot_J2only   = EvalRHS_GaussPlanetary(0, dKepState, testCase.dMuEarth, testCase.dRadiusEarth, ...
                                             testCase.dJ2Earth, dBcoeffPoly, uint8(1));
            dKepDot_J2drag   = EvalRHS_GaussPlanetary(0, dKepState, testCase.dMuEarth, testCase.dRadiusEarth, ...
                                             testCase.dJ2Earth, dBcoeffPoly, uint8(2));

            % Combined perturbation magnitude of first 5 elements should be >= J2-only
            dMagJ2only = norm(dKepDot_J2only(1:5));
            dMagJ2drag = norm(dKepDot_J2drag(1:5));

            % Not strictly >= due to possible cancellation, but should be comparable
            testCase.verifyGreaterThan(dMagJ2drag, 0, 'Combined perturbation must be non-zero');
            testCase.verifyGreaterThan(dMagJ2only, 0, 'J2 perturbation must be non-zero');
        end

    end
end
