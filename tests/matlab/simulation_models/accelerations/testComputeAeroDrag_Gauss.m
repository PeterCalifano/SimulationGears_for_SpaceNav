classdef testComputeAeroDrag_Gauss < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for ComputeAeroDrag_Gauss (atmospheric drag acceleration in RSW frame).
    % Tests output dimensions, physical direction, altitude dependence, and boundary conditions.
    % NOTE: Requires kepl2rv on the path.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth     = 398600.4418;
        dRadiusEarth = 6378.137;
    end

    methods (Test)

        function testOutputDimension(testCase)
            dResult = ComputeAeroDrag_Gauss(6800, 0.001, deg2rad(51.6), deg2rad(30), ...
                                     deg2rad(60), deg2rad(30), testCase.dMuEarth, 0.01, testCase.dRadiusEarth);
            testCase.verifySize(dResult, [3, 1]);
        end

        function testZeroBcoeffGivesZeroDrag(testCase)
            % With zero ballistic coefficient, drag must vanish
            dResult = ComputeAeroDrag_Gauss(6800, 0.001, deg2rad(51.6), deg2rad(30), ...
                                     deg2rad(60), deg2rad(30), testCase.dMuEarth, 0, testCase.dRadiusEarth);
            testCase.verifyEqual(dResult, zeros(3, 1), 'AbsTol', 1e-20, ...
                'Zero ballistic coefficient must give zero drag');
        end

        function testDragDecreasesWithAltitude(testCase)
            % Higher altitude should produce less drag (exponential atmosphere)
            dBcoeff = 0.01;
            dIncl = deg2rad(51.6);
            dRaan = deg2rad(30);
            dArgP = deg2rad(60);
            dTA   = deg2rad(30);

            % Low orbit (altitude ~422 km)
            dAccLow = ComputeAeroDrag_Gauss(6800, 0.001, dIncl, dRaan, dArgP, dTA, ...
                                      testCase.dMuEarth, dBcoeff, testCase.dRadiusEarth);

            % Higher orbit (altitude ~1622 km)
            dAccHigh = ComputeAeroDrag_Gauss(8000, 0.001, dIncl, dRaan, dArgP, dTA, ...
                                       testCase.dMuEarth, dBcoeff, testCase.dRadiusEarth);

            testCase.verifyGreaterThan(norm(dAccLow), norm(dAccHigh), ...
                'Lower altitude must produce higher drag acceleration');
        end

        function testDragScalesWithBcoeff(testCase)
            % Drag should scale linearly with ballistic coefficient
            dIncl = deg2rad(51.6);
            dRaan = deg2rad(30);
            dArgP = deg2rad(60);
            dTA   = deg2rad(30);

            dAcc1 = ComputeAeroDrag_Gauss(6800, 0.001, dIncl, dRaan, dArgP, dTA, ...
                                    testCase.dMuEarth, 0.01, testCase.dRadiusEarth);
            dAcc2 = ComputeAeroDrag_Gauss(6800, 0.001, dIncl, dRaan, dArgP, dTA, ...
                                    testCase.dMuEarth, 0.02, testCase.dRadiusEarth);

            testCase.verifyEqual(norm(dAcc2) / norm(dAcc1), 2, 'RelTol', 1e-10, ...
                'Drag should scale linearly with ballistic coefficient');
        end

        function testBelowAtmosphereReturnsZero(testCase)
            % For altitudes below 70 km, no atmosphere data exists --> zero drag
            % This requires an orbit with r < R_E + 70 km, i.e., suborbital
            % We test at altitude ~50 km (SMA ~ 6428 km, circular)
            dResult = ComputeAeroDrag_Gauss(6428, 0.0, deg2rad(45), 0, 0, 0, ...
                                      testCase.dMuEarth, 0.01, testCase.dRadiusEarth);
            testCase.verifyEqual(dResult, zeros(3, 1), 'AbsTol', 1e-20, ...
                'Below atmosphere model range should return zero drag');
        end

        function testExtrapolatedTailRemainsActiveBelow1300Km(testCase)
            dAltitude = 1250;

            dResult = ComputeAeroDrag_Gauss(testCase.dRadiusEarth + dAltitude, 0.0, ...
                                      deg2rad(45), 0, 0, 0, testCase.dMuEarth, ...
                                      0.01, testCase.dRadiusEarth);

            testCase.verifyGreaterThan(norm(dResult), 0, ...
                'Extrapolated atmosphere tail should remain active below 1300 km');
        end

        function testAbove1300KmReturnsZero(testCase)
            dAltitude = 1350;

            dResult = ComputeAeroDrag_Gauss(testCase.dRadiusEarth + dAltitude, 0.0, ...
                                      deg2rad(45), 0, 0, 0, testCase.dMuEarth, ...
                                      0.01, testCase.dRadiusEarth);

            testCase.verifyEqual(dResult, zeros(3, 1), 'AbsTol', 1e-20, ...
                'Above 1300 km the drag model must return zero');
        end

    end
end
