classdef testSimulateTargetFacetBrightness < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for facet-based target photometry.

    methods (TestClassSetup)

        function addSimulationGearsPath(~)
            charTestDir = fileparts(mfilename('fullpath'));
            charRepoRoot = fullfile(charTestDir, '..', '..', '..', '..');
            charMatlabRoot = fullfile(charRepoRoot, 'matlab');
            if isfolder(charMatlabRoot)
                addpath(genpath(charMatlabRoot));
            end
        end

    end

    methods (Test)

        function testLambertSingleTriangleKnownBrightness(testCase)
            ui32Faces = uint32([1; 2; 3]);
            dVertices_TB = [ 1.0,  0.0,  0.0; ...
                             0.0,  1.0,  0.0; ...
                            -1.0, -1.0,  0.0];
            dDCM_INfromTB = eye(3);
            dSunPosition_IN = [0.0; 0.0; 10.0];
            dObserverPosition_IN = [0.0; 0.0; 5.0];

            [dBrightness, strBrightnessData] = SimulateTargetFacetBrightness(ui32Faces, ...
                                                                            dVertices_TB, ...
                                                                            dDCM_INfromTB, ...
                                                                            dSunPosition_IN, ...
                                                                            dObserverPosition_IN, ...
                                                                            'charScatteringLaw', 'lambert');

            testCase.verifyEqual(dBrightness, 1.5, 'AbsTol', 1e-14);
            testCase.verifyEqual(strBrightnessData.ui32NumActiveFacets, uint32(1));
            testCase.verifyEqual(strBrightnessData.dCosIncidence, 1.0, 'AbsTol', 1e-14);
            testCase.verifyEqual(strBrightnessData.dCosEmission, 1.0, 'AbsTol', 1e-14);
        end

        function testBoresightSpinKeepsSymmetricBrightnessConstant(testCase)
            [ui32Faces, dVertices_TB] = testCase.buildBoxMesh([-1.0; -0.5; -0.4], ...
                                                              [ 1.0;  0.5;  0.4]);
            dSunPosition_IN = [0.0; 0.0; 100.0];
            dObserverPosition_IN = [0.0; 0.0; 10.0];
            dBrightnessHist = zeros(1, 17);

            for ui32Idx = uint32(1):uint32(numel(dBrightnessHist))
                dTheta = 2.0 * pi * double(ui32Idx - uint32(1)) / double(numel(dBrightnessHist) - 1);
                dBrightnessHist(ui32Idx) = SimulateTargetFacetBrightness(ui32Faces, ...
                                                                        dVertices_TB, ...
                                                                        testCase.rotZ(dTheta), ...
                                                                        dSunPosition_IN, ...
                                                                        dObserverPosition_IN, ...
                                                                        'charScatteringLaw', 'lommel_seeliger');
            end

            testCase.verifyLessThan(max(dBrightnessHist) - min(dBrightnessHist), 1e-12);
        end

        function testDegenerateTriangleRejected(testCase)
            ui32Faces = uint32([1; 2; 3]);
            dVertices_TB = [0.0, 1.0, 2.0; ...
                            0.0, 0.0, 0.0; ...
                            0.0, 0.0, 0.0];

            testCase.verifyError(@() SimulateTargetFacetBrightness(ui32Faces, ...
                                                                  dVertices_TB, ...
                                                                  eye(3), ...
                                                                  [0.0; 0.0; 10.0], ...
                                                                  [0.0; 0.0; 5.0]), ...
                                 'SimulateTargetFacetBrightness:DegenerateShapeModel');
        end

    end

    methods (Static, Access = private)

        function [ui32Faces, dVertices_TB] = buildBoxMesh(dMinCorner_TB, dMaxCorner_TB)
            dX0 = dMinCorner_TB(1);
            dY0 = dMinCorner_TB(2);
            dZ0 = dMinCorner_TB(3);
            dX1 = dMaxCorner_TB(1);
            dY1 = dMaxCorner_TB(2);
            dZ1 = dMaxCorner_TB(3);

            dVertices_TB = [dX0 dX1 dX1 dX0 dX0 dX1 dX1 dX0; ...
                            dY0 dY0 dY1 dY1 dY0 dY0 dY1 dY1; ...
                            dZ0 dZ0 dZ0 dZ0 dZ1 dZ1 dZ1 dZ1];

            ui32Faces = uint32([1 1 5 5 1 1 2 2 3 3 4 4; ...
                                2 3 6 7 5 6 6 7 7 8 8 5; ...
                                3 4 7 8 6 2 7 3 8 4 5 1]);
        end

        function dDCM = rotZ(dTheta)
            dCosTheta = cos(dTheta);
            dSinTheta = sin(dTheta);
            dDCM = [dCosTheta, -dSinTheta, 0.0; ...
                    dSinTheta,  dCosTheta, 0.0; ...
                    0.0,        0.0,       1.0];
        end

    end
end
