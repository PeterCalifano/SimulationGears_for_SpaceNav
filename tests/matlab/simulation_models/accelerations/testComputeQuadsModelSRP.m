classdef testComputeQuadsModelSRP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for ComputeQuadsModelSRP.
    % Cross-validates the quadrilateral SRP integration against the analytical spherical-shell model.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dSunDir_SCB = [1; 0; 0];
        dqIdentity  = [1; 0; 0; 0];
        dMassSC     = 120.0;
        dRadius     = 1.7;
        dSolarPressure = 4.56e-6;
        dRelTolSphericalShell = 1.2e-2;
        dTorqueTolScale = 1.5e-2;
        ui32NumMuBins = uint32(48);
        ui32NumAzBins = uint32(96);
    end

    methods (Test)

        function testSphericalShellCrossValidation(testCase)
            [dQuadsArea, dQuadNormals_SCB, dQuadPressCentre_SCB] = ...
                testCase.buildSphericalShellQuads(testCase.dRadius, testCase.ui32NumMuBins, testCase.ui32NumAzBins);

            dCoeffCases = [0.0, 0.0; ...
                           1.0, 0.0; ...
                           0.0, 1.0; ...
                           0.35, 0.40];

            for idCase = 1:size(dCoeffCases, 1)
                dDiffuseCoeff = dCoeffCases(idCase, 1);
                dSpecularCoeff = dCoeffCases(idCase, 2);
                dDiffSpecQuadsCoeffs = repmat([dDiffuseCoeff, dSpecularCoeff], numel(dQuadsArea), 1);

                [dAccelQuad_IN, dTorqueQuad_SCB, dCosSunPhaseAngle] = ComputeQuadsModelSRP( ...
                    testCase.dSunDir_SCB, ...
                    testCase.dqIdentity, ...
                    testCase.dMassSC, ...
                    zeros(3, 1), ...
                    testCase.dSolarPressure, ...
                    dQuadsArea, ...
                    dDiffSpecQuadsCoeffs, ...
                    dQuadNormals_SCB, ...
                    dQuadPressCentre_SCB);

                dAccelSphere_IN = testCase.computeSphericalShellAcceleration(dDiffuseCoeff);

                testCase.verifySize(dAccelQuad_IN, [3, 1]);
                testCase.verifySize(dTorqueQuad_SCB, [3, 1]);
                testCase.verifySize(dCosSunPhaseAngle, [numel(dQuadsArea), 1]);
                dRelAccelError = norm(dAccelQuad_IN - dAccelSphere_IN) / norm(dAccelSphere_IN);
                testCase.verifyLessThan(dRelAccelError, testCase.dRelTolSphericalShell, ...
                    sprintf('Quadrilateral SRP model must match the spherical-shell solution for case %d', idCase));
                testCase.verifyLessThan(norm(dAccelQuad_IN(2:3)), 1e-18, ...
                    sprintf('Spherical-shell acceleration must remain aligned with the Sun line for case %d', idCase));

                dReferenceTorque = testCase.dSolarPressure * pi * testCase.dRadius^3;
                testCase.verifyLessThan(norm(dTorqueQuad_SCB), testCase.dTorqueTolScale * dReferenceTorque, ...
                    sprintf('Net torque must vanish for a centered spherical shell (case %d)', idCase));
            end
        end

    end

    methods (Access = private)

        function dAccelSphere_IN = computeSphericalShellAcceleration(testCase, dDiffuseCoeff)
            dProjectedArea = pi * testCase.dRadius^2;
            dSphereCoeff = 1 + (4 / 9) * dDiffuseCoeff;
            dAccelSphere_IN = -(testCase.dSolarPressure * dProjectedArea * dSphereCoeff / testCase.dMassSC) * testCase.dSunDir_SCB;
        end

        function [dQuadsArea, dQuadNormals_SCB, dQuadPressCentre_SCB] = buildSphericalShellQuads(~, dRadius, ui32NumMuBins, ui32NumAzBins)
            dMuEdges = linspace(-1, 1, double(ui32NumMuBins) + 1);
            dAzEdges = linspace(0, 2 * pi, double(ui32NumAzBins) + 1);

            nQuads = double(ui32NumMuBins * ui32NumAzBins);
            dQuadsArea = zeros(nQuads, 1);
            dQuadNormals_SCB = zeros(3, nQuads);
            dQuadPressCentre_SCB = zeros(3, nQuads);

            dDeltaAz = dAzEdges(2) - dAzEdges(1);
            idQ = 1;
            for idMu = 1:double(ui32NumMuBins)
                dMuCenter = 0.5 * (dMuEdges(idMu) + dMuEdges(idMu + 1));
                dSinEl = sqrt(max(0, 1 - dMuCenter^2));

                for idAz = 1:double(ui32NumAzBins)
                    dAzCenter = 0.5 * (dAzEdges(idAz) + dAzEdges(idAz + 1));

                    dQuadNormals_SCB(:, idQ) = [dSinEl * cos(dAzCenter); ...
                                                dSinEl * sin(dAzCenter); ...
                                                dMuCenter];
                    dQuadPressCentre_SCB(:, idQ) = dRadius * dQuadNormals_SCB(:, idQ);
                    dQuadsArea(idQ) = dRadius^2 * dDeltaAz * (dMuEdges(idMu + 1) - dMuEdges(idMu));
                    idQ = idQ + 1;
                end
            end
        end

    end
end
