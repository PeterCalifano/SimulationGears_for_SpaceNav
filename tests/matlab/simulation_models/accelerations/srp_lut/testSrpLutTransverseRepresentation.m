function strVerification = testSrpLutTransverseRepresentation()
%% SIGNATURE
% strVerification = testSrpLutTransverseRepresentation()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that radial response cannot create transverse force off the grid.
% Compare reconstructed force and transverse response against independent
% panel evaluations on the same directions for three grid resolutions.
% Example: strVerification = testSrpLutTransverseRepresentation();
% Output: Passing radial, storage and refinement checks with response errors in m^2.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification   Radial error and RMS/maximum panel-response discrepancies.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Verify nodal transverse representation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpLutTestFixture, GenerateSrpLutDirections, ComputeQuadsModelSRP,
% PackSrpResponseLut, EvaluateSrpResponseLut, EvalJac_SrpResponseLut.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

arguments (Output)
    strVerification (1, 1) struct
end

% Exercise constant and direction-dependent radial laws at independent directions.
[strRadial, strHost] = BuildSrpLutTestFixture(true);
dDirections = GenerateSrpLutDirections(uint32(256));
dMaxRadialError = 0;
for bVaryCoefficient = [false, true]
    if bVaryCoefficient
        [dAzimuth, dElevation] = meshgrid(strHost.dAzimuth, strHost.dElevation);
        strHost.dEffectiveCr = 2 + 0.3 * cosd(dElevation).*cosd(dAzimuth);
        strRadial = PackSrpResponseLut(strHost, ui32Capacity=uint32([73, 37]), ...
                                     bIncludeTransverse=true);
    end
    strScalar = PackSrpResponseLut(strHost, ui32Capacity=uint32([73, 37]));
    assert(~isfield(strScalar, 'dTransverseForcePerPressure'));
    for ui32Query = uint32(1):uint32(size(dDirections, 2))
        dQuery = dDirections(:, ui32Query);
        [dScalarForce, dScalarCr] = EvaluateSrpResponseLut(dQuery, strScalar);
        [dForce, dCr, dTransverse] = EvaluateSrpResponseLut(dQuery, strRadial, true);
        [dJac, ~, dJacTransverse] = EvalJac_SrpResponseLut(dQuery, strRadial, true);
        dMaxRadialError = max(dMaxRadialError, norm(dTransverse));
        assert(isequal(dForce, dScalarForce) && dCr == dScalarCr);
        assert(norm(dTransverse) < 1e-13 && norm(dJacTransverse, 'fro') < 1e-13);
        assert(norm(dJac - EvalJac_SrpResponseLut(dQuery, strScalar), 'fro') < 1e-13);
    end
end

% Compare panel and table responses on fixed off-grid queries across refinement.
dGridSteps = [10, 5, 2];
dRmsForceError = zeros(size(dGridSteps));
dMaxForceError = zeros(size(dGridSteps));
dRmsTransverseError = zeros(size(dGridSteps));
dMaxTransverseError = zeros(size(dGridSteps));
for ui32Grid = uint32(1):uint32(numel(dGridSteps))
    [strLut, strPanelHost, strPanel] = BuildSrpLutTestFixture(false, true, dGridSteps(ui32Grid));
    dForceErrors = zeros(1, size(dDirections, 2));
    dTransverseErrors = zeros(size(dForceErrors));
    for ui32Query = uint32(1):uint32(size(dDirections, 2))
        dSunDir_SCB = dDirections(:, ui32Query);
        dDirectForce = ComputeQuadsModelSRP(dSunDir_SCB, [1;0;0;0], 1, zeros(3, 1), 1, ...
            strPanel.dSCquadsArea, strPanel.dDiffSpecQuadsCoeffs, ...
            strPanel.dQuadsNormals_SCB, strPanel.dQuadsPressCentre_SCB);
        dDirectTransverse = dDirectForce - dSunDir_SCB * dot(dSunDir_SCB, dDirectForce);
        [dForce, ~, dTransverse] = EvaluateSrpResponseLut(dSunDir_SCB, strLut, true);
        dForceErrors(ui32Query) = norm(dForce - dDirectForce);
        dTransverseErrors(ui32Query) = norm(dTransverse - dDirectTransverse);
        assert(abs(dot(dSunDir_SCB, dTransverse)) < 1e-12);
    end
    dRmsForceError(ui32Grid) = sqrt(mean(dForceErrors.^2));
    dMaxForceError(ui32Grid) = max(dForceErrors);
    dRmsTransverseError(ui32Grid) = sqrt(mean(dTransverseErrors.^2));
    dMaxTransverseError(ui32Grid) = max(dTransverseErrors);

    % Recover interior node forces from their independent scalar/transverse parts.
    for ui32Elevation = uint32(2):uint32(numel(strPanelHost.dElevation) - 1)
        dElevation = strPanelHost.dElevation(ui32Elevation);
        ui32Azimuth = uint32(3);
        dAzimuth = strPanelHost.dAzimuth(ui32Azimuth);
        dQuery = [cosd(dElevation)*cosd(dAzimuth); cosd(dElevation)*sind(dAzimuth); sind(dElevation)];
        dExpected = strPanelHost.dForcePerPressure(:, ui32Elevation, ui32Azimuth);
        assert(norm(EvaluateSrpResponseLut(dQuery, strLut, true) - dExpected) < 1e-12);
    end
end
assert(all(diff(dRmsForceError) < 0) && all(diff(dRmsTransverseError) < 0));

strVerification = struct('bPassed', true, 'dMaxRadialError', dMaxRadialError, ...
    'dGridSteps', dGridSteps, 'dRmsForceError', dRmsForceError, 'dMaxForceError', dMaxForceError, ...
    'dRmsTransverseError', dRmsTransverseError, 'dMaxTransverseError', dMaxTransverseError);
disp(strVerification);
end
