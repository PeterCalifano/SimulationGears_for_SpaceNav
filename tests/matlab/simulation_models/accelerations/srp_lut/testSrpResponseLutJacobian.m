function strVerification = testSrpResponseLutJacobian()
%% SIGNATURE
% strVerification = testSrpResponseLutJacobian()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify analytical scalar/vector derivatives with an independent constant-Cr
% oracle, central differences on regular cells, scaling identities and explicit
% knot/pole behavior. Exercise compact and legacy fixed-capacity payloads.
% Example: strVerification = testSrpResponseLutJacobian();
% Output: Passing derivative metrics and exercised direction/mode counts.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Run SetupSimGears and add this test directory first.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification   Numeric tolerances and maximum discrepancy evidence.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Verify actual LUT scalar/transverse derivatives.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpLutTestFixture, EvalJac_SrpResponseLut, EvaluateSrpResponseLut,
% GenerateSrpLutDirections, ValidateSrpResponseLut.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

arguments (Output)
    strVerification (1, 1) struct
end

% Recover the closed-form derivative of a constant radial response.
strConstant = BuildSrpLutTestFixture(true);
dQuery = [1.3;0.37;0.28];
dDirection = dQuery/norm(dQuery);
[dJac, dGradient, ~, dForce] = EvalJac_SrpResponseLut(dQuery, strConstant, false);
dExpected = -(eye(3)-dDirection*dDirection.')/norm(dQuery);
assert(norm(dForce+dDirection) < 1e-13 && norm(dJac-dExpected, 'fro') < 1e-13);
assert(norm(dGradient) < 1e-13);

% Validate the selected interpolation derivative independently of its finite differences.
strLut = BuildSrpLutTestFixture(false);
dDirections = GenerateSrpLutDirections(uint32(240));
dMaxRelative = 0;
dMaxCrError = 0;
dMaxTransverse = 0;
ui32RegularCount = uint32(0);
dStep = 1e-6;
for bTransverse = [false, true]
    for ui32Direction = uint32(1):uint32(size(dDirections, 2))
        dQuery = 2.7*dDirections(:, ui32Direction);
        [dJac, dCrGradient, dJacTransverse, dForce, dCr, dTransverse, bRegular] = ...
            EvalJac_SrpResponseLut(dQuery, strLut, bTransverse);
        if ~bRegular
            continue
        end
        ui32RegularCount = ui32RegularCount+1;
        [dSource, dSourceCr, dSourceTransverse] = EvaluateSrpResponseLut(dQuery, strLut, bTransverse);
        assert(norm(dForce-dSource) < 1e-13 && abs(dCr-dSourceCr) < 1e-13 && ...
            norm(dTransverse-dSourceTransverse) < 1e-13);

        % Differentiate the scalar and transverse outputs using the same query perturbations.
        dNumericalJacobian = zeros(3, 3);
        dNumericalCrGradient = zeros(1, 3);
        dNumericalTransverseJacobian = zeros(3, 3);
        for ui32Axis = uint32(1):uint32(3)
            dPerturbation = zeros(3, 1);
            dPerturbation(ui32Axis) = dStep;
            [dPositive, dPositiveCr, dPositiveTransverse] = ...
                EvaluateSrpResponseLut(dQuery+dPerturbation, strLut, bTransverse);
            [dNegative, dNegativeCr, dNegativeTransverse] = ...
                EvaluateSrpResponseLut(dQuery-dPerturbation, strLut, bTransverse);
            dNumericalJacobian(:, ui32Axis) = (dPositive-dNegative)/(2*dStep);
            dNumericalCrGradient(ui32Axis) = (dPositiveCr-dNegativeCr)/(2*dStep);
            dNumericalTransverseJacobian(:, ui32Axis) = (dPositiveTransverse-dNegativeTransverse)/(2*dStep);
        end

        % Check analytical partials and homogeneity in the displacement magnitude.
        dRelative = norm(dJac-dNumericalJacobian, 'fro')/max(norm(dNumericalJacobian, 'fro'), 1e-7);
        dMaxRelative = max(dMaxRelative, dRelative);
        dMaxCrError = max(dMaxCrError, norm(dCrGradient-dNumericalCrGradient));
        dMaxTransverse = max(dMaxTransverse, norm(dJacTransverse-dNumericalTransverseJacobian, 'fro'));
        assert(dRelative < 2e-7 && norm(dCrGradient-dNumericalCrGradient) < 2e-8 && ...
            norm(dJacTransverse-dNumericalTransverseJacobian, 'fro') < 2e-8);
        assert(norm(dJac*dQuery) < 1e-12);
        [dScaledJac, ~, ~, dScaledForce] = EvalJac_SrpResponseLut(10*dQuery, strLut, bTransverse);
        assert(norm(dScaledForce-dForce) < 1e-12 && norm(10*dScaledJac-dJac, 'fro') < 1e-11);
        if bTransverse
            dUnit = dQuery/norm(dQuery);
            dJacUnit = (eye(3)-dUnit*dUnit.')/norm(dQuery);
            assert(norm(dUnit.'*dJacTransverse+dTransverse.'*dJacUnit) < 1e-12);
        else
            assert(all(dJacTransverse == 0, 'all'));
        end
    end
end
assert(ui32RegularCount >= 450);

% Exercise boundary semantics without claiming a two-sided derivative at a kink.
for dBoundary = [eye(3), -eye(3)]
    [dJac, ~, ~, dForce, ~, ~, bRegular] = EvalJac_SrpResponseLut(dBoundary, strLut, true);
    assert(~bRegular && all(isfinite(dJac), 'all') && all(isfinite(dForce)));
end
strLegacy = strLut;
strLegacy.dAzimuth(74:361) = 0;
strLegacy.dElevation(38:181) = 0;
strLegacy.dEffectiveCr(181, 361) = 0;
strLegacy.dTransverseForcePerPressure(3, 181, 361) = 0;
ValidateSrpResponseLut(strLegacy);
assert(norm(EvaluateSrpResponseLut(dQuery, strLegacy, true)- ...
    EvaluateSrpResponseLut(dQuery, strLut, true)) < 1e-13);
strVerification = struct('bPassed', true, 'ui32RegularModeQueries', ui32RegularCount, ...
    'dMaxRelativeJacobianError', dMaxRelative, 'dMaxCrGradientError', dMaxCrError, ...
    'dMaxTransverseJacobianError', dMaxTransverse, 'bConstantOraclePassed', true, ...
    'bBoundarySemanticsPassed', true, 'bCompactLegacyParityPassed', true);
disp(strVerification);
end
