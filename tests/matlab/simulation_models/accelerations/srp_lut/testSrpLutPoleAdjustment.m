function strVerification = testSrpLutPoleAdjustment(strResponseLut)
%% SIGNATURE
% strVerification = testSrpLutPoleAdjustment(strResponseLut)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify deterministic pole lookup, its analytical derivative and Sun-line
% projection invariants. Keep finite perturbations inside the adjusted cap.
% Compare force changes with independent host interpolation of the unadjusted
% table; report the tiny artificial change without qualifying the original pole.
% Example: strVerification = testSrpLutPoleAdjustment(BuildSrpLutTestFixture());
% Output: Passing pole/near-pole checks and force/derivative discrepancies.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strResponseLut   Prepared immutable numeric response payload.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification  Query count, derivative discrepancy and force adjustment [m^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Verify deterministic pole regularization.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalJac_SrpResponseLut, EvaluateSrpResponseLut, interp2 (host-only oracle).
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strResponseLut (1, 1) struct
end

arguments (Output)
    strVerification (1, 1) struct
end

% Track the adjusted force derivative separately from its change to the raw LUT.
dMaxRelative = 0;
dMaxForceAdjustment = 0;
dMaxSwitchJump = 0;
ui32QueryCount = uint32(0);
for dPoleSign = [-1, 1]
    for dOffset = [zeros(2, 1), [1e-10;-2e-10], [-2e-10;1e-10]]
        dQuery = [dOffset;dPoleSign];
        for bTransverse = [false, true]
            [dJacobian, ~, ~, dForce, ~, dTransverse, bRegular] = ...
                EvalJac_SrpResponseLut(dQuery, strResponseLut, bTransverse);
            assert(~bRegular && all(isfinite(dJacobian), 'all') && all(isfinite(dForce)));
            dRepeated = EvaluateSrpResponseLut(dQuery, strResponseLut, bTransverse);
            assert(isequal(dRepeated, EvaluateSrpResponseLut(dQuery, strResponseLut, bTransverse)));
            assert(norm(dRepeated-dForce) < 1e-12);
            assert(abs(dQuery.'*dTransverse) < 1e-12);
            assert(norm(dJacobian*dQuery) < 1e-12);

            % Keep central perturbations inside the deterministic pole-adjustment cap.
            dNumerical = zeros(3, 3);
            dStep = 2e-11;
            for ui32Axis = uint32(1):uint32(3)
                dDelta = zeros(3, 1);
                dDelta(ui32Axis) = dStep;
                dNumerical(:, ui32Axis) = ( ...
                    EvaluateSrpResponseLut(dQuery+dDelta, strResponseLut, bTransverse)- ...
                    EvaluateSrpResponseLut(dQuery-dDelta, strResponseLut, bTransverse))/(2*dStep);
            end
            dRelative = norm(dJacobian-dNumerical, 'fro')/max(norm(dJacobian, 'fro'), 1e-7);
            dMaxRelative = max(dMaxRelative, dRelative);
            assert(dRelative < 5e-4, 'Pole derivative differs from the adjusted force model.');
            dUnadjusted = UnadjustedForce_(dQuery, strResponseLut, bTransverse);
            dMaxForceAdjustment = max(dMaxForceAdjustment, norm(dForce-dUnadjusted));
            ui32QueryCount = ui32QueryCount+1;
        end
    end

    % Characterize the selected branches immediately across the adjustment boundary.
    for bTransverse = [false, true]
        dInside = EvaluateSrpResponseLut([2.5e-9*(1-1e-6);0;dPoleSign], strResponseLut, bTransverse);
        dOutside = EvaluateSrpResponseLut([2.5e-9*(1+1e-6);0;dPoleSign], strResponseLut, bTransverse);
        dMaxSwitchJump = max(dMaxSwitchJump, norm(dInside-dOutside));
    end
end
strVerification = struct('bPassed', true, 'ui32QueryCount', ui32QueryCount, ...
    'dMaxRelativeJacobianError', dMaxRelative, 'dMaxForceAdjustment_m2', dMaxForceAdjustment, ...
    'dMaxSwitchJump_m2', dMaxSwitchJump, 'dTilt_rad', 1e-8, 'dCapRadius', 2.5e-9);
disp(strVerification);
end

function dForce = UnadjustedForce_(dQuery, strLut, bTransverse)
% Interpolate the raw table with the host library, without the pole adjustment.
arguments (Input)
    dQuery (3, 1) double
    strLut (1, 1) struct
    bTransverse (1, 1) logical
end

arguments (Output)
    dForce (3, 1) double
end

% Interpolate the raw scalar law with the host implementation.
dUnit = dQuery/norm(dQuery);
dAzimuth = atan2d(dUnit(2), dUnit(1));
dElevation = atan2d(dUnit(3), hypot(dUnit(1), dUnit(2)));
dAzimuthGrid = strLut.dAzimuth(1:strLut.ui32AzimuthCount);
dElevationGrid = strLut.dElevation(1:strLut.ui32ElevationCount);
dCoeff = interp2(dAzimuthGrid, dElevationGrid, ...
    strLut.dEffectiveCr(1:strLut.ui32ElevationCount, 1:strLut.ui32AzimuthCount), ...
    dAzimuth, dElevation);
dForce = -strLut.dReferenceArea_m2*dCoeff*dUnit;
if bTransverse
    % Project raw transverse interpolation without applying the algorithm's pole tilt.
    dVector = zeros(3, 1);
    for ui32Axis = uint32(1):uint32(3)
        dSlice = squeeze(strLut.dTransverseForcePerPressure(ui32Axis, ...
            1:strLut.ui32ElevationCount, 1:strLut.ui32AzimuthCount));
        dVector(ui32Axis) = interp2(dAzimuthGrid, dElevationGrid, dSlice, dAzimuth, dElevation);
    end
    dForce = dForce+(eye(3)-dUnit*dUnit.')*dVector;
end
end
