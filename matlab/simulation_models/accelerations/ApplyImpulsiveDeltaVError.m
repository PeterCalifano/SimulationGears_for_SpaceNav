function dDV_WithError = ApplyImpulsiveDeltaVError(dNominalDV, ...
    dSigmaMagnitudeFrac, dSigmaDirectionInRad) %#codegen
%% SIGNATURE
% dDV_WithError = ApplyImpulsiveDeltaVError(dNominalDV, ...
%     dSigmaMagnitudeFrac, dSigmaDirectionInRad)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply independent Gaussian magnitude and small-angle direction errors to a
% nominal impulse. Draw a fractional parallel error and an angular error about
% a uniformly distributed perpendicular axis. Retain the existing first-order
% model; the angular sigma must not depend on the impulse magnitude or units.
% Return impulses below machine epsilon unchanged. Draw from the current RNG state.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dNominalDV              (3, 1) Nominal impulse in the caller's velocity units.
% dSigmaMagnitudeFrac     (1, 1) Fractional magnitude standard deviation [-].
% dSigmaDirectionInRad    (1, 1) Angular standard deviation [rad].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDV_WithError           (3, 1) Realized impulse in the input velocity units.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-12-2025    Pietro Califano     Implement first version.
% 27-09-2026    Pietro Califano, Codex gpt-6  Correct angular dispersion scaling.
% 29-09-2026    Pietro Califano, Codex gpt-6  Consolidate the generic impulse contract.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB RNG, cross.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dNominalDV (3, 1) double {mustBeReal}
    dSigmaMagnitudeFrac (1, 1) double {mustBeNonnegative}
    dSigmaDirectionInRad (1, 1) double {mustBeNonnegative}
end

arguments (Output)
    dDV_WithError (3, 1) double
end

% Preserve the existing negligible-impulse guard before normalizing.
dNominalMagnitude = norm(dNominalDV);
if abs(dNominalMagnitude) < eps
    dDV_WithError = dNominalDV;
    return
end

dUnitImpulse = dNominalDV / dNominalMagnitude;

% Draw the fractional magnitude error along the nominal direction.
dMagnitudeError = dSigmaMagnitudeFrac * dNominalMagnitude * randn(1, 1);
dParallelError = dMagnitudeError * dUnitImpulse;

% Select a random perpendicular axis, with a deterministic degenerate fallback.
if dSigmaDirectionInRad > 0
    dAxisTmp = randn(3, 1);
    dAxisTmp = dAxisTmp - (transpose(dAxisTmp) * dUnitImpulse) * dUnitImpulse;
    dAxisNorm = norm(dAxisTmp);

    if dAxisNorm < 1e-12
        if abs(dUnitImpulse(1)) < 0.9
            dAxisTmp = [1; 0; 0] - dUnitImpulse(1) * dUnitImpulse;
        else
            dAxisTmp = [0; 1; 0] - dUnitImpulse(2) * dUnitImpulse;
        end
        dAxisTmp = dAxisTmp / norm(dAxisTmp);
    else
        dAxisTmp = dAxisTmp / dAxisNorm;
    end

    % Apply the first-order angular error once; cross already scales the impulse.
    dAngularError = dSigmaDirectionInRad * randn(1, 1);
    dTangentialError = dAngularError * cross(dAxisTmp, dNominalDV);
else
    dTangentialError = zeros(3, 1);
end

% Combine independent parallel and tangential errors in the input velocity units.
dDV_WithError = dNominalDV + dParallelError + dTangentialError;

end
