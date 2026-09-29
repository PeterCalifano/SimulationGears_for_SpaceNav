function dDV_WithError = ApplyImpulsiveDeltaVError(dNominalDV, ...
    dSigmaMagnitudeFrac, dSigmaDirectionInRad) %#codegen
%% SIGNATURE
% dDV_WithError = ApplyImpulsiveDeltaVError(dNominalDV, ...
%     dSigmaMagnitudeFrac, dSigmaDirectionInRad)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply independent Gaussian magnitude and direction errors to a nominal impulse.
% Rotate exactly about a uniformly distributed perpendicular axis, then apply
% the signed factor (1 + magnitude error). Preserve negative factors without
% clipping. Direction-only errors preserve impulse magnitude and velocity units.
% Interpret angular sigma as the standard deviation of the unwrapped signed
% Gaussian rotation angle. For direction-only errors, the principal pointing
% error wraps to [0, pi] for large draws.
% Return impulses below machine epsilon unchanged. Draw from the current RNG state.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dNominalDV              (3, 1) Nominal impulse in the caller's velocity units.
% dSigmaMagnitudeFrac     (1, 1) Fractional signed-scale error standard deviation [-].
% dSigmaDirectionInRad    (1, 1) Unwrapped signed-angle standard deviation [rad].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDV_WithError           (3, 1) Realized impulse in the input velocity units.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-12-2025    Pietro Califano     Implement first version.
% 27-09-2026    Pietro Califano, Codex gpt-6  Correct angular dispersion scaling.
% 29-09-2026    Pietro Califano, Codex gpt-6  Consolidate the generic impulse contract.
% 29-09-2026    Pietro Califano, Codex gpt-6  Extend direction errors to exact rotations.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RotationVectorToDCM, MATLAB RNG.
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
dMagnitudeScale = 1.0 + dSigmaMagnitudeFrac * randn(1, 1);

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

    % Apply the sampled angle as an active rotation, preserving impulse magnitude.
    dAngularError = dSigmaDirectionInRad * randn(1, 1);
    dErrorRotation = RotationVectorToDCM(dAngularError * dAxisTmp);
    dRotatedImpulse = dErrorRotation * dNominalDV;
else
    dRotatedImpulse = dNominalDV;
end

% Scale the complete rotated impulse in the caller's velocity units.
dDV_WithError = dMagnitudeScale * dRotatedImpulse;

end
