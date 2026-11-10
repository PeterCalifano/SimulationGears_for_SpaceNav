function bIsInEclipse = IsInCylindricalTargetShadow(dPosSC_IN, dSunPos_IN, dTargetRadius) %#codegen
arguments
    dPosSC_IN     (3,1) double {mustBeFinite}
    dSunPos_IN    (3,1) double {mustBeFinite}
    dTargetRadius (1,1) double {mustBeFinite}
end
%% PROTOTYPE
% bIsInEclipse = IsInCylindricalTargetShadow(dPosSC_IN, dSunPos_IN, dTargetRadius)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the hard cylindrical target-shadow predicate used by the inertial dynamics SRP models.
% The target is assumed to be centered at the inertial-frame origin, and the shadow axis is the anti-Sun
% direction. This is intentionally distinct from conical penumbra/umbra shadowing models.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN:       (3,1) double   Spacecraft position with respect to the target in inertial frame [LU].
% dSunPos_IN:      (3,1) double   Sun position with respect to the target in inertial frame [LU].
% dTargetRadius:   (1,1) double   Cylindrical shadow radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bIsInEclipse:    (1,1) logical  True when the spacecraft is behind the target and inside the cylinder.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract max-fidelity cylindrical SRP shadow predicate.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
bIsInEclipse = false;

if dTargetRadius <= 0.0 || ~any(abs(dSunPos_IN) > 0.0)
    return
end

dSunDir_IN = dSunPos_IN / norm(dSunPos_IN);
dProjectionOnAntiSun = dot(dPosSC_IN, -dSunDir_IN);

if dProjectionOnAntiSun <= 0.0
    return
end

dPerpFromShadowAxis = norm(dPosSC_IN + dProjectionOnAntiSun * dSunDir_IN);
bIsInEclipse = dPerpFromShadowAxis <= dTargetRadius;

end
