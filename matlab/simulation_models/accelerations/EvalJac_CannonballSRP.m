function dJacAccSRP_IN = EvalJac_CannonballSRP(dPosSunToSC_IN, ...
                                                dCoeffSRP, ...
                                                bIsInEclipse, ...
                                                bRecomputePressureFromDistance, ...
                                                dCachedDistSunToSC, ...
                                                bCachedIsSRPActive) %#codegen
arguments (Input)
    dPosSunToSC_IN                  (3,1) double {mustBeFinite}
    dCoeffSRP                       (1,1) double {mustBeFinite, mustBeNonnegative}
    bIsInEclipse                    (1,1) logical = false
    bRecomputePressureFromDistance  (1,1) logical {coder.mustBeConst} = false
    dCachedDistSunToSC              (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0
    bCachedIsSRPActive              (1,1) logical = false
end
arguments (Output)
    dJacAccSRP_IN (3,3) double
end
%% PROTOTYPE
% dJacAccSRP_IN = EvalJac_CannonballSRP(dPosSunToSC_IN, dCoeffSRP, bIsInEclipse, ...
%     bRecomputePressureFromDistance, dCachedDistSunToSC, bCachedIsSRPActive) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical position Jacobian of the cannonball SRP acceleration returned by ComputeCannonballSRP.
% The Jacobian is with respect to spacecraft inertial position, assuming Sun position is fixed during the RHS
% evaluation. Eclipse is modeled as a hard switch, so the Jacobian is zero while bIsInEclipse is true.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSunToSC_IN                 (3,1) double   Sun-to-spacecraft vector in inertial frame [LU]
% dCoeffSRP                      (1,1) double   SRP coefficient, mode-dependent [LU/TU^2] or [LU^3/TU^2]
% bIsInEclipse                   (1,1) logical  If true, SRP Jacobian is zero
% bRecomputePressureFromDistance (1,1) logical  If true, include inverse-square pressure variation
% dCachedDistSunToSC             (1,1) double   Optional precomputed Sun-spacecraft distance [LU]
% bCachedIsSRPActive             (1,1) logical  Optional activity flag returned by ComputeCannonballSRP
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacAccSRP_IN (3,3) double   d(a_SRP)/d(r_SC) in inertial coordinates [1/TU^2]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5      Added standalone cannonball SRP acceleration Jacobian.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
dJacAccSRP_IN = zeros(3, 3);

if dCachedDistSunToSC > 0.0
    % Use cached distance and activity status if provided, otherwise compute from inputs
    dDistSunToSC = dCachedDistSunToSC;
    % Define bIsSRPActive based on cached value if provided, otherwise compute from inputs
    if nargin >= 6
        bIsSRPActive = bCachedIsSRPActive;
    else
        bIsSRPActive = ~bIsInEclipse && dCoeffSRP > 0.0;
    end

else
    % Compute distance and activity status from inputs
    dDistSunToSC = norm(dPosSunToSC_IN);
    bIsSRPActive = ~bIsInEclipse && dCoeffSRP > 0.0;
end

assert(dDistSunToSC > 0.0, ...
    'EvalJac_CannonballSRP:ZeroDistance', ...
    'Sun-spacecraft distance must be positive.');

if ~bIsSRPActive
    % Return zero Jacobian when SRP is inactive (eclipse or zero coefficient)
    return;
end

% Compute Jacobian of cannonball SRP acceleration with respect to spacecraft position
dSunLineOuter = dPosSunToSC_IN * dPosSunToSC_IN.';
dInvDist = 1.0 / dDistSunToSC;

if coder.const(bRecomputePressureFromDistance)
    % Inverse-square pressure mode: a_SRP = -dCoeffSRP * dSunLine / dDist^3, so Jacobian includes both 1/d^3 and 3*dSunLine/d^5 terms
    dInvDist3 = dInvDist^3;
    dInvDist5 = dInvDist3 * dInvDist^2;
    dJacAccSRP_IN(:,:) = dCoeffSRP * (dInvDist3 * eye(3) - 3.0 * dInvDist5 * dSunLineOuter);
else
    % Fixed-pressure mode: a_SRP = -dCoeffSRP * dSunLine / dDist, so Jacobian includes both 1/d and dSunLine/d^3 terms
    dInvDist3 = dInvDist^3;
    dJacAccSRP_IN(:,:) = dCoeffSRP * (dInvDist * eye(3) - dInvDist3 * dSunLineOuter);
end

end
