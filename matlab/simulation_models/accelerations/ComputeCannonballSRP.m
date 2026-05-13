function [dAccSRP_IN, dDistSunToSC, bIsSRPActive] = ComputeCannonballSRP(dPosSunToSC_IN, ...
                                                                        dCoeffSRP, ...
                                                                        bIsInEclipse) %#codegen
arguments (Input)
    dPosSunToSC_IN double {mustBeFinite}
    dCoeffSRP      double {mustBeFinite, mustBeNonnegative}
    bIsInEclipse   logical
end
arguments (Output)
    dAccSRP_IN   (3,1) double
    dDistSunToSC (1,1) double
    bIsSRPActive (1,1) logical
end
%% PROTOTYPE
% [dAccSRP_IN, dDistSunToSC, bIsSRPActive] = ComputeCannonballSRP(dPosSunToSC_IN, ...
%     dCoeffSRP, bIsInEclipse) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes cannonball solar-radiation-pressure acceleration in the inertial frame.
% dPosSunToSC_IN is the vector from the Sun to the spacecraft, so positive acceleration is away from the Sun.
%
% dCoeffSRP is the current-pressure coefficient:
%     dCoeffSRP = P_SRP * Cr * A / m
% where P_SRP has already been evaluated by the caller for the current Sun-spacecraft distance.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSunToSC_IN (3,1) double   Sun-to-spacecraft vector in inertial frame [LU]
% dCoeffSRP      (1,1) double   Current SRP acceleration coefficient [LU/TU^2]
% bIsInEclipse   (1,1) logical  If true, SRP acceleration is zero
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccSRP_IN    (3,1) double   Cannonball SRP acceleration in inertial frame [LU/TU^2]
% dDistSunToSC  (1,1) double   Sun-spacecraft distance [LU]
% bIsSRPActive  (1,1) logical  False when eclipsed or coefficient is zero
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5      Added standalone cannonball SRP acceleration kernel.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code

assert(isequal(size(dPosSunToSC_IN), [3, 1]), ...
    'ComputeCannonballSRP:InvalidSunLine', ...
    'Sun-spacecraft vector must be a nonempty 3x1 finite vector.');
assert(isscalar(dCoeffSRP), ...
    'ComputeCannonballSRP:InvalidCoefficient', ...
    'SRP coefficient must be a nonempty finite nonnegative scalar.');
assert(isscalar(bIsInEclipse), ...
    'ComputeCannonballSRP:InvalidEclipseFlag', ...
    'Eclipse flag must be a nonempty scalar logical.');

% Initialize outputs
dAccSRP_IN = zeros(3, 1);
dDistSunToSC = norm(dPosSunToSC_IN);
assert(dDistSunToSC > 0.0, ...
    'ComputeCannonballSRP:ZeroDistance', ...
    'Sun-spacecraft distance must be positive.');

% Determine if SRP is active based on eclipse status and coefficient.
bIsSRPActive = ~bIsInEclipse && dCoeffSRP > 0.0;

if ~bIsSRPActive
    return;
end

% Positive acceleration is away from the Sun because dPosSunToSC_IN points from Sun to spacecraft.
dAccSRP_IN(:) = dCoeffSRP * dPosSunToSC_IN / dDistSunToSC;

end
