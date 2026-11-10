function dAcceleration = ComputeThirdBodyGravity(dSpacecraftPositionFromMain, ...
    dBodyPositionFromMain, dBodyGM) %#codegen
%% SIGNATURE
% dAcceleration = ComputeThirdBodyGravity(dSpacecraftPositionFromMain, dBodyPositionFromMain, dBodyGM)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute differential third-body gravity for a spacecraft state relative to a main body:
%   a = GM * ((s-r)/||s-r||^3 - s/||s||^3),
% where r is spacecraft-from-main position and s is third-body-from-main position. Both vectors must use the
% same frame, origin, length unit, and epoch. The implementation uses an algebraically equivalent formulation
% that retains the small tidal acceleration when ||r|| is much smaller than ||s||.
% Example: ComputeThirdBodyGravity([1; 0; 0], [2; 0; 0], 4) returns [3; 0; 0].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSpacecraftPositionFromMain (3,1) double   Spacecraft position relative to the main body [LU].
% dBodyPositionFromMain       (3,1) double   Third-body position relative to the main body [LU].
% dBodyGM                     (1,1) double   Third-body gravitational parameter [LU^3/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAcceleration               (3,1) double   Spacecraft-minus-main differential acceleration [LU/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 15-09-2026    Pietro Califano, Codex GPT-6     Add reusable cancellation-resistant differential-gravity kernel.
% 11-10-2026    Pietro Califano, Codex GPT-6     Consolidate the shared gravity interface.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dSpacecraftPositionFromMain (3,1) double {mustBeFinite}
    dBodyPositionFromMain       (3,1) double {mustBeFinite}
    dBodyGM                     (1,1) double {mustBeFinite, mustBeNonnegative}
end

arguments (Output)
    dAcceleration               (3,1) double
end

% Reject singular geometry before evaluating inverse-distance powers.
dPositionFromBody = dSpacecraftPositionFromMain - dBodyPositionFromMain;
dMainToBodyDistanceSquared = dot(dBodyPositionFromMain, dBodyPositionFromMain);
dBodyToSpacecraftDistanceSquared = dot(dPositionFromBody, dPositionFromBody);

assert(dMainToBodyDistanceSquared > 0.0, ...
    'ComputeThirdBodyGravity:ZeroMainBodyDistance', ...
    'The third body must not coincide with the main body.');
assert(dBodyToSpacecraftDistanceSquared > 0.0, ...
    'ComputeThirdBodyGravity:ZeroSpacecraftDistance', ...
    'The spacecraft must not coincide with the third body.');

% Evaluate f = (1+q)^(3/2)-1 through the factored difference of cubes.
% With q = (||r||^2 - 2*r'*s)/||s||^2, this avoids subtracting nearly equal
% unit values in far-field configurations.
dQ = dot(dSpacecraftPositionFromMain, ...
    dSpacecraftPositionFromMain - 2.0 * dBodyPositionFromMain) / ...
    dMainToBodyDistanceSquared;

dDistanceRatio = sqrt(dBodyToSpacecraftDistanceSquared / dMainToBodyDistanceSquared);
dDistanceRatioCubed = dDistanceRatio * dDistanceRatio * dDistanceRatio;
dF = dQ * (3.0 + 3.0 * dQ + dQ * dQ) / (1.0 + dDistanceRatioCubed);

dInverseBodyToSpacecraftCubed = 1.0 / ...
    (dBodyToSpacecraftDistanceSquared * sqrt(dBodyToSpacecraftDistanceSquared));
    
dAcceleration = -dBodyGM * dInverseBodyToSpacecraftCubed * ...
    (dSpacecraftPositionFromMain + dF * dBodyPositionFromMain);

end
