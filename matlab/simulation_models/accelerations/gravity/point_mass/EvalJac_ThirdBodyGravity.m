function dAccelerationJacobian = EvalJac_ThirdBodyGravity(dSpacecraftPositionFromMain, ...
    dBodyPositionFromMain, dBodyGM) %#codegen
%% SIGNATURE
% dAccelerationJacobian = EvalJac_ThirdBodyGravity(dSpacecraftPositionFromMain, dBodyPositionFromMain, dBodyGM)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the spacecraft-position Jacobian of ComputeThirdBodyGravity. The indirect main-body acceleration is
% constant with respect to spacecraft position, so the derivative is the point-mass gravity gradient evaluated
% at spacecraft-minus-third-body position. Both input positions must use the same frame, origin, units, and epoch.
% Example: EvalJac_ThirdBodyGravity([1; 0; 0], [2; 0; 0], 4) returns diag([8, -4, -4]).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSpacecraftPositionFromMain (3,1) double   Spacecraft position relative to the main body [LU].
% dBodyPositionFromMain       (3,1) double   Third-body position relative to the main body [LU].
% dBodyGM                     (1,1) double   Third-body gravitational parameter [LU^3/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccelerationJacobian       (3,3) double   Derivative with respect to spacecraft position [1/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 15-09-2026    Pietro Califano, Codex GPT-6   Add reusable fixed-size differential-gravity derivative kernel.
% 11-10-2026    Pietro Califano, Codex GPT-6   Consolidate the shared gravity interface.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalJac_PointMassGravity()
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dSpacecraftPositionFromMain (3,1) double {mustBeFinite}
    dBodyPositionFromMain       (3,1) double {mustBeFinite}
    dBodyGM                     (1,1) double {mustBeFinite, mustBeNonnegative}
end

arguments (Output)
    dAccelerationJacobian       (3,3) double
end

% Reject singular geometry before evaluating inverse-distance powers.
dMainToBodyDistanceSquared = dot(dBodyPositionFromMain, dBodyPositionFromMain);
dPositionFromBody = dSpacecraftPositionFromMain - dBodyPositionFromMain;
dBodyToSpacecraftDistanceSquared = dot(dPositionFromBody, dPositionFromBody);

assert(dMainToBodyDistanceSquared > 0.0, ...
    'EvalJac_ThirdBodyGravity:ZeroMainBodyDistance', ...
    'The third body must not coincide with the main body.');
assert(dBodyToSpacecraftDistanceSquared > 0.0, ...
    'EvalJac_ThirdBodyGravity:ZeroSpacecraftDistance', ...
    'The spacecraft must not coincide with the third body.');

dAccelerationJacobian = EvalJac_PointMassGravity(dPositionFromBody, dBodyGM);

end
