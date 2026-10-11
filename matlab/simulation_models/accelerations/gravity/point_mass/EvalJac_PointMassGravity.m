function dAccelerationJacobian = EvalJac_PointMassGravity(dPositionFromBody, dBodyGM) %#codegen
%% SIGNATURE
% dAccelerationJacobian = EvalJac_PointMassGravity(dPositionFromBody, dBodyGM)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the position Jacobian of ComputePointMassGravity. All vectors are expressed in one caller-selected
% frame and the derivative is with respect to dPositionFromBody:
%   da/dr = GM * (3*r*r'/||r||^5 - I/||r||^3).
% Length and time units are arbitrary but must be consistent between position and GM.
% Example: EvalJac_PointMassGravity([2; 0; 0], 4) returns diag([1, -0.5, -0.5]).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPositionFromBody     (3,1) double   Position from the attracting body to the evaluation point [LU].
% dBodyGM               (1,1) double   Gravitational parameter [LU^3/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccelerationJacobian (3,3) double   Acceleration derivative with respect to position [1/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 15-09-2026    Pietro Califano, Codex GPT-6     Extract reusable fixed-size point-mass derivative kernel.
% 11-10-2026    Pietro Califano, Codex GPT-6     Consolidate the shared gravity interface.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dPositionFromBody     (3,1) double {mustBeFinite}
    dBodyGM               (1,1) double {mustBeFinite, mustBeNonnegative}
end

arguments (Output)
    dAccelerationJacobian (3,3) double
end

% Reject singular geometry before evaluating inverse-distance powers.
dDistanceSquared = dot(dPositionFromBody, dPositionFromBody);
assert(dDistanceSquared > 0.0, ...
    'EvalJac_PointMassGravity:ZeroDistance', ...
    'The evaluation point must not coincide with the attracting body.');

dInverseDistance = 1.0 / sqrt(dDistanceSquared);
dInverseDistanceCubed = dInverseDistance / dDistanceSquared;
dInverseDistanceFifth = dInverseDistanceCubed / dDistanceSquared;
dAccelerationJacobian = dBodyGM * ( ...
    3.0 * dInverseDistanceFifth * (dPositionFromBody * transpose(dPositionFromBody)) - ...
    dInverseDistanceCubed * eye(3));

end
