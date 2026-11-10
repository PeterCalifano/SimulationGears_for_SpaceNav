function dAcceleration = ComputePointMassGravity(dPositionFromBody, dBodyGM) %#codegen
%% SIGNATURE
% dAcceleration = ComputePointMassGravity(dPositionFromBody, dBodyGM)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute point-mass gravity at a position measured from the attracting body. All vectors are expressed in one
% caller-selected frame. Length and time units are arbitrary but must be consistent between position and GM.
% The acceleration points toward the body:
%   a = -GM * r / ||r||^3.
% Example: ComputePointMassGravity([2; 0; 0], 4) returns [-1; 0; 0].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPositionFromBody   (3,1) double   Position from the attracting body to the evaluation point [LU].
% dBodyGM             (1,1) double   Gravitational parameter [LU^3/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAcceleration       (3,1) double   Point-mass acceleration [LU/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 15-09-2026    Pietro Califano, Codex     Extract reusable fixed-size point-mass acceleration kernel.
% 11-10-2026    Pietro Califano          Consolidate the shared gravity interface.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dPositionFromBody (3,1) double {mustBeFinite}
    dBodyGM           (1,1) double {mustBeFinite, mustBeNonnegative}
end

arguments (Output)
    dAcceleration     (3,1) double
end

% Reject singular geometry before evaluating inverse-distance powers.
dDistanceSquared = dot(dPositionFromBody, dPositionFromBody);
assert(dDistanceSquared > 0.0, ...
    'ComputePointMassGravity:ZeroDistance', ...
    'The evaluation point must not coincide with the attracting body.');

dInverseDistanceCubed = 1.0 / (dDistanceSquared * sqrt(dDistanceSquared));
dAcceleration = -dBodyGM * dInverseDistanceCubed * dPositionFromBody;

end
