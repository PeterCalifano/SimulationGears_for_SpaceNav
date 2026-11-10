function dJacobian = EvalJac_2BP(dState, dGravParam) %#codegen
%% SIGNATURE
% dJacobian = EvalJac_2BP(dState, dGravParam)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical Jacobian of the Cartesian two-body problem equations of motion. Only the first 3 elements of
% dState (position) are used to compute the gravity-gradient block. The Jacobian has the structure:
%   A = [ 0  I ]
%       [ G  0 ]
% where G = d/dr(-mu * r / ||r||^3) is the 3x3 gravity-gradient matrix.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dState       (:,1) double   State vector (at least 3 elements; only x,y,z are used)
% dGravParam   (1,1) double   Central-body gravitational parameter [LU^3/TU^2]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacobian    (6,6) double   Jacobian matrix of the 2BP dynamics evaluated at dState
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-04-2026    Pietro Califano, Codex 5.4      Extracted reusable 2BP Jacobian from RHS_2BPwSTM.
% 11-10-2026    Pietro Califano                Reuse the shared point-mass derivative kernel.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalJac_PointMassGravity
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dState      (:,1) double {mustBeNumeric}
    dGravParam  (1,1) double {mustBeNumeric, mustBePositive}
end

arguments (Output)
    dJacobian   (6,6) double
end

% Assemble kinematics and the shared attractive gravity gradient.
dJacobian = zeros(6, 6);
dJacobian(1:3, 4:6) = eye(3);
dJacobian(4:6, 1:3) = EvalJac_PointMassGravity(dState(1:3), dGravParam);

end
