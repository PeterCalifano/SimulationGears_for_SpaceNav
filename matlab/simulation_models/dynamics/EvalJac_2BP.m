function dJacobian = EvalJac_2BP(dState, dGravParam) %#codegen
arguments (Input)
    dState      (:,1) double {mustBeNumeric}
    dGravParam  (1,1) double {mustBeNumeric, mustBePositive}
end
arguments (Output)
    dJacobian   (6,6) double
end
%% PROTOTYPE
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
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

dPosX = dState(1);
dPosY = dState(2);
dPosZ = dState(3);

dRadiusSq = dPosX*dPosX + dPosY*dPosY + dPosZ*dPosZ;
dInvRadius = 1.0 / sqrt(dRadiusSq);
dInvRadius3 = dInvRadius / dRadiusSq;
dInvRadius5 = dInvRadius3 / dRadiusSq;

% Gravity-gradient block: d/dr(-mu * r / r^3) = -mu/r^3 * I + 3*mu/r^5 * r*r'
dGravOverRadius3 = dGravParam * dInvRadius3;
dScale = 3.0 * dGravParam * dInvRadius5;

dUxx = dScale * dPosX * dPosX - dGravOverRadius3;
dUyy = dScale * dPosY * dPosY - dGravOverRadius3;
dUzz = dScale * dPosZ * dPosZ - dGravOverRadius3;
dUxy = dScale * dPosX * dPosY;
dUxz = dScale * dPosX * dPosZ;
dUyz = dScale * dPosY * dPosZ;

dJacobian = [0,    0,    0,    1,  0,  0;
             0,    0,    0,    0,  1,  0;
             0,    0,    0,    0,  0,  1;
             dUxx, dUxy, dUxz, 0,  0,  0;
             dUxy, dUyy, dUyz, 0,  0,  0;
             dUxz, dUyz, dUzz, 0,  0,  0];

end
