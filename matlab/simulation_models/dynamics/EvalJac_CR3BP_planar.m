function dJacobian = EvalJac_CR3BP_planar(dState, dMassRatio) %#codegen
arguments (Input)
    dState      (:,1) double {mustBeNumeric}
    dMassRatio  (1,1) double {mustBeNumeric, mustBePositive, mustBeLessThan(dMassRatio, 1)}
end
arguments (Output)
    dJacobian   (4,4) double
end
%% PROTOTYPE
% dJacobian = EvalJac_CR3BP_planar(dState, dMassRatio)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical Jacobian of the planar CR3BP equations of motion. Only the first 2 elements of dState (position)
% are used. Structure: A = [0 I; Uxx C] where Uxx is 2x2 Hessian of pseudo-potential and C is Coriolis.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dState       (:,1) double   State vector (at least 2 elements; only x,y used)
% dMassRatio   (1,1) double   Mass ratio m2/(m1+m2)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacobian    (4,4) double   Jacobian of the planar CR3BP
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 09-10-2022    Symbolic Math Toolbox   Auto-generated (hardcoded mu=0.012).
% 28-03-2026    Claude Code             Manual rewrite with parametric mass ratio, modern header.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

dPosX = dState(1);
dPosY = dState(2);

dGravP1 = 1 - dMassRatio;
dGravP2 = dMassRatio;

% Relative positions to primaries
dRelX_P1 = dPosX + dMassRatio;
dRelX_P2 = dPosX - (1 - dMassRatio);

% Distances squared (planar)
dDistP1_sq = dRelX_P1^2 + dPosY^2;
dDistP2_sq = dRelX_P2^2 + dPosY^2;

dInvDistP1 = 1.0 / sqrt(dDistP1_sq);
dInvDistP2 = 1.0 / sqrt(dDistP2_sq);
dInvDistP1_3 = dInvDistP1 / dDistP1_sq;
dInvDistP2_3 = dInvDistP2 / dDistP2_sq;
dInvDistP1_5 = dInvDistP1_3 / dDistP1_sq;
dInvDistP2_5 = dInvDistP2_3 / dDistP2_sq;
dGravP1OverDistP1_3 = dGravP1 * dInvDistP1_3;
dGravP2OverDistP2_3 = dGravP2 * dInvDistP2_3;
dScaleP1 = 3.0 * dGravP1 * dInvDistP1_5;
dScaleP2 = 3.0 * dGravP2 * dInvDistP2_5;

dSumInvCube = dGravP1OverDistP1_3 + dGravP2OverDistP2_3;

% Hessian of pseudo-potential (2x2)
dUxx = 1 - dSumInvCube + dScaleP1 * dRelX_P1^2 + dScaleP2 * dRelX_P2^2;
dUyy = 1 - dSumInvCube + dScaleP1 * dPosY^2    + dScaleP2 * dPosY^2;
dUxy = dScaleP1 * dRelX_P1 * dPosY + dScaleP2 * dRelX_P2 * dPosY;

% Assemble 4x4 Jacobian
dJacobian = [0,    0,    1,  0;
    0,    0,    0,  1;
    dUxx, dUxy, 0,  2;
    dUxy, dUyy, -2, 0];

end
