function dJacobian = EvalJac_CR3BP(dState, dMassRatio) %#codegen
arguments (Input)
    dState      (:,1) double {mustBeNumeric}
    dMassRatio  (1,1) double {mustBeNumeric, mustBePositive, mustBeLessThan(dMassRatio, 1)}
end
arguments (Output)
    dJacobian   (6,6) double
end
%% PROTOTYPE
% dJacobian = EvalJac_CR3BP(dState, dMassRatio)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical Jacobian (state transition matrix A(t)) of the CR3BP equations of motion evaluated at a given
% state. Only the position components of dState are used (first 3 elements). The Jacobian has the structure:
%   A = [  0      I   ]
%       [ Uxx+K   C   ]
% where Uxx is the Hessian of the pseudo-potential, K = diag([1,1,0]) (centrifugal), C = [0 2 0; -2 0 0; 0 0 0] (Coriolis).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dState       (:,1) double   State vector (at least 3 elements; only positions x,y,z are used)
% dMassRatio   (1,1) double   Mass ratio of the CR3BP system, m2/(m1+m2) in (0,1)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacobian    (6,6) double   Jacobian matrix of the CR3BP evaluated at dState
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2022    Symbolic Math Toolbox   Auto-generated version.
% 28-03-2026    Claude Code             Manual rewrite with readable partial derivatives.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

% Extract position
dPosX = dState(1);
dPosY = dState(2);
dPosZ = dState(3);

% Mass parameters
dGravP1 = 1 - dMassRatio;  % (1-mu)
dGravP2 = dMassRatio;       % mu

% Position relative to primaries
dRelX_P1 = dPosX + dMassRatio;         % x + mu
dRelX_P2 = dPosX - (1 - dMassRatio);   % x - (1-mu)

% Distances to primaries (squared)
dDistP1_sq = dRelX_P1^2 + dPosY^2 + dPosZ^2;
dDistP2_sq = dRelX_P2^2 + dPosY^2 + dPosZ^2;

% Inverse distance powers for potential derivatives
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

% Common terms: sum of gravitational inverse cube contributions
dSumInvCube = dGravP1OverDistP1_3 + dGravP2OverDistP2_3;

% Hessian of the pseudo-potential Uxx (3x3 symmetric matrix)
% Uxx_ij = d^2 U / (dxi dxj) + centrifugal contribution for diagonal (i=j, i<=2)
% General formula: Uij = -delta_ij * (sum_k mk/rk^3) + 3 * sum_k mk * (xi-xk_i)*(xj-xk_j) / rk^5

% Diagonal elements (include centrifugal +1 for xx, yy; 0 for zz)
dUxx = 1 - dSumInvCube + dScaleP1 * dRelX_P1^2 + dScaleP2 * dRelX_P2^2;
dUyy = 1 - dSumInvCube + dScaleP1 * dPosY^2    + dScaleP2 * dPosY^2;
dUzz =   - dSumInvCube + dScaleP1 * dPosZ^2    + dScaleP2 * dPosZ^2;

% Off-diagonal elements (symmetric)
dUxy = dScaleP1 * dRelX_P1 * dPosY + dScaleP2 * dRelX_P2 * dPosY;
dUxz = dScaleP1 * dRelX_P1 * dPosZ + dScaleP2 * dRelX_P2 * dPosZ;
dUyz = dScaleP1 * dPosY    * dPosZ + dScaleP2 * dPosY    * dPosZ;

% Assemble 6x6 Jacobian: A = [0 I; Uxx C] where C is the Coriolis matrix
dJacobian = [0,    0,    0,    1,  0,  0;
    0,    0,    0,    0,  1,  0;
    0,    0,    0,    0,  0,  1;
    dUxx, dUxy, dUxz, 0,  2,  0;
    dUxy, dUyy, dUyz, -2, 0,  0;
    dUxz, dUyz, dUzz, 0,  0,  0];

end
