function dxdt = RHS_CR3BP_planar(dState, dMassRatio) %#codegen
arguments (Input)
    dState      (4,1) double {mustBeNumeric}
    dMassRatio  (1,1) double {mustBeNumeric, mustBePositive, mustBeLessThan(dMassRatio, 1)}
end
arguments (Output)
    dxdt        (4,1) double
end
%% PROTOTYPE
% dxdt = RHS_CR3BP_planar(dState, dMassRatio)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Right-hand side of the planar Circular Restricted Three-Body Problem (CR3BP) in the synodic frame.
% State is [x; y; vx; vy] in non-dimensional units. This is the z=0 reduction of the full 3D CR3BP.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dState       (4,1) double   Planar state [x; y; vx; vy] in synodic frame (non-dim)
% dMassRatio   (1,1) double   Mass ratio m2/(m1+m2)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxdt         (4,1) double   Time derivative of the planar state (non-dim)
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
dVelX = dState(3);
dVelY = dState(4);

dGravP1 = 1 - dMassRatio;
dGravP2 = dMassRatio;

% Relative positions to primaries in x-direction
dRelX_P1 = dPosX + dMassRatio;
dRelX_P2 = dPosX - (1 - dMassRatio);

% Distances squared (planar, z=0)
dDistP1_sq = dRelX_P1^2 + dPosY^2;
dDistP2_sq = dRelX_P2^2 + dPosY^2;
dInvDistP1 = 1.0 / sqrt(dDistP1_sq);
dInvDistP2 = 1.0 / sqrt(dDistP2_sq);
dInvDistP1_3 = dInvDistP1 / dDistP1_sq;
dInvDistP2_3 = dInvDistP2 / dDistP2_sq;
dGravP1OverDistP1_3 = dGravP1 * dInvDistP1_3;
dGravP2OverDistP2_3 = dGravP2 * dInvDistP2_3;

% Equations of motion (synodic frame with Coriolis and centrifugal)
dAccX = dPosX + 2*dVelY - dGravP1OverDistP1_3 * dRelX_P1 - dGravP2OverDistP2_3 * dRelX_P2;
dAccY = dPosY - 2*dVelX - dGravP1OverDistP1_3 * dPosY    - dGravP2OverDistP2_3 * dPosY;

dxdt = [dVelX; dVelY; dAccX; dAccY];

end
