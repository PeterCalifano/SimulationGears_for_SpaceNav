function dxdt = RHS_CR3BP(dState, dMassRatio) %#codegen
arguments (Input)
    dState      (6,1) double {mustBeNumeric}
    dMassRatio  (1,1) double {mustBeNumeric, mustBePositive, mustBeLessThan(dMassRatio, 1)}
end
arguments (Output)
    dxdt        (6,1) double
end
%% PROTOTYPE
% dxdt = RHS_CR3BP(dState, dMassRatio)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Right-hand side of the Circular Restricted Three-Body Problem (CR3BP) equations of motion in the
% synodic (rotating) reference frame. The state vector is [x; y; z; vx; vy; vz] in non-dimensional
% units. The mass ratio dMassRatio = m2/(m1+m2) where m2 is the smaller primary.
% Positions of the primaries in the synodic frame: P1 at (-dMassRatio, 0, 0), P2 at (1-dMassRatio, 0, 0).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dState       (6,1) double   State vector [x; y; z; vx; vy; vz] in synodic frame (non-dim)
% dMassRatio   (1,1) double   Mass ratio of the CR3BP system, m2/(m1+m2) in (0,1)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxdt         (6,1) double   Time derivative of the state vector (non-dim)
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-10-2022    Symbolic Math Toolbox   Auto-generated version.
% 28-03-2026    Claude Code             Manual rewrite with readable variables, modern header.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

% Extract position and velocity
dPosX  = dState(1);
dPosY  = dState(2);
dPosZ  = dState(3);
dVelX  = dState(4);
dVelY  = dState(5);
dVelZ  = dState(6);

% Position vectors relative to primary (1-dMassRatio) and secondary (dMassRatio) bodies
% Primary P1 is at (-dMassRatio, 0, 0), Secondary P2 is at (1-dMassRatio, 0, 0)
dRelX_P1 = dPosX + dMassRatio;        % x - (-dMassRatio) = x + dMassRatio
dRelX_P2 = dPosX - (1 - dMassRatio);  % x - (1 - dMassRatio)

% Distances to primaries (cubed for gravitational terms)
dDistP1_sq = dRelX_P1^2 + dPosY^2 + dPosZ^2;
dDistP2_sq = dRelX_P2^2 + dPosY^2 + dPosZ^2;
dInvDistP1 = 1.0 / sqrt(dDistP1_sq);
dInvDistP2 = 1.0 / sqrt(dDistP2_sq);
dInvDistP1_3 = dInvDistP1 / dDistP1_sq;
dInvDistP2_3 = dInvDistP2 / dDistP2_sq;

% Gravitational parameters for each primary in the synodic frame
dGravP1 = 1 - dMassRatio;  % Mass parameter of P1
dGravP2 = dMassRatio;       % Mass parameter of P2
dGravP1OverDistP1_3 = dGravP1 * dInvDistP1_3;
dGravP2OverDistP2_3 = dGravP2 * dInvDistP2_3;

% Accelerations in synodic frame (includes Coriolis and centrifugal terms)
dAccX = dPosX + 2*dVelY - dGravP1OverDistP1_3 * dRelX_P1 - dGravP2OverDistP2_3 * dRelX_P2;
dAccY = dPosY - 2*dVelX - dGravP1OverDistP1_3 * dPosY    - dGravP2OverDistP2_3 * dPosY;
dAccZ =                  - dGravP1OverDistP1_3 * dPosZ    - dGravP2OverDistP2_3 * dPosZ;

% Assemble state derivative
dxdt = [dVelX; dVelY; dVelZ; dAccX; dAccY; dAccZ];

end
