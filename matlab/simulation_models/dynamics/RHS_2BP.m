function [dxdt] = RHS_2BP(~, dPosVelState, strDynParams) %#codegen
%% SIGNATURE
% [dxdt] = RHS_2BP(~, dPosVelState, strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Right-hand side of the two-body problem (2BP) equations of motion in Cartesian coordinates. The state vector
% is [position; velocity] in an inertial frame. Gravitational parameter is provided via strDynParams.dGravParam.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosVelState    (6,1) double   Position and velocity state vector [LU, LU/TU]
% strDynParams    (1,1) struct   Dynamics parameters struct with field dGravParam [LU^3/TU^2]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxdt            (6,1) double   Time derivative of state vector [LU/TU, LU/TU^2]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 2022            Pietro Califano     Initial version.
% 28-03-2026      Claude Code         Modernized header, arguments block, naming conventions.
% 11-10-2026      Pietro Califano     Reuse the shared point-mass acceleration kernel.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePointMassGravity
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    ~
    dPosVelState    (6,1) double {mustBeNumeric}
    strDynParams    (1,1) struct
end

arguments (Output)
    dxdt            (6,1) double
end

% Resolve position in the central body's inertial frame.
dPosVec = dPosVelState(1:3);

% Assemble state derivative: velocity and gravitational acceleration
dxdt = [dPosVelState(4:6);
        ComputePointMassGravity(dPosVec, strDynParams.dGravParam)];

end
