function [dxdt] = RHS_2BP(~, dPosVelState, strDynParams) %#codegen
arguments (Input)
    ~
    dPosVelState    (6,1) double {mustBeNumeric}
    strDynParams    (1,1) struct
end
arguments (Output)
    dxdt            (6,1) double
end
%% PROTOTYPE
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
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

% Extract position vector and compute distance from central body
dPosVec = dPosVelState(1:3);
dDistNorm = norm(dPosVec);

% Assemble state derivative: velocity and gravitational acceleration
dxdt = [dPosVelState(4:6);
        -strDynParams.dGravParam / dDistNorm^3 * dPosVec];

end
