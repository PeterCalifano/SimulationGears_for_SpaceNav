function [dQuaternionHistory_TBfromIN, dAngVelHistory_TB] = PropagateTorqueFreeBodyHistory(dTimegrid, ...
    dInitialQuaternion_TBfromIN, ...
    dInitialAngVel_TB, ...
    dInertiaTensor_TB, ...
    dMaxStep) %#codegen
%% SIGNATURE
% [dQuaternionHistory_TBfromIN, dAngVelHistory_TB] = PropagateTorqueFreeBodyHistory(dTimegrid, ...
%     dInitialQuaternion_TBfromIN, dInitialAngVel_TB, dInertiaTensor_TB, dMaxStep)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Propagate a torque-free rigid body's passive inertial-to-body quaternion and body-frame angular velocity over
% one requested output grid. The function validates the physical inertia once, performs complete maximum-size
% internal steps plus at most one remainder step per output interval, and calls the stateless MathCore
% RK4-RKMK4 primitive. It owns no scenario configuration, source selection, frame conversion, or interpolation
% policy.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTimegrid                    (1,N) double strictly increasing output epochs [s]
% dInitialQuaternion_TBfromIN  (4,1) double scalar-first Hamilton/passive quaternion
% dInitialAngVel_TB            (3,1) double initial body-frame angular velocity [rad/s]
% dInertiaTensor_TB            (3,3) double physical center-of-mass inertia tensor
% dMaxStep                     (1,1) double maximum internal integration step [s]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dQuaternionHistory_TBfromIN  (4,N) double normalized, sign-continuous quaternion history
% dAngVelHistory_TB            (3,N) double body-frame angular-velocity history [rad/s]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     First stateless whole-history reference implementation.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% 12-11-2026  Pietro Califano, Claude Opus 5.5   Check the inertia with ValidatePhysicalInertia.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% IntegrateRigidBodyRK4RKMK4Step, ValidatePhysicalInertia
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
    dInitialQuaternion_TBfromIN (4,1) double {mustBeReal, mustBeFinite}
    dInitialAngVel_TB (3,1) double {mustBeReal, mustBeFinite}
    dInertiaTensor_TB (3,3) double {mustBeReal, mustBeFinite}
    dMaxStep (1,1) double {mustBePositive, mustBeFinite}
end
arguments (Output)
    dQuaternionHistory_TBfromIN (4,:) double
    dAngVelHistory_TB (3,:) double
end

% Require at least two strictly increasing epochs and a physical inertia tensor
ui32NumTimes = uint32(numel(dTimegrid));
if ui32NumTimes < uint32(2) || any(diff(dTimegrid) <= 0.0)
    error('PropagateTorqueFreeBodyHistory:InvalidTimegrid', ...
        'Timegrid must contain at least two strictly increasing finite epochs.');
end

% Use the symmetric part of the accepted tensor for both the Euler dynamics and the physical-inertia check
dInertiaTensor_TB = ValidatePhysicalInertia(dInertiaTensor_TB);

% Normalize the initial quaternion
dInitialQuaternionNorm = norm(dInitialQuaternion_TBfromIN);
if dInitialQuaternionNorm <= eps
    error('PropagateTorqueFreeBodyHistory:InvalidQuaternion', ...
        'Initial quaternion must have strictly positive norm.');
end
dCurrentQuaternion = dInitialQuaternion_TBfromIN / dInitialQuaternionNorm;
dCurrentAngVel_TB = dInitialAngVel_TB;
dZeroTorque_TB = zeros(3, 1);

% Allocate exactly the requested output shape; internal states remain scalar temporaries and never cross the
% public whole-history boundary.
dQuaternionHistory_TBfromIN = zeros(4, double(ui32NumTimes));
dAngVelHistory_TB = zeros(3, double(ui32NumTimes));
dQuaternionHistory_TBfromIN(:, 1) = dCurrentQuaternion;
dAngVelHistory_TB(:, 1) = dCurrentAngVel_TB;

for ui32TimeIdx = uint32(2):ui32NumTimes

    % Count the full internal steps of this output interval
    dOutputInterval = dTimegrid(ui32TimeIdx) - dTimegrid(ui32TimeIdx - 1);
    dNumFullSteps = floor(dOutputInterval / dMaxStep);
    if dNumFullSteps > double(intmax('uint32'))
        error('PropagateTorqueFreeBodyHistory:TooManyInternalSteps', ...
            'One output interval requires more internal steps than the uint32 loop supports.');
    end
    ui32NumFullSteps = uint32(dNumFullSteps);

    % Take the full steps
    for ui32StepIdx = uint32(1):ui32NumFullSteps
        [dCurrentQuaternion, dCurrentAngVel_TB] = IntegrateRigidBodyRK4RKMK4Step(dInertiaTensor_TB, ...
            dCurrentQuaternion, ...
            dCurrentAngVel_TB, ...
            dZeroTorque_TB, ...
            dMaxStep);
    end

    % Land exactly on each requested epoch without the class integrator's synthetic midpoint special case for
    % two-entry output grids.
    dRemainderStep = dOutputInterval - double(ui32NumFullSteps) * dMaxStep;
    dTimeTolerance = 16.0 * eps(max([1.0, dOutputInterval, dMaxStep]));
    if dRemainderStep > dTimeTolerance
        [dCurrentQuaternion, dCurrentAngVel_TB] = IntegrateRigidBodyRK4RKMK4Step(dInertiaTensor_TB, ...
            dCurrentQuaternion, ...
            dCurrentAngVel_TB, ...
            dZeroTorque_TB, ...
            dRemainderStep);
    elseif dRemainderStep < -dTimeTolerance
        error('PropagateTorqueFreeBodyHistory:InvalidStepPartition', ...
            'Internal step partition produced a negative remainder.');
    end

    % Quaternion signs carry no physical state, but continuity is required by downstream interpolation and
    % fitting.
    if dot(dCurrentQuaternion, dQuaternionHistory_TBfromIN(:, ui32TimeIdx - 1)) < 0.0
        dCurrentQuaternion = -dCurrentQuaternion;
    end
    dQuaternionHistory_TBfromIN(:, ui32TimeIdx) = dCurrentQuaternion;
    dAngVelHistory_TB(:, ui32TimeIdx) = dCurrentAngVel_TB;
end

end
