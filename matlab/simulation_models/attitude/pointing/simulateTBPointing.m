function [o_dqSCBwrtIN_ref, o_dqSCBwrtIN_est, ...
    o_dqCAMwrtIN_ref, o_dqCAMwrtIN_est] = simulateTBPointing(i_dXSC_IN_ref, ...
    i_drTargetPoint_IN, ...
    i_dqCAMwrtSCB, ...
    i_dSigmaAKE, ...
    i_bIS_VSRPplus, ...
    i_bINVERSE_Z_AXIS, ...
    i_bIS_QLEFT_HANDED) %#codegen
arguments
    i_dXSC_IN_ref        (6,1)
    i_drTargetPoint_IN   (3,1)
    i_dqCAMwrtSCB        (4,1)
    i_dSigmaAKE          (1,1)
    i_bIS_VSRPplus       (1,1) logical
    i_bINVERSE_Z_AXIS    (1,1) logical
    i_bIS_QLEFT_HANDED   (1,1) logical
end
%% PROTOTYPE
% [o_dqSCBwrtIN_ref, o_dqSCBwrtIN_est, o_dqCAMwrtIN_ref, o_dqCAMwrtIN_est] = simulateTBPointing(i_dXSC_IN_ref, ...
%     i_drTargetPoint_IN, ...
%     i_dqCAMwrtSCB, ...
%     i_dSigmaAPE,
%     i_bIS_VSRPplus, ...
%     i_bINVERSE_Z_AXIS, ...
%     i_bIS_QLEFT_HANDED) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% ACHTUNG: MAJOR REWORKING TO DO --> simulate both pointing error and knowledge error (double scatter)
% Simulation model for pointing of NAVCAM to targeted point in Visual-based
% navigation scenarios. The attitude is build based on the +Z boresight of
% the camera, with the other axis based on the orbit angular momentum and 
% triad constraint. The SCB attitude is built backward through the knowledge 
% of the CAM attitude quaternion wrt SCB frame.
% ASSUMPTIONS: 
%   1) Camera reference frame assumed aligned with "opposite" LVLH with
%      respect to the main attractor.
%   2) Attitude error is injected in a simplified way as white noise based 
%      on the APE GNC performance (zero mean, 1 sigma accuracy). 
%   3) CAM origin assumed coincident with SCB origin.
%   4) Quaternions in VSRPplus convention.
% NOTE on quaternion convention:
% (SV) Scalar first, Vector last
% (P) Passive 
% (R) Successive coordinate transformations have the unmodified quaternion chain on the Right side of
%     the triple product.
% (plus) Right-Handed Rule for the imaginary numbers i, j, k. (aka Hamilton)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_dXSC_IN_ref:      [Nx, 1] SC state vector with [Position, Velocity] as first
%                             6 components. Assumed as "truth" in this routine.
% i_drTargetPoint_IN: [3, 1]  Position vector of the targeted point in the Global
%                             frame. Assumed as "truth".
% i_dqCAMwrtSCB:      [4, 1]  Configuration attitude quaternion of the Camera
%                             frame wrt the SC Body frame. JPL convention.
%                             Assumed as "truth".
% i_dSigmaAKE:        [1]     1-sigma Standard dev. of Attitude knowledge error
%                             simulated as White Gaussian noise half-cone angle
%                             in RADIANS.
% i_bIS_VSRPplus:     [1]
% i_bINVERSE_Z_AXIS:  [1]
% i_bIS_QLEFT_HANDED: [1]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dqSCBwrtIN_ref: [4, 1]  Reference SCB attitude quaternion wrt IN frame
% o_dqSCBwrtIN_est: [4, 1]  Simulated estimated SCB attitude quaternion (WN noise)
% o_dqCAMwrtIN_ref: [4, 1]  Reference Cam attitude quaternion wrt IN frame
% o_dqCAMwrtIN_est: [4, 1]  Simulated estimated Cam attitude quaternion (WN noise)
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-10-2023    Pietro Califano    Coded from previous model. 
% 31-01-2024    Pietro Califano    Target pointing attitude quaternion verified.
% 29-04-2024    Pietro Califano    Upgrade to handle different quaternion conventions (Not validated)
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% qCross()
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Add displacement of CAM origin from SCB
% 2) Add correlation of the attitude error in time using a FOGM
% 3) MAJOR: Distinguish between pointing error (to add to ref to simulate
%           pointing error (first scattering from truth) and knowledge 
%           error (second scattering). The current version is not accurate.
% -------------------------------------------------------------------------------------------------------------
%% Function code

assert(iscolumn(i_dXSC_IN_ref), 'Input state vector must be a column vector!')
assert(iscolumn(i_drTargetPoint_IN), 'Input target position must be a column vector!')

% Relative position in IN from CAM origin to the TB centre
CAMframeZdir_IN_ref = i_drTargetPoint_IN(1:3) - i_dXSC_IN_ref(1:3);

if not(exist('i_bINVERSE_Z_AXIS', 'var'))
    i_bINVERSE_Z_AXIS = false;
end

% Compute 1st direction (+Z) in IN
if i_bINVERSE_Z_AXIS == false
    CAMframeZdir_IN_ref = CAMframeZdir_IN_ref./norm(CAMframeZdir_IN_ref);

elseif i_bINVERSE_Z_AXIS == true
    CAMframeZdir_IN_ref = -CAMframeZdir_IN_ref./norm(CAMframeZdir_IN_ref);
    
end

% Compute 2nd direction (+X) in IN
hVec = cross(i_dXSC_IN_ref(1:3), i_dXSC_IN_ref(4:6));
CAMframeXdir_IN_ref = hVec./norm(hVec);

% Compute 3rd direction (+Y) in IN (Right Handed triad)
CAMframeYdir_IN_ref = cross(CAMframeZdir_IN_ref, CAMframeXdir_IN_ref);
CAMframeYdir_IN_ref = CAMframeYdir_IN_ref/norm(CAMframeYdir_IN_ref);

% Build attitude representation in DCM converting from CAM to CI

dDCM_fromCAMtoIN_ref = [CAMframeXdir_IN_ref, CAMframeYdir_IN_ref, CAMframeZdir_IN_ref];

if i_bIS_QLEFT_HANDED
    dDCM_fromCAMtoIN_ref = transpose(dDCM_fromCAMtoIN_ref);
end
    
% Generate CAM attitude quaternion truth and knowledge 
o_dqCAMwrtIN_ref = DCM2quat(dDCM_fromCAMtoIN_ref, i_bIS_VSRPplus);

if i_dSigmaAKE > 0

    if i_dSigmaAKE >= deg2rad(10/3)
        warning('Small angle approximation is not valid.')
    end

    dqNoiseVec = i_dSigmaAKE * randn(3, 1);
    dqNoiseScalar = sqrt(1 - (norm(dqNoiseVec))^2);

    dqNoise = [dqNoiseVec; dqNoiseScalar];
    % Add white noise for pointing error simulation
    o_dqCAMwrtIN_est = qCross(o_dqCAMwrtIN_ref, dqNoise);
    o_dqCAMwrtIN_est = o_dqCAMwrtIN_est/norm(o_dqCAMwrtIN_est);

else
    o_dqCAMwrtIN_est = o_dqCAMwrtIN_ref;
end

% Generate SCB attitude quaternion truth and knowledge

if i_bIS_QLEFT_HANDED == false
    % If quaternion is Right-Handed
    o_dqSCBwrtIN_ref = qCross(o_dqCAMwrtIN_ref, i_dqCAMwrtSCB);
    o_dqSCBwrtIN_est = qCross(o_dqCAMwrtIN_est, i_dqCAMwrtSCB);
else
    % Switch rotation sign if Left Handed
    o_dqSCBwrtIN_ref = qCross(i_dqCAMwrtSCB, o_dqCAMwrtIN_ref);
    o_dqSCBwrtIN_est = qCross(i_dqCAMwrtSCB, o_dqCAMwrtIN_est);
end


end
