function [dPotentialPert, dAccPert_W, dAccPert_TB, dPosSC_TB] = EvalExtSphHarmExpInWorldFrame( ...
    dPosSC_W, dDCM_WfromTB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
arguments
    dPosSC_W            (3,1) double {mustBeFinite, mustBeReal}
    dDCM_WfromTB        (3,3) double {mustBeFinite, mustBeReal}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeFinite, mustBeReal}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dPotentialPert, dAccPert_W, dAccPert_TB, dPosSC_TB] = EvalExtSphHarmExpInWorldFrame( ...
%     dPosSC_W, dDCM_WfromTB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% World-frame wrapper for the Exterior Spherical Harmonics Expansion model.
%
% The spacecraft position is rotated into the target-body fixed frame,
% evaluated there, and the non-spherical acceleration is rotated back to
% the world frame.
%
% ACHTUNG: The input rotation must map target-body-fixed vectors into the
% world frame. The potential is frame-invariant, while the acceleration is
% returned in both frames for convenience.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_W:          [3x1]     Position in the world frame.
% dDCM_WfromTB:      [3x3]     Rotation matrix mapping target-body-fixed
%                              vectors into the world frame.
% ui32MaxDegree:     [1]       Maximum harmonic degree.
% dCSlmCoeffCols:    [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
% dGravParam:        [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:    [1]       Reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:    [1]       Non-spherical potential [LU^2/TU^2].
% dAccPert_W:        [3x1]     Non-spherical acceleration in the world frame.
% dAccPert_TB:       [3x1]     Non-spherical acceleration in the target-body
%                              fixed frame.
% dPosSC_TB:         [3x1]     Position rotated into the target-body frame.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-04-2026    Pietro Califano     Add world-frame wrapper for Ext SHE evaluation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalExtSphHarmExpInTargetFrame()
% -------------------------------------------------------------------------------------------------------------

%% Function code
dPosSC_TB = transpose(dDCM_WfromTB) * dPosSC_W;

% Evaluate the potential and acceleration in the target-body frame
[dPotentialPert, dAccPert_TB] = EvalExtSphHarmExpInTargetFrame( ...
    dPosSC_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

dAccPert_W = dDCM_WfromTB * dAccPert_TB;

end
