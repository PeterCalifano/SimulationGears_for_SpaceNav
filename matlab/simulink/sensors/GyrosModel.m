function [o_dMeasAngRate, dNewBiasValue] = GyrosModel(i_dTrueAngRate, i_dScaleFactor, ...
    i_dOrthogErrMatrix, i_dConfigMat_IMUfromSCB, i_dSigmaRRW, i_dSigmaARW, ...
    i_dPrevBiasValue, i_dSamplingTime)%#codegen
%% SIGNATURE
% [o_dMeasAngRate, dNewBiasValue] = GyrosModel(i_dTrueAngRate, i_dScaleFactor, i_dOrthogErrMatrix, ...
%     i_dConfigMat_IMUfromSCB, i_dSigmaRRW, i_dSigmaARW, i_dPrevBiasValue, i_dSamplingTime)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Entry point of the MATLAB Function block of GyrosModel.slx and of codegen_GyrosModel. It delegates to
% GyrosModelStep, which holds the model and documents it. MATLAB code calls GyrosModelStep directly:
% GyrosModel.slx in this folder shadows this file there.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% See GyrosModelStep.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% See GyrosModelStep.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 20-06-2024  Pietro Califano               GyrosModel (PhD course Lab2).
% 29-10-2026  Pietro Califano, Claude Opus 5.5  Move the model body to GyrosModelStep.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% GyrosModelStep.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    i_dTrueAngRate          (3,1) double
    i_dScaleFactor          (3,1) double
    i_dOrthogErrMatrix      (3,3) double
    i_dConfigMat_IMUfromSCB (3,3) double
    i_dSigmaRRW             (1,1) double
    i_dSigmaARW             (1,1) double
    i_dPrevBiasValue        (3,1) double
    i_dSamplingTime         (1,1) double
end

[o_dMeasAngRate, dNewBiasValue] = GyrosModelStep(i_dTrueAngRate, i_dScaleFactor, i_dOrthogErrMatrix, ...
    i_dConfigMat_IMUfromSCB, i_dSigmaRRW, i_dSigmaARW, i_dPrevBiasValue, i_dSamplingTime);

end
