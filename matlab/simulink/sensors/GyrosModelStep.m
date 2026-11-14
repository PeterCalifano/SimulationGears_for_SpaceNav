function [o_dMeasAngRate, dNewBiasValue] = GyrosModelStep(i_dTrueAngRate, i_dScaleFactor, ...
    i_dOrthogErrMatrix, i_dConfigMat_IMUfromSCB, i_dSigmaRRW, i_dSigmaARW, ...
    i_dPrevBiasValue, i_dSamplingTime) %#codegen
%% SIGNATURE
% [o_dMeasAngRate, dNewBiasValue] = GyrosModelStep(i_dTrueAngRate, i_dScaleFactor, i_dOrthogErrMatrix, ...
%     i_dConfigMat_IMUfromSCB, i_dSigmaRRW, i_dSigmaARW, i_dPrevBiasValue, i_dSamplingTime)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% One sample of the three-axis gyroscope model: bias random walk (rate random walk, RRW) and white rate noise
% (angle random walk, ARW) on the true angular rate seen through the scale factor, the axis misalignment and the
% mounting of the IMU:
%   b_k     = b_{k-1} + w_RRW,   w_RRW ~ N(0, i_dSigmaRRW / (2 Ts) I)
%   omega_m = (I + diag(i_dScaleFactor)) i_dOrthogErrMatrix i_dConfigMat_IMUfromSCB omega + b_k + w_ARW,
%             w_ARW ~ N(0, i_dSigmaARW / (2 Ts) I)
% The bias is stepped before it is used, so the first sample already carries one RRW increment. The two
% parameters are variances (they are passed to mvnrnd as covariance scales despite their names): the per-sample
% standard deviations are sqrt(i_dSigmaRRW / (2 Ts)) and sqrt(i_dSigmaARW / (2 Ts)). Draws: RRW (three
% values), then ARW (three values), from the MATLAB global stream through mvnrnd; a zero variance draws nothing.
% GyrosModel.m delegates here and stays the entry point of the MATLAB Function block of GyrosModel.slx and of
% codegen_GyrosModel. GyrosModel.slx in the same folder shadows GyrosModel.m in MATLAB code (calls resolve to
% the Simulink model), so MATLAB callers use this function.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_dTrueAngRate          (3,1) double true angular rate of the spacecraft body (SCB axes) [rad/s]
% i_dScaleFactor          (3,1) double scale-factor errors per IMU axis [-]
% i_dOrthogErrMatrix      (3,3) double axis misalignment (non-orthogonality) matrix [-]
% i_dConfigMat_IMUfromSCB (3,3) double rotation from SCB to IMU axes [-]
% i_dSigmaRRW             (1,1) double RRW variance parameter [(rad/s)^2 s]
% i_dSigmaARW             (1,1) double ARW variance parameter [(rad/s)^2 s]
% i_dPrevBiasValue        (3,1) double bias of the previous sample (IMU axes) [rad/s]
% i_dSamplingTime         (1,1) double sampling period Ts [s]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dMeasAngRate          (3,1) double measured angular rate (IMU axes) [rad/s]
% dNewBiasValue           (3,1) double bias of this sample (IMU axes) [rad/s]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 20-06-2024  Pietro Califano               GyrosModel (PhD course Lab2).
% 29-10-2026  Pietro Califano, Claude Opus 5.5  Move the body of GyrosModel here unchanged, so that MATLAB code
%                                               can call the model.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% mvnrnd (Statistics and Machine Learning Toolbox).
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

% Variables declaration
dNoiseInputRRW = zeros(3, 1);
dNewBiasValue  = zeros(3, 1);
dNoiseInputARW = zeros(3, 1);
o_dMeasAngRate = zeros(3, 1);

% Simulate White noise and random walk bias values
dNoiseInputRRW(:) = 1/sqrt(2*i_dSamplingTime) * mvnrnd([0; 0; 0], i_dSigmaRRW * eye(3));
dNoiseInputARW(:) = 1/sqrt(2*i_dSamplingTime) * mvnrnd([0; 0; 0], i_dSigmaARW * eye(3));

dNewBiasValue(:) = i_dPrevBiasValue + dNoiseInputRRW;


% Rotate the true rate into the IMU axes, apply misalignment and scale factor, add bias and white noise
o_dMeasAngRate(:) = ((eye(3) + diag(i_dScaleFactor)) * (i_dOrthogErrMatrix * i_dConfigMat_IMUfromSCB) * ...
                     i_dTrueAngRate) + dNewBiasValue + dNoiseInputARW;

end
