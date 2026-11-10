function [dMeasDistance, bValidityFlag, dMeasErr] = ...
    ApplyLidarMeasurementError(dTrueDistance, bIntersectionFlag, dWhiteNoiseSigma, ...
                              dConstantBias, dValidInterval, bEnableNoise, bEnableChecks) %#codegen
%% SIGNATURE
% [dMeasDistance, bValidityFlag, dMeasErr] = ApplyLidarMeasurementError(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply the shared LiDAR noise, bias and range-validity policy after tracing.
% A geometric miss remains invalid and consumes no noise. Range checks use
% inclusive endpoints after the realized sensor error has been added.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTrueDistance       Geometric range in the mesh length unit.
% bIntersectionFlag   True when tracing found a forward surface.
% dWhiteNoiseSigma    Sensor white-noise standard deviation in that length unit.
% dConstantBias       Additive sensor bias.
% dValidInterval      (2,1) inclusive lower and upper range bounds.
% bEnableNoise        Apply the configured noise and bias.
% bEnableChecks       Enforce the range-validity interval.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasDistance       Measured range; -1 on a geometric miss.
% bValidityFlag       Measurement satisfies geometry and enabled range checks.
% dMeasErr            Realized additive sensor error; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Share sensor policy across codegen interfaces.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% LaserRangefinderNoiseModel.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dTrueDistance       (1, 1) double
    bIntersectionFlag   (1, 1) logical
    dWhiteNoiseSigma    (1, 1) double
    dConstantBias       (1, 1) double
    dValidInterval      (2, 1) double
    bEnableNoise        (1, 1) logical
    bEnableChecks       (1, 1) logical
end

arguments (Output)
    dMeasDistance  (1, 1) double
    bValidityFlag  (1, 1) logical
    dMeasErr       (1, 1) double
end

% Return the geometric miss before advancing the sensor's random stream.
dMeasDistance = -1;
bValidityFlag = false;
dMeasErr = 0;

if ~bIntersectionFlag
    return
end

% Apply one authoritative noise model to valid geometric returns.
if bEnableNoise
    dMeasErr = LaserRangefinderNoiseModel(dWhiteNoiseSigma, dTrueDistance, dConstantBias);
end

dMeasDistance = dTrueDistance + dMeasErr;
bValidityFlag = true;

% Keep interval acceptance independent of the geometric hit flag.
if bEnableChecks
    if dMeasDistance < dValidInterval(1) || dMeasDistance > dValidInterval(2) || ...
            dMeasDistance < 0
        bValidityFlag = false;
    end
end
end
