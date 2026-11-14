function [dMeasDistance, bValidityFlag, dMeasErr] = ...
    ApplyLidarMeasurementError(dTrueDistance, bIntersectionFlag, dWhiteNoiseSigma, ...
                              dConstantBias, dValidInterval, bEnableNoise, bEnableChecks) %#codegen
%% SIGNATURE
% [dMeasDistance, bValidityFlag, dMeasErr] = ApplyLidarMeasurementError(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply the shared LiDAR noise, bias and range-validity policy after tracing.
% A geometric miss remains invalid and consumes no noise. Inclusive physical range checks use the noiseless
% intersection, so sensor error cannot create or remove physical acquisition support. Every geometric hit
% retains its existing noise draw, including hits outside the physical interval. Invalid numeric measured
% outputs are rejected independently of physical support.
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
% bValidityFlag       Geometry satisfies enabled physical bounds and the realized measurement is
%                     finite/nonnegative.
% dMeasErr            Realized additive sensor error; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Share sensor policy across codegen interfaces.
% 28-10-2026  Pietro Califano, Codex (GPT-6)  Check physical support before noise without changing hit draws.
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

%% Function code

% Return the geometric miss before advancing the sensor's random stream.
dMeasDistance = -1;
bValidityFlag = false;
dMeasErr = 0;

if ~bIntersectionFlag
    return
end

% Resolve acquisition support before noise while retaining every geometric hit's draw schedule.
bValidityFlag = isfinite(dTrueDistance) && dTrueDistance >= 0;
if bEnableChecks
    bValidityFlag = bValidityFlag && dTrueDistance >= dValidInterval(1) && ...
        dTrueDistance <= dValidInterval(2);
end

% A hit outside physical support still consumes the same sensor-noise draw as an in-range hit.
if bEnableNoise
    dMeasErr = LaserRangefinderNoiseModel(dWhiteNoiseSigma, dTrueDistance, dConstantBias);
end

dMeasDistance = dTrueDistance + dMeasErr;

% Numeric output validity is independent of the inclusive physical range decision.
bValidityFlag = bValidityFlag && isfinite(dMeasDistance) && dMeasDistance >= 0;
end
