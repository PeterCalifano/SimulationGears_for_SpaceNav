function dMeasurement = RunCentroidingSensorExample()
%% SIGNATURE
% dMeasurement = RunCentroidingSensorExample()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Project a target with an explicit constant pixel bias, without consuming RNG.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasurement: [72;95] pixels. Printed output: Centroid: 72.0 95.0 pixels
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Demonstrate the owned sensor API.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; rename from RunCentroidSensorExample.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CCentroidingSensorModel.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
end
arguments (Output)
    dMeasurement (2,1) double
end

% Project [2;3;10] with fx=100, fy=120, cx=50, cy=60 and add the bias [2;-1]
objSensor = CCentroidingSensorModel('dConstantBias', [2; -1]);
[~, dMeasurement, bValid] = objSensor.step(0, [2; 3; 10], ...
    [100, 0, 50; 0, 120, 60; 0, 0, 1], zeros(2, 1), zeros(2, 1));

assert(bValid);
fprintf('Centroid: %.1f %.1f pixels\n', dMeasurement);
end
