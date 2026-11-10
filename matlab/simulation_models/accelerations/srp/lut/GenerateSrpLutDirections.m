function dDirections = GenerateSrpLutDirections(ui32DirectionCount)
%% SIGNATURE
% dDirections = GenerateSrpLutDirections(ui32DirectionCount)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate deterministic Fibonacci-sphere directions without using random state.
% Use this independent sampling for panel parity and lookup interpolation audits.
% Example: dDirections = GenerateSrpLutDirections(uint32(240));
% Output: A 3-by-240 array of unit sphere directions.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32DirectionCount        Positive number of directions.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDirections               Unit vectors expressed in the caller's frame [-].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026  Pietro Califano, Codex gpt-6  Share deterministic independent audit sampling.
% 01-10-2026  Pietro Califano, Codex gpt-6  Rename the LUT direction generator and clarify sampling.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    ui32DirectionCount (1, 1) uint32 {mustBePositive}
end

arguments (Output)
    dDirections (3, :) double
end

% Space samples uniformly along the sphere axis; offset them to avoid either pole.
dSampleIndex = 0:double(ui32DirectionCount)-1;
dAxisCoordinate = 1 - 2 * (dSampleIndex + 0.5) / double(ui32DirectionCount);

% Spread azimuths by the golden angle independently of the LUT grid.
dAzimuthAngle = dSampleIndex * pi * (3 - sqrt(5));
dEquatorialRadius = sqrt(1 - dAxisCoordinate.^2);
dDirections = [dEquatorialRadius .* cos(dAzimuthAngle); ...
               dEquatorialRadius .* sin(dAzimuthAngle); ...
               dAxisCoordinate];
end
