function dPointSet = GenerateFibonacciSpherePoints(ui32NumPts, dRadius, dPhaseOffset)%#codegen
arguments
    ui32NumPts      (1,1) uint32
    dRadius         (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dPhaseOffset    (1,1) double {mustBeFinite, mustBeReal} = 0.0
end
%% PROTOTYPE
% dPointSet = GenerateFibonacciSpherePoints(ui32NumPts, dRadius, dPhaseOffset)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generates approximately uniform points on a sphere surface using the
% Fibonacci spiral construction.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32NumPts:      [1]         Number of requested points.
% dRadius:         [1]         Sphere radius.
% dPhaseOffset:    [1]         Optional phase offset applied to the spiral.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPointSet:       [3 x N]     Cartesian point set on the sphere surface.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Promote helper from SH fitter to shared utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
if ui32NumPts == uint32(0)
    error('GenerateFibonacciSpherePoints:InvalidCount', ...
        'ui32NumPts must be at least 1.');
end

% DEVNOTE: The Fibonacci sphere point generation is based on the method described by
% Markus Deserno in "How to generate equidistributed points on the surface of a sphere"
% (https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf), which provides a simple and efficient way to distribute points approximately uniformly on a sphere using the golden angle. The phase offset allows for decorrelation between different shells when generating multiple sets of points, which can be beneficial for numerical stability in applications like gravity field fitting.

% Calculate the golden angle in radians
dGoldenAngle = pi * (3.0 - sqrt(5.0));
dPointSet = zeros(3, double(ui32NumPts));

% Generate points using the Fibonacci spiral method
for idPt = 1:double(ui32NumPts)
    dZ = 1.0 - 2.0 * (idPt - 0.5) / double(ui32NumPts);
    dRxy = sqrt(max(0.0, 1.0 - dZ * dZ));
    dTheta = mod((idPt - 1 + dPhaseOffset) * dGoldenAngle, 2.0 * pi);
    dPointSet(:, idPt) = dRadius * [dRxy * cos(dTheta); dRxy * sin(dTheta); dZ];
end

end
