function [dSamplePos_TB, ui32ShellIds] = GenerateShellPointSet(dShellRadii, ui32PtsPerShell, dPhaseBase)%#codegen
arguments
    dShellRadii     (1,:) double {mustBeFinite, mustBeReal, mustBePositive}
    ui32PtsPerShell (:,:) uint32
    dPhaseBase      (1,1) double {mustBeFinite, mustBeReal} = 0.0
end
%% PROTOTYPE
% [dSamplePos_TB, ui32ShellIds] = GenerateShellPointSet(dShellRadii, ui32PtsPerShell, dPhaseBase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generates a stacked sample set on multiple spherical shells, using a
% Fibonacci distribution on each shell.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dShellRadii:     [1 x Ns]    Shell radii.
% ui32PtsPerShell: [1 x Ns]    Number of points per shell. A scalar value
%                              is broadcast to all shells.
% dPhaseBase:      [1]         Base phase offset for shell-to-shell decorrelation.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSamplePos_TB:   [3 x N]     All generated sample positions.
% ui32ShellIds:    [N x 1]     Shell index for each sample.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Promote helper from SH fitter to shared utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% GenerateFibonacciSpherePoints()
% -------------------------------------------------------------------------------------------------------------

ui32PtsPerShell = reshape(ui32PtsPerShell, 1, []);
if isscalar(ui32PtsPerShell)
    ui32PtsPerShell = repmat(ui32PtsPerShell, size(dShellRadii));
end

if numel(ui32PtsPerShell) ~= numel(dShellRadii)
    error('GenerateShellPointSet:SizeMismatch', ...
        'ui32PtsPerShell must be scalar or match the number of shell radii.');
end

if any(ui32PtsPerShell < uint32(1), 'all')
    error('GenerateShellPointSet:InvalidCounts', ...
        'Each shell must contain at least one sample point.');
end

ui32NumShells = uint32(numel(dShellRadii));
ui32TotalPts = sum(ui32PtsPerShell);

dSamplePos_TB = zeros(3, double(ui32TotalPts));
ui32ShellIds = zeros(double(ui32TotalPts), 1, 'uint32');

% Generate points shell by shell (uniformly spaced in radius, Fibonacci distribution in angle)
idStart = uint32(1);
for idShell = uint32(1):ui32NumShells

    ui32Count = ui32PtsPerShell(idShell);

    % Generate Fibonacci points on the current shell with a phase offset for decorrelation
    dShellPts = GenerateFibonacciSpherePoints(ui32Count, dShellRadii(idShell), dPhaseBase + double(idShell));
    idStop = idStart + ui32Count - uint32(1);

    dSamplePos_TB(:, double(idStart):double(idStop)) = dShellPts;
    ui32ShellIds(double(idStart):double(idStop)) = idShell;

    idStart = idStop + uint32(1);
end

end
