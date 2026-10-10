function strMetrics = ComputeGravityFieldFitMetrics(dPotentialPred, ...
                                                dPotentialTrue, ...
                                                dAccPredTB, ...
                                                dAccTrueTB, ...
                                                ui32ShellIds, ...
                                                ui32NumShells)
arguments
    dPotentialPred  (:,1) double {mustBeFinite, mustBeReal}
    dPotentialTrue  (:,1) double {mustBeFinite, mustBeReal}
    dAccPredTB      (3,:) double {mustBeFinite, mustBeReal}
    dAccTrueTB      (3,:) double {mustBeFinite, mustBeReal}
    ui32ShellIds    (:,1) uint32
    ui32NumShells   (1,1) uint32
end
%% PROTOTYPE
% strMetrics = ComputeGravityFieldFitMetrics( ...
%     dPotentialPred, dPotentialTrue, dAccPredTB, dAccTrueTB, ui32ShellIds, ui32NumShells)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes pointwise, global RMS, and shell-wise relative error metrics
% between two perturbative gravity field sample sets.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPotentialPred:  [N x 1]     Predicted perturbative potential.
% dPotentialTrue:  [N x 1]     Reference perturbative potential.
% dAccPredTB:      [3 x N]     Predicted perturbative acceleration.
% dAccTrueTB:      [3 x N]     Reference perturbative acceleration.
% ui32ShellIds:    [N x 1]     Shell assignment for each point.
% ui32NumShells:   [1]         Total number of shells.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strMetrics: struct with RMS, max, shell-wise, and pointwise relative
%             error metrics for potential and acceleration.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Promote helper from SH fitter to shared utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Input validation
ui32NumSamples = uint32(numel(dPotentialPred));
if numel(dPotentialTrue) ~= double(ui32NumSamples) || ...
        size(dAccPredTB, 2) ~= double(ui32NumSamples) || ...
        size(dAccTrueTB, 2) ~= double(ui32NumSamples) || ...
        numel(ui32ShellIds) ~= double(ui32NumSamples)
    error('ComputeGravityFieldFitMetrics:SizeMismatch', ...
        'All field arrays and ui32ShellIds must contain the same number of samples.');
end

% Compute pointwise norms and differences
dAccDiffNorm = vecnorm(dAccPredTB - dAccTrueTB, 2, 1);
dAccTrueNorm = vecnorm(dAccTrueTB, 2, 1);

dPotDiffAbs = abs(dPotentialPred - dPotentialTrue);
dPotTrueAbs = abs(dPotentialTrue);

% Compute global RMS and relative metrics
dAccRMS = sqrt(mean(dAccDiffNorm.^2));
dAccRMSrel = dAccRMS / max(sqrt(mean(dAccTrueNorm.^2)), eps);

dPotRMS = sqrt(mean(dPotDiffAbs.^2));
dPotRMSrel = dPotRMS / max(sqrt(mean(dPotTrueAbs.^2)), eps);

dAccPointRel = dAccDiffNorm ./ max(dAccTrueNorm, max(max(dAccTrueNorm), eps(1.0)));
dPotPointRel = dPotDiffAbs ./ max(dPotTrueAbs, max(max(dPotTrueAbs), eps(1.0)));

dAccShellRMSrel = NaN(1, double(ui32NumShells));
dPotShellRMSrel = NaN(1, double(ui32NumShells));

% Compute shell-wise relative RMS metrics
for idShell = 1:double(ui32NumShells)
    
    bMask = ui32ShellIds == uint32(idShell);
    dAccShellDiff = dAccDiffNorm(bMask);
    dAccShellTrue = dAccTrueNorm(bMask);
    dPotShellDiff = dPotDiffAbs(bMask);
    dPotShellTrue = dPotTrueAbs(bMask);

    dAccShellRMSrel(idShell) = sqrt(mean(dAccShellDiff.^2)) / ...
        max(sqrt(mean(dAccShellTrue.^2)), eps(1.0));

    dPotShellRMSrel(idShell) = sqrt(mean(dPotShellDiff.^2)) / ...
        max(sqrt(mean(dPotShellTrue.^2)), eps(1.0));
end

% Compile metrics into output struct
strMetrics = struct();
strMetrics.dAccRMS = dAccRMS;
strMetrics.dAccRMSrel = dAccRMSrel;
strMetrics.dAccMaxRel = max(dAccPointRel);
strMetrics.dPotentialRMS = dPotRMS;
strMetrics.dPotentialRMSrel = dPotRMSrel;
strMetrics.dPotentialMaxRel = max(dPotPointRel);
strMetrics.dAccShellRMSrel = dAccShellRMSrel;
strMetrics.dPotentialShellRMSrel = dPotShellRMSrel;
strMetrics.dAccPointRel = dAccPointRel(:);
strMetrics.dPotentialPointRel = dPotPointRel(:);

end
