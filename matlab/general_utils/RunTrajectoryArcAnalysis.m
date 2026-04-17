% RunTrajectoryArcAnalysis
%
% General-purpose entry point to inspect trajectory arcs and manoeuvre
% content either from loaded kernels or from user-provided state histories.

clear
close all
clc

%% User configuration
charSourceMode = "states";   % "states" | "kernel"
bUseManualPhases = false;
bShowFigures = true;
charStateLengthUnits = "m";  % "m" | "km"

% Manual phase table is used only when bUseManualPhases is true.
strPhaseTable = struct('charName', {}, 'dStartTime', {}, 'dEndTime', {});

% State-history mode
dTimestamps = 0:60:3600;
dPos_W = [cos(dTimestamps / 1200); sin(dTimestamps / 1200); 0.2 * sin(dTimestamps / 900)];
dVel_W = gradient(dPos_W', mean(diff(dTimestamps)))';
dSunPosition_W = repmat([1; 0; 0], 1, numel(dTimestamps));

% Kernel mode
varTargetID = int32(-99902);
charFrame = "J2000";
varObserver = int32(-658030);
charCorrection = "NONE";
charKernelLengthUnits = "km";
charCoverageKernelPath = '';
dCoverageIntervals = zeros(0, 2);
dTimegrid = zeros(1, 0);

%% Analysis
objAnalyzer = CTrajectoryArcAnalyzer();

if strcmpi(charSourceMode, "kernel")
    strAnalysis = objAnalyzer.analyzeKernelTrajectory( ...
        varTargetID, ...
        'dTimegrid', dTimegrid, ...
        'charFrame', charFrame, ...
        'varObserver', varObserver, ...
        'charCorrection', charCorrection, ...
        'charCoverageKernelPath', charCoverageKernelPath, ...
        'dCoverageIntervals', dCoverageIntervals, ...
        'strPhaseTable', strPhaseTable, ...
        'bUseManualPhases', bUseManualPhases);
else
    strAnalysis = objAnalyzer.analyzeStateHistory( ...
        dTimestamps, ...
        dPos_W, ...
        dVel_W, ...
        'dSunPosition_W', dSunPosition_W, ...
        'strPhaseTable', strPhaseTable, ...
        'bUseManualPhases', bUseManualPhases);
end

strPlotOutputs = PlotTrajectoryArcAnalysis( ... %#ok<NASGU>
    strAnalysis, ...
    'bShowFigures', bShowFigures, ...
    'charTitlePrefix', "Trajectory arc analysis");

if strcmpi(charSourceMode, "kernel")
    dMetersPerUnit = lengthScaleToMeters_(charKernelLengthUnits);
else
    dMetersPerUnit = lengthScaleToMeters_(charStateLengthUnits);
end

printArcAnalysisSummary_(strAnalysis, dMetersPerUnit);

function printArcAnalysisSummary_(strAnalysis, dMetersPerUnit)
fprintf('Trajectory arc analysis summary\n');
fprintf('  Segment 0 setup:\n');
printPhaseSetup_(strAnalysis.strPhases(1).dStartTime, ...
    strAnalysis.dPos_W(:, strAnalysis.strPhases(1).dStartIdx), ...
    strAnalysis.dVel_W(:, strAnalysis.strPhases(1).dStartIdx), ...
    dMetersPerUnit);

if ~isempty(strAnalysis.dManeuverTimes)
    fprintf('  Detected manoeuvres:\n');
    for idMan = 1:numel(strAnalysis.dManeuverTimes)
        dDeltaVMps = dMetersPerUnit * strAnalysis.dManeuverDeltaV_W(:, idMan);
        fprintf('    %2d  ET %.8g | dV [m/s] = %s | |dV| = %.8g\n', ...
            idMan, ...
            strAnalysis.dManeuverTimes(idMan), ...
            formatVector_(dDeltaVMps), ...
            norm(dDeltaVMps));

        if idMan + 1 <= numel(strAnalysis.strPhases)
            fprintf('\tPhase %d setup:\n', idMan);
            printPhaseSetup_(strAnalysis.strPhases(idMan + 1).dStartTime, ...
                strAnalysis.dPos_W(:, strAnalysis.strPhases(idMan + 1).dStartIdx), ...
                strAnalysis.dVel_W(:, strAnalysis.strPhases(idMan + 1).dStartIdx), ...
                dMetersPerUnit);
        end
    end
end
end

function printPhaseSetup_(dTimestamp, dPos, dVel, dMetersPerUnit)
fprintf('\tET [s]: %.8g\n', dTimestamp);
fprintf('\tPosition [m]: %s\n', formatVector_(dMetersPerUnit * dPos));
fprintf('\tVelocity [m/s]: %s\n', formatVector_(dMetersPerUnit * dVel));
end

function charVector = formatVector_(dVector)
cellEntries = arrayfun(@(dValue) sprintf('%.8g', dValue), dVector(:)', 'UniformOutput', false);
charVector = sprintf('[%s]', strjoin(cellEntries, ', '));
end

function dMetersPerUnit = lengthScaleToMeters_(charLengthUnits)
switch lower(char(charLengthUnits))
    case 'm'
        dMetersPerUnit = 1.0;
    case 'km'
        dMetersPerUnit = 1000.0;
    otherwise
        error('Unsupported length unit: %s', charLengthUnits);
end
end
