function tests = testCTrajectoryArcAnalyzer
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
charStageRoot = fullfile(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))), 'matlab');
addpath(genpath(charStageRoot));
testCase.TestData.charStageRoot = charStageRoot;
end

function setup(testCase)
set(groot, 'defaultFigureVisible', 'off');
close all force;
addTeardown(testCase, @close, 'all', 'force');
addTeardown(testCase, @set, groot, 'defaultFigureVisible', 'on');
end

function testDetectPhasesFromVelocityJump(testCase)
[dTimestamps, dPos_W, dVel_W, dSunPosition_W] = buildSyntheticStateHistory_();

objAnalyzer = CTrajectoryArcAnalyzer('dGroupingGap_s', 15.0);
strAnalysis = objAnalyzer.analyzeStateHistory( ...
    dTimestamps, dPos_W, dVel_W, ...
    'dSunPosition_W', dSunPosition_W, ...
    'dVelocityJumpThreshold', 0.05);

verifyEqual(testCase, numel(strAnalysis.strPhases), 3);
verifyEqual(testCase, numel(strAnalysis.dManeuverTimes), 2);
verifyEqual(testCase, size(strAnalysis.dPhaseIntervals), [3, 2]);
verifyEqual(testCase, size(strAnalysis.dManeuverDeltaV_W), [3, 2]);
verifyFalse(testCase, isempty(strAnalysis.dPhaseAngleDeg));
end

function testManualPhaseValidation(testCase)
[dTimestamps, dPos_W, dVel_W, dSunPosition_W] = buildSyntheticStateHistory_();

strPhaseTable = struct( ...
    'charName', {'Warmup', 'Tracking', 'Departure'}, ...
    'dStartTime', {dTimestamps(1), dTimestamps(4), dTimestamps(7)}, ...
    'dEndTime', {dTimestamps(4), dTimestamps(7), dTimestamps(end)});

objAnalyzer = CTrajectoryArcAnalyzer('dGroupingGap_s', 15.0);
strAnalysis = objAnalyzer.analyzeStateHistory( ...
    dTimestamps, dPos_W, dVel_W, ...
    'dSunPosition_W', dSunPosition_W, ...
    'bUseManualPhases', true, ...
    'strPhaseTable', strPhaseTable);

verifyEqual(testCase, string({strAnalysis.strPhases.charName}), ["Warmup", "Tracking", "Departure"]);
verifyEqual(testCase, numel(strAnalysis.dManeuverTimes), 2);
verifyEqual(testCase, size(strAnalysis.dManeuverDeltaV_W), [3, 2]);
end

function testInvalidManualPhaseTableFails(testCase)
[dTimestamps, dPos_W, dVel_W, ~] = buildSyntheticStateHistory_();

strPhaseTable = struct( ...
    'charName', {'BadA', 'BadB'}, ...
    'dStartTime', {dTimestamps(1), dTimestamps(5)}, ...
    'dEndTime', {dTimestamps(6), dTimestamps(end)});

objAnalyzer = CTrajectoryArcAnalyzer('dGroupingGap_s', 15.0);
try
    objAnalyzer.analyzeStateHistory( ...
        dTimestamps, dPos_W, dVel_W, ...
        'bUseManualPhases', true, ...
        'strPhaseTable', strPhaseTable);
    verifyFail(testCase, 'Expected analyzeStateHistory to reject overlapping manual phases.');
catch ME
    verifyNotEmpty(testCase, ME.message);
    verifyTrue(testCase, contains(string(ME.message), "must not overlap"));
end
end

function testPlotTrajectoryArcAnalysis(testCase)
[dTimestamps, dPos_W, dVel_W, dSunPosition_W] = buildSyntheticStateHistory_();

objAnalyzer = CTrajectoryArcAnalyzer('dGroupingGap_s', 15.0);
strAnalysis = objAnalyzer.analyzeStateHistory( ...
    dTimestamps, dPos_W, dVel_W, ...
    'dSunPosition_W', dSunPosition_W, ...
    'dVelocityJumpThreshold', 0.05);

strPlotOutputs = PlotTrajectoryArcAnalysis( ...
    strAnalysis, ...
    'bShowFigures', false, ...
    'charTitlePrefix', "Synthetic arc analysis");

verifyTrue(testCase, isgraphics(strPlotOutputs.objSegmentFig, 'figure'));
verifyTrue(testCase, isgraphics(strPlotOutputs.objTimelineFig, 'figure'));
verifyTrue(testCase, isgraphics(strPlotOutputs.objPhaseTrajectoryFig, 'figure'));
end

function [dTimestamps, dPos_W, dVel_W, dSunPosition_W] = buildSyntheticStateHistory_()
dTimestamps = 0:10:90;
dPos_W = [ ...
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9; ...
    0, 0.4, 0.8, 1.2, 1.3, 1.4, 1.8, 2.2, 2.4, 2.6; ...
    0, 0.1, 0.2, 0.3, 0.7, 1.1, 1.2, 1.3, 1.4, 1.5];

dVel_W = [ ...
    0.10, 0.10, 0.10, 0.10, 0.25, 0.25, 0.25, 0.05, 0.05, 0.05; ...
    0.02, 0.02, 0.02, 0.02, 0.08, 0.08, 0.08, 0.01, 0.01, 0.01; ...
    0.01, 0.01, 0.01, 0.01, 0.04, 0.04, 0.04, 0.02, 0.02, 0.02];

dSunPosition_W = repmat([1.0; 0.3; 0.1], 1, numel(dTimestamps));
end
