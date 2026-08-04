function tests = testCreateOrHandleFig
%% SIGNATURE
% tests = testCreateOrHandleFig
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify figure creation and resolution contracts used by non-interactive plotting utilities.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% [-]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    Function-based test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 04-08-2026    Pietro Califano, Codex gpt 5.6     Add non-activating numbered-figure resolution regression.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CreateOrHandleFig_
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function setupOnce(~)
charRepositoryRoot = fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
run(fullfile(charRepositoryRoot, 'matlab', 'SetupSimGears.m'));
end

function testExistingHiddenFigureResolutionPreservesGraphicsState_(testCase)
objSentinelFig = figure('Visible', 'off');
objSentinelCleaner = onCleanup(@() CloseFigureIfValid_(objSentinelFig)); %#ok<NASGU>
objTargetFig = figure('Visible', 'off');
objTargetCleaner = onCleanup(@() CloseFigureIfValid_(objTargetFig)); %#ok<NASGU>
set(groot, 'CurrentFigure', objSentinelFig);

[objResolvedFig, ~] = CreateOrHandleFig_(gobjects(1), "painters", false, uint32(objTargetFig.Number));

verifyEqual(testCase, objResolvedFig, objTargetFig);
verifyEqual(testCase, get(groot, 'CurrentFigure'), objSentinelFig);
verifyEqual(testCase, string(objTargetFig.Visible), "off");
end

function CloseFigureIfValid_(objFig)
if isgraphics(objFig, 'figure')
    close(objFig);
end
end
