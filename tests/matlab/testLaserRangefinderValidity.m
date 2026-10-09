function tests = testLaserRangefinderValidity
%% SIGNATURE
% tests = testLaserRangefinderValidity
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that LiDAR range validity rejects measurements outside either bound of the configured interval.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% LaserRangefinderModel.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
charRepoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));
testCase.TestData.charRepoRoot = charRepoRoot;
end

function testRejectsRangeAboveUpperBound(testCase)
strTargetModelData = BuildSingleTriangleTarget_();

[dMeasuredRange, bIntersection, bValid] = LaserRangefinderModel( ...
    strTargetModelData, [1.0; 0.0; 0.0], [-10.0; 0.0; 0.0], 0.0, 0.0, ...
    zeros(3,1), [0.0; 5.0], false, false, true);

verifyTrue(testCase, bIntersection);
verifyGreaterThan(testCase, dMeasuredRange, 5.0);
verifyFalse(testCase, bValid);
end

function testRejectsRangeBelowLowerBound(testCase)
strTargetModelData = BuildSingleTriangleTarget_();

[dMeasuredRange, bIntersection, bValid] = LaserRangefinderModel( ...
    strTargetModelData, [1.0; 0.0; 0.0], [-10.0; 0.0; 0.0], 0.0, 0.0, ...
    zeros(3,1), [9.5; 20.0], false, false, true);

verifyTrue(testCase, bIntersection);
verifyLessThan(testCase, dMeasuredRange, 9.5);
verifyFalse(testCase, bValid);
end

function strTargetModelData = BuildSingleTriangleTarget_()
strTargetModelData = struct();
strTargetModelData.i32triangVertexPtrs = int32([1; 2; 3]);
strTargetModelData.dVerticesPositions = [-1.0, -1.0, -1.0; ...
                                         -1.0,  1.0,  0.0; ...
                                         -1.0, -1.0,  1.0];
end
