function tests = testDatasetConversionFromSimStates
% Function-based tests for conversions between datasets and CSimulationState arrays.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
charThisFile = mfilename('fullpath');
% Walk up to repository root from lib/SimulationGears_for_SpaceNav/tests/matlab
charRepoRoot = fullfile("../../../../../..", charThisFile);

addpath(fullfile(charRepoRoot, 'lib', 'SimulationGears_for_SpaceNav', 'matlab'));
addpath(fullfile(charRepoRoot, 'lib', 'SimulationGears_for_SpaceNav', 'matlab', 'general_utils'));
addpath(fullfile(charRepoRoot, 'lib', 'SimulationGears_for_SpaceNav', 'matlab', 'general_utils', 'datastructs'));

% Make sure nav-backend datastructs are reachable (CSimulationState, SNavState).
if exist('CSimulationState', 'class') ~= 8 || exist('SNavState', 'class') ~= 8
    charNavBackendDatastructs = fullfile(charRepoRoot, '..', '..', 'nav-backend', 'matlab', 'src', 'datastructs');
    if exist(charNavBackendDatastructs, 'dir')
        addpath(charNavBackendDatastructs);
    end
end

bDepsOK = exist('CSimulationState', 'class') == 8 && exist('SNavState', 'class') == 8;
assumeTrue(testCase, bDepsOK, 'CSimulationState/SNavState not found on path.');
end

function testRoundTripConversion(testCase)
[objDataset, data] = buildReferenceDataset();

[arrSimStates, dManStartOut, dManDeltaVOut, dManTimegridsOut] = SReferenceMissionDesign.toSimulationStates(objDataset);

objDatasetRoundtrip = SReferenceMissionDesign.FromSimulationStates(arrSimStates, ...
                                    "dManoeuvresStartTimestamps", dManStartOut, ...
                                    "dManoeuvresDeltaV_W", dManDeltaVOut, ...
                                    "dManoeuvresTimegrids", dManTimegridsOut, ...
                                    "dEarthPosition_W", data.dEarthPosition_W, ...
                                    "enumWorldFrame", objDataset.enumWorldFrame, ...
                                    "charLengthUnits", objDataset.charLengthUnits, ...
                                    "cellAdditionalBodiesTags", objDataset.cellAdditionalBodiesTags, ...
                                    "cellAdditionalTargetFrames", objDataset.cellAdditionalTargetFrames);

verifyEqual(testCase, numel(arrSimStates), numel(data.dTimestamps));
verifyMissionDatasetMatches(testCase, objDatasetRoundtrip, objDataset, 1e-9);
end

function testIntermediateRepresentationToMissionDesign(testCase)
[objDataset, data] = buildReferenceDataset();

[arrSimStates, dManStartOut, dManDeltaVOut, dManTimegridsOut] = SReferenceMissionDesign.toSimulationStates(objDataset);

assumeTrue(testCase, exist('cameraIntrinsics', 'class') == 8, 'cameraIntrinsics not available.');
objCamera = buildCamera();
bImageAcquisitionMask = logical([1, 0, 1]);

objIntermediate = SDatasetFromSimStateIntermediateRepr( ...
    uint32(numel(arrSimStates)), ...
    arrSimStates, ...
    objCamera, ...
    char(string(objDataset.enumWorldFrame)), ...
    dManTimegridsOut, ...
    dManStartOut, ...
    dManDeltaVOut, ...
    data.dEarthPosition_W, ...
    data.dRelativeTimestamps, ...
    objDataset.cellAdditionalBodiesTags, ...
    objDataset.cellAdditionalTargetFrames, ...
    objDataset.charLengthUnits, ...
    bImageAcquisitionMask);

objDatasetFromIntermediate = SReferenceMissionDesign.fromSimStatesIntermediateRepr(objIntermediate);

verifyMissionDatasetMatches(testCase, objDatasetFromIntermediate, objDataset, 1e-9);
end

function testIntermediateRepresentationToImagesDataset(testCase)
[objDataset, data] = buildReferenceDataset();

[arrSimStates, dManStartOut, dManDeltaVOut, dManTimegridsOut] = SReferenceMissionDesign.toSimulationStates(objDataset);

assumeTrue(testCase, exist('cameraIntrinsics', 'class') == 8, 'cameraIntrinsics not available.');
objCamera = buildCamera();
bImageAcquisitionMask = logical([1, 0, 1]);

objIntermediate = SDatasetFromSimStateIntermediateRepr( ...
    uint32(numel(arrSimStates)), ...
    arrSimStates, ...
    objCamera, ...
    char(string(objDataset.enumWorldFrame)), ...
    dManTimegridsOut, ...
    dManStartOut, ...
    dManDeltaVOut, ...
    data.dEarthPosition_W, ...
    data.dRelativeTimestamps, ...
    objDataset.cellAdditionalBodiesTags, ...
    objDataset.cellAdditionalTargetFrames, ...
    objDataset.charLengthUnits, ...
    bImageAcquisitionMask);

objImagesDataset = SReferenceImagesDataset.FromSimStatesIntermediateRepr(objIntermediate);

verifyEqual(testCase, string(objImagesDataset.enumWorldFrame), string(objDataset.enumWorldFrame));
verifyEqual(testCase, objImagesDataset.dPosSC_W, objDataset.dPosSC_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objImagesDataset.dVelSC_W, objDataset.dVelSC_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objImagesDataset.dDCM_SCfromW, objDataset.dDCM_SCfromW, 'AbsTol', 1e-9);
verifyEqual(testCase, objImagesDataset.dDCM_TBfromW, objDataset.dDCM_TBfromW, 'AbsTol', 1e-9);
verifyEqual(testCase, objImagesDataset.dTargetPosition_W, objDataset.dTargetPosition_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objImagesDataset.dSunPosition_W, objDataset.dSunPosition_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objImagesDataset.dRelativeTimestamps, objDataset.dRelativeTimestamps, 'AbsTol', 1e-12);
verifyEqual(testCase, objImagesDataset.dManoeuvresStartTimestamps, objDataset.dManoeuvresStartTimestamps);
verifyEqual(testCase, objImagesDataset.dManoeuvresDeltaV_SC, objDataset.dManoeuvresDeltaV_SC);
verifyEqual(testCase, objImagesDataset.objCamera.FocalLength, objCamera.FocalLength, 'AbsTol', 1e-12);
verifyEqual(testCase, objImagesDataset.objCamera.PrincipalPoint, objCamera.PrincipalPoint, 'AbsTol', 1e-12);
verifyEqual(testCase, objImagesDataset.bImageAcquisitionMask, bImageAcquisitionMask);
end

function [objDataset, data] = buildReferenceDataset()
data.dTimestamps = [0, 10, 20];
data.dRelativeTimestamps = [0, 12, 25];

data.dPosSC_W = [0, 1, 2; 10, 11, 12; 20, 21, 22];
data.dVelSC_W = [0.1, 0.2, 0.3; 0.4, 0.5, 0.6; 0.7, 0.8, 0.9];
data.dStateSC_W = [data.dPosSC_W; data.dVelSC_W];

data.dDCM_TBfromW = cat(3, rotZ(0.1), rotZ(0.2), rotZ(0.3));
data.dDCM_SCfromW = cat(3, rotZ(-0.1), rotZ(-0.2), rotZ(-0.3));

data.dTargetPosition_W = 100 + data.dPosSC_W;
data.dSunPosition_W    = 1000 + data.dPosSC_W;
data.dEarthPosition_W  = [2, 3, 4; 5, 6, 7; 8, 9, 10];

data.dManoeuvresStartTimestamps = [5, 15];
data.dManoeuvresDeltaV_SC       = [0.1, 0.0; 0.0, 0.2; 0.0, 0.0];
data.dManoeuvresTimegrids       = [];

data.cellAdditionalBodiesPos_W = { ...
    [10, 11, 12; 20, 21, 22; 30, 31, 32], ...
    [100, 110, 120; 200, 210, 220; 300, 310, 320] ...
    };
data.cellAdditionalBodiesDCM_TBfromW = { ...
    cat(3, rotX(0.05), rotX(0.1), rotX(0.15)), ...
    cat(3, rotZ(0.2), rotZ(0.4), rotZ(0.6)) ...
    };
data.cellAdditionalBodiesTags = {"Phobos", "Deimos"};
data.cellAdditionalTargetFrames = {"PHO", "DEI"};
data.charLengthUnits = "km";

objDataset = SReferenceMissionDesign(EnumFrameName.IN, ...
                                     data.dTimestamps, ...
                                     data.dStateSC_W, ...
                                     data.dDCM_TBfromW, ...
                                     data.dTargetPosition_W, ...
                                     data.dSunPosition_W, ...
                                     data.dEarthPosition_W, ...
                                     "dDCM_SCfromW", data.dDCM_SCfromW, ...
                                     "dRelativeTimestamps", data.dRelativeTimestamps, ...
                                     "dManoeuvresStartTimestamps", data.dManoeuvresStartTimestamps, ...
                                     "dManoeuvresDeltaV_SC", data.dManoeuvresDeltaV_SC, ...
                                     "dManoeuvresTimegrids", data.dManoeuvresTimegrids);

objDataset.cellAdditionalBodiesPos_W = data.cellAdditionalBodiesPos_W;
objDataset.cellAdditionalBodiesDCM_TBfromW = data.cellAdditionalBodiesDCM_TBfromW;
objDataset.cellAdditionalBodiesTags = data.cellAdditionalBodiesTags;
objDataset.cellAdditionalTargetFrames = data.cellAdditionalTargetFrames;
objDataset.charLengthUnits = data.charLengthUnits;
end

function verifyMissionDatasetMatches(testCase, actual, expected, tol)
verifyEqual(testCase, string(actual.enumWorldFrame), string(expected.enumWorldFrame));
verifyEqual(testCase, actual.dPosSC_W, expected.dPosSC_W, 'AbsTol', tol);
verifyEqual(testCase, actual.dVelSC_W, expected.dVelSC_W, 'AbsTol', tol);
verifyEqual(testCase, actual.dDCM_SCfromW, expected.dDCM_SCfromW, 'AbsTol', tol);
verifyEqual(testCase, actual.dDCM_TBfromW, expected.dDCM_TBfromW, 'AbsTol', tol);
verifyEqual(testCase, actual.dTargetPosition_W, expected.dTargetPosition_W, 'AbsTol', tol);
verifyEqual(testCase, actual.dSunPosition_W, expected.dSunPosition_W, 'AbsTol', tol);
verifyEqual(testCase, actual.dEarthPosition_W, expected.dEarthPosition_W, 'AbsTol', tol);
verifyEqual(testCase, actual.dRelativeTimestamps, expected.dRelativeTimestamps, 'AbsTol', 1e-12);
verifyEqual(testCase, actual.dManoeuvresStartTimestamps, expected.dManoeuvresStartTimestamps);
verifyEqual(testCase, actual.dManoeuvresDeltaV_SC, expected.dManoeuvresDeltaV_SC);
verifyEqual(testCase, actual.dManoeuvresTimegrids, expected.dManoeuvresTimegrids);
verifyEqual(testCase, string(actual.charLengthUnits), string(expected.charLengthUnits));
verifyEqual(testCase, string(actual.cellAdditionalBodiesTags), string(expected.cellAdditionalBodiesTags));
verifyEqual(testCase, string(actual.cellAdditionalTargetFrames), string(expected.cellAdditionalTargetFrames));

verifyCellArrayEqual(testCase, actual.cellAdditionalBodiesPos_W, expected.cellAdditionalBodiesPos_W, tol);
verifyCellArrayEqual(testCase, actual.cellAdditionalBodiesDCM_TBfromW, expected.cellAdditionalBodiesDCM_TBfromW, tol);
end

function verifyCellArrayEqual(testCase, actual, expected, tol)
verifyEqual(testCase, numel(actual), numel(expected));
for idx = 1:numel(expected)
    verifyEqual(testCase, actual{idx}, expected{idx}, 'AbsTol', tol);
end
end

function objCamera = buildCamera()
objCamera = CCameraIntrinsics([500, 600], [320, 240], [480, 640]);
end

function dDCM = rotZ(theta)
c = cos(theta);
s = sin(theta);
dDCM = [c, -s, 0; s, c, 0; 0, 0, 1];
end

function dDCM = rotX(theta)
c = cos(theta);
s = sin(theta);
dDCM = [1, 0, 0; 0, c, -s; 0, s, c];
end
