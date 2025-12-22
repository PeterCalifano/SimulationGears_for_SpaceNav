function tests = testDatasetConversionFromSimStateArray
% Function-based test for conversions between datasets and CSimulationState arrays.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
charThisFile = mfilename('fullpath');
% Walk up to repository root from lib/SimulationGears_for_SpaceNav/tests/matlab
charRepoRoot = fileparts(fileparts(fileparts(fileparts(fileparts(charThisFile)))));

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

dTimestamps = [0, 10, 20];
dRelativeTimestamps = dTimestamps - dTimestamps(1);

dPosSC_W = [0, 1, 2; 10, 11, 12; 20, 21, 22];
dVelSC_W = [0.1, 0.2, 0.3; 0.4, 0.5, 0.6; 0.7, 0.8, 0.9];
dStateSC_W = [dPosSC_W; dVelSC_W];

dDCM_TBfromW = repmat(eye(3), 1, 1, numel(dTimestamps));
dDCM_SCfromW = dDCM_TBfromW;

dTargetPosition_W = 100 + dPosSC_W;
dSunPosition_W    = 1000 + dPosSC_W;
dEarthPosition_W  = 2 * ones(3, numel(dTimestamps));

dManoeuvresStartTimestamps = [5, 15];
dManoeuvresDeltaV_SC       = [0.1, 0.0; 0.0, 0.2; 0.0, 0.0];
dManoeuvresTimegrids       = [];

objDataset = SReferenceMissionDesign(EnumFrameName.IN, ...
                                     dTimestamps, ...
                                     dStateSC_W, ...
                                     dDCM_TBfromW, ...
                                     dTargetPosition_W, ...
                                     dSunPosition_W, ...
                                     dEarthPosition_W, ...
                                     "dDCM_SCfromW", dDCM_SCfromW, ...
                                     "dRelativeTimestamps", dRelativeTimestamps, ...
                                     "dManoeuvresStartTimestamps", dManoeuvresStartTimestamps, ...
                                     "dManoeuvresDeltaV_SC", dManoeuvresDeltaV_SC, ...
                                     "dManoeuvresTimegrids", dManoeuvresTimegrids);

[arrSimStates, dManStartOut, dManDeltaVOut, dManTimegridsOut] = SReferenceMissionDesign.toSimulationStates(objDataset);

objDatasetRoundtrip = SReferenceMissionDesign.fromSimulationStates(arrSimStates, ...
                                    "dManoeuvresStartTimestamps", dManStartOut, ...
                                    "dManoeuvresDeltaV_W", dManDeltaVOut, ...
                                    "dManoeuvresTimegrids", dManTimegridsOut, ...
                                    "dEarthPosition_W", dEarthPosition_W, ...
                                    "enumWorldFrame", objDataset.enumWorldFrame, ...
                                    "charLengthUnits", objDataset.charLengthUnits);

verifyEqual(testCase, numel(arrSimStates), numel(dTimestamps));
verifyEqual(testCase, objDatasetRoundtrip.dPosSC_W, objDataset.dPosSC_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objDatasetRoundtrip.dVelSC_W, objDataset.dVelSC_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objDatasetRoundtrip.dDCM_SCfromW, objDataset.dDCM_SCfromW, 'AbsTol', 1e-9);
verifyEqual(testCase, objDatasetRoundtrip.dDCM_TBfromW, objDataset.dDCM_TBfromW, 'AbsTol', 1e-9);
verifyEqual(testCase, objDatasetRoundtrip.dTargetPosition_W, objDataset.dTargetPosition_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objDatasetRoundtrip.dSunPosition_W, objDataset.dSunPosition_W, 'AbsTol', 1e-9);
verifyEqual(testCase, objDatasetRoundtrip.dRelativeTimestamps, objDataset.dRelativeTimestamps, 'AbsTol', 1e-12);
verifyEqual(testCase, objDatasetRoundtrip.dManoeuvresStartTimestamps, objDataset.dManoeuvresStartTimestamps);
verifyEqual(testCase, objDatasetRoundtrip.dManoeuvresDeltaV_SC, objDataset.dManoeuvresDeltaV_SC);
end
