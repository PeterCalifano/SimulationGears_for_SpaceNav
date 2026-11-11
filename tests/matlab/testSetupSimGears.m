function objTests = testSetupSimGears()
%% SIGNATURE
% objTests = testSetupSimGears()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify the opt-in external directories of SetupSimGears: set variables add the expected folders at
% the expected end of the path, a wrong root fails with its own error identifier.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Cover the Spectra-RT and CommManager4MATLAB variables.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SetupSimGears.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function setup(testCase)
% Restore the path and the three variables after each test; start every test with them unset.
charOriginalPath = path;
cellVariables = {'SIM_GEARS_CAMERA_NOISE_MEX_DIR', 'SIM_GEARS_SPECTRA_RT_DIR', 'SIM_GEARS_COMM_MANAGER_DIR'};
cellOriginalValues = cellfun(@getenv, cellVariables, 'UniformOutput', false);
testCase.addTeardown(@() RestoreState_(charOriginalPath, cellVariables, cellOriginalValues));

for ui32VariableIdx = uint32(1):uint32(numel(cellVariables))
    setenv(cellVariables{ui32VariableIdx}, '');
end

charRoot = tempname;
mkdir(charRoot);
testCase.addTeardown(@() rmdir(charRoot, 's'));
testCase.TestData.charRoot = charRoot;
end

function testSpectraBuildAddsBothWrapperFoldersBeforeSimulationGears(testCase)
% A Spectra-RT build root puts wrap/spectra_rt and wrap/spectra_rt_mex ahead of the SimulationGears folders.
charRoot = testCase.TestData.charRoot;
mkdir(fullfile(charRoot, 'wrap', 'spectra_rt'));
mkdir(fullfile(charRoot, 'wrap', 'spectra_rt_mex'));
setenv('SIM_GEARS_SPECTRA_RT_DIR', charRoot);

SetupSimGears();

cellPath = strsplit(path, pathsep);
ui32WrapperIdx = uint32(find(strcmp(cellPath, fullfile(charRoot, 'wrap', 'spectra_rt'))));
ui32MexIdx = uint32(find(strcmp(cellPath, fullfile(charRoot, 'wrap', 'spectra_rt_mex'))));
ui32SetupIdx = uint32(find(strcmp(cellPath, fileparts(which('SetupSimGears')))));
verifyNumElements(testCase, ui32WrapperIdx, 1);
verifyNumElements(testCase, ui32MexIdx, 1);
verifyLessThan(testCase, ui32WrapperIdx, ui32SetupIdx);
verifyLessThan(testCase, ui32MexIdx, ui32SetupIdx);
end

function testSpectraRootWithoutWrapperFails(testCase)
% A root without the wrapper folders fails and adds none of its folders.
setenv('SIM_GEARS_SPECTRA_RT_DIR', testCase.TestData.charRoot);

verifyError(testCase, @() SetupSimGears(), 'SetupSimGears:MissingSpectraRtDirectory');
verifyFalse(testCase, contains(path, testCase.TestData.charRoot));
end

function testCommManagerGoesToTheEndWithSubfolders(testCase)
% A CommManager4MATLAB root adds src and its subfolders after every SimulationGears folder.
charRoot = testCase.TestData.charRoot;
mkdir(fullfile(charRoot, 'src', 'utils'));
setenv('SIM_GEARS_COMM_MANAGER_DIR', charRoot);

SetupSimGears();

cellPath = strsplit(path, pathsep);
ui32SrcIdx = uint32(find(strcmp(cellPath, fullfile(charRoot, 'src'))));
ui32UtilsIdx = uint32(find(strcmp(cellPath, fullfile(charRoot, 'src', 'utils'))));
ui32SetupIdx = uint32(find(strcmp(cellPath, fileparts(which('SetupSimGears')))));
verifyNumElements(testCase, ui32SrcIdx, 1);
verifyNumElements(testCase, ui32UtilsIdx, 1);
verifyGreaterThan(testCase, ui32SrcIdx, ui32SetupIdx);
verifyGreaterThan(testCase, ui32UtilsIdx, ui32SetupIdx);
end

function testUnsetVariablesAddNothing(testCase)
% With every variable unset, no folder under the test root reaches the path.
mkdir(fullfile(testCase.TestData.charRoot, 'wrap', 'spectra_rt'));

SetupSimGears();

verifyFalse(testCase, contains(path, testCase.TestData.charRoot));
end

function RestoreState_(charOriginalPath, cellVariables, cellOriginalValues)
% Put back the MATLAB path and the environment variables.
arguments (Input)
    charOriginalPath (1,:) char
    cellVariables (1,:) cell
    cellOriginalValues (1,:) cell
end

path(charOriginalPath);
for ui32VariableIdx = uint32(1):uint32(numel(cellVariables))
    setenv(cellVariables{ui32VariableIdx}, cellOriginalValues{ui32VariableIdx});
end
end
