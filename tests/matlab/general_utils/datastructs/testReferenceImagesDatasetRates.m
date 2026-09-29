function objTests = testReferenceImagesDatasetRates()
%% SIGNATURE
% objTests = testReferenceImagesDatasetRates()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify source-rate preservation when constructing and converting image datasets.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Load the provider through SetupSimGears before running this suite.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests    Function-based MATLAB unit tests.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-09-2026  Pietro Califano, Codex gpt-6    Cover populated, zero and absent source rates.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SReferenceMissionDesign, SReferenceImagesDataset, CCameraIntrinsics
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testConversionPreservesSourceRates(objTestCase)
% Use identity attitudes so deriving a rate from attitude samples cannot satisfy this contract.
cellRates = {[], zeros(3,3), [0.1, 0.2, 0.3; -0.4, 0.0, 0.4; 0.0, 0.6, 0.7]};
for ui32Case = uint32(1):uint32(numel(cellRates))
    objMission = BuildMission_(cellRates{ui32Case});
    objImages = SReferenceImagesDataset.FromSReferenceMissionDesign(objMission);
    objTestCase.verifyEqual(objImages.dTargetAngVel_IN, objMission.dTargetAngVel_IN);
    objTestCase.verifyEqual(objImages.dTimestamps, objMission.dTimestamps);
    objTestCase.verifyEqual(objImages.dStateSC_W, objMission.dStateSC_W);
    objTestCase.verifyEqual(objImages.dDCM_TBfromW, objMission.dDCM_TBfromW);
    objTestCase.verifyEqual(objImages.dSunPosition_W, objMission.dSunPosition_W);
    objTestCase.verifyEqual(objImages.charLengthUnits, objMission.charLengthUnits);
end
end

function testConstructorRetainsDeclaredRates(objTestCase)
objMission = BuildMission_([0.1, 0.2, 0.3; -0.4, 0.0, 0.4; 0.0, 0.6, 0.7]);
objImages = SReferenceImagesDataset(CCameraIntrinsics(), objMission.enumWorldFrame, ...
    objMission.dTimestamps, objMission.dStateSC_W, objMission.dDCM_TBfromW, ...
    objMission.dTargetPosition_W, objMission.dSunPosition_W, objMission.dEarthPosition_W, ...
    dTargetAngVel_IN=objMission.dTargetAngVel_IN);
objTestCase.verifyEqual(objImages.dTargetAngVel_IN, objMission.dTargetAngVel_IN);
end

function objMission = BuildMission_(dRates)
%% DESCRIPTION
% Build distinct state samples and retain the supplied model rates without coupling them to attitude.
arguments (Input)
    dRates (3,:) double
end
arguments (Output)
    objMission (1,1) SReferenceMissionDesign
end
dTimegrid = [0.0, 15.0, 40.0];
objMission = SReferenceMissionDesign(EnumFrameName.IN, dTimegrid, ...
    reshape(1.0:18.0, 6, 3), repmat(eye(3),1,1,3), zeros(3,3), ...
    repmat([1.5e11;0.0;0.0],1,3), zeros(3,3), ...
    dTargetAngVel_IN=dRates);
objMission.charLengthUnits = 'm';
end
