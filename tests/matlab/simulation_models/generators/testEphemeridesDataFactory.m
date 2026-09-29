function tests = testEphemeridesDataFactory
%% SIGNATURE
% tests = testEphemeridesDataFactory
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that target-attitude ephemerides retain source-provided inertial angular velocity without recovering it
% from sampled attitudes.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB unit-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% 29-09-2026  Pietro Califano, Codex gpt-6      Cover both layouts, time domains and absent-rate reuse.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EphemeridesDataFactory, RotationVectorToDCM.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testCarriesSourceAngularVelocity(testCase)
dEphemerisTimegrid = linspace(100.0, 800.0, 16);
dNominalAngVel_IN = [0.7e-4; -1.1e-4; 1.9e-4];
dInitialDCM_INfromTB = RotationVectorToDCM([0.23; -0.17; 0.09]);
dDCM_INfromTB = zeros(3,3,numel(dEphemerisTimegrid));
for ui32TimeIdx = uint32(1):uint32(numel(dEphemerisTimegrid))
    dElapsedTime = dEphemerisTimegrid(ui32TimeIdx) - dEphemerisTimegrid(1);
    dDCM_INfromTB(:,:,ui32TimeIdx) = RotationVectorToDCM( ...
        -dElapsedTime .* dNominalAngVel_IN) * dInitialDCM_INfromTB;
end

strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = dDCM_INfromTB;
strMainBodyRefData.dAngVel_IN = repmat(dNominalAngVel_IN, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.dSpinAxis_TB = [0.0; 1.0; 0.0];
strMainBodyRefData.charSpinAxisSource = "SCENARIO_DECLARED";
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 2.0e7; -0.6e7], ...
    1, numel(dEphemerisTimegrid));
strDynParams = struct('strMainData', struct(), 'strBody3rdData', struct());

strDynParams = EphemeridesDataFactory(dEphemerisTimegrid, uint32(5), uint32(7), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false, ...
    'bUseAbsoluteTimegrid', true);

strAttitudeData = strDynParams.strMainData.strAttData;
verifyEqual(testCase, strAttitudeData.dNominalAngVel_IN, dNominalAngVel_IN, 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.dAngVel_IN, strMainBodyRefData.dAngVel_IN, 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.dAngVelTimegrid, dEphemerisTimegrid, 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.dTargetSpinAxis_TB, [0.0; 1.0; 0.0], 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.charTargetSpinAxisSource, "SCENARIO_DECLARED");
end

function testRejectsMalformedAngularVelocity(testCase)
dEphemerisTimegrid = linspace(0.0, 70.0, 8);
strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.dAngVel_IN = zeros(3, numel(dEphemerisTimegrid) - 1);
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 0.0; 0.0], ...
    1, numel(dEphemerisTimegrid));
strDynParams = struct('strMainData', struct(), 'strBody3rdData', struct());

verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetAngularVelocity');
end

function testRateMetadataAcrossLayoutsAndTimeDomains(objTestCase)
assumeTrue(objTestCase, exist('EphCoeffsGeneration', 'file') == 2, ...
    'Load the real external RCS ephemeris helpers before running this integration check.');
dTimegrid = linspace(100.0, 800.0, 16);
cellRateSequences = {zeros(3,16), ...
    [linspace(0.01,0.02,16); linspace(-0.03,0.01,16); linspace(0.02,0.04,16)]};

% Verify rate transport independently of coefficient layout and time-domain selection.
for bUseRcs = [false, true]
    for bAbsoluteTime = [false, true]
        for bScaleToDays = [false, true]
            for ui32Sequence = uint32(1):uint32(numel(cellRateSequences))
                strSource = BuildSource_(dTimegrid);
                strSource.dAngVel_IN = cellRateSequences{ui32Sequence};
                strOutput = EphemeridesDataFactory(dTimegrid, 3.0, 3.0, ...
                    struct('strMainData', struct(), 'strBody3rdData', struct()), strSource, [], ...
                    bUseInterpFcnFromRCS1=bUseRcs, bUseAbsoluteTimegrid=bAbsoluteTime, ...
                    bScaleTimeToDays=bScaleToDays, bEnableInterpValidation=false, ...
                    bAdd3rdBodiesPosition=false, bAdd3rdBodiesAttitude=false);
                strAttitude = strOutput.strMainData.strAttData;
                dExpectedGrid = dTimegrid;
                if bUseRcs || bAbsoluteTime
                    if bScaleToDays
                        dExpectedGrid = dTimegrid / 86400.0;
                    end
                else
                    dExpectedGrid = dTimegrid - dTimegrid(1);
                end
                objTestCase.verifyEqual(strAttitude.dAngVel_IN, strSource.dAngVel_IN);
                objTestCase.verifyEqual(strAttitude.dNominalAngVel_IN, strSource.dAngVel_IN(:,1));
                objTestCase.verifyEqual(strAttitude.dAngVelTimegrid, dExpectedGrid);
            end
        end
    end
end
end

function testAbsentRatesClearOnlyOwnedContext(objTestCase)
assumeTrue(objTestCase, exist('EphCoeffsGeneration', 'file') == 2, ...
    'Load the real external RCS ephemeris helpers before running this integration check.');
dTimegrid = linspace(100.0,800.0,16);
strSource = BuildSource_(dTimegrid);
strPreviousContext = struct('dAngVel_IN', ones(3,16), ...
    'dAngVelTimegrid', dTimegrid, 'dNominalAngVel_IN', ones(3,1), 'charConsumerTag', "retained");

% Reuse a dynamics payload without letting its previous source invent missing rate data.
for bUseRcs = [false, true]
    strOutput = EphemeridesDataFactory(dTimegrid, 3.0, 3.0, ...
        struct('strMainData', struct('strAttData', strPreviousContext), 'strBody3rdData', struct()), ...
        strSource, [], bUseInterpFcnFromRCS1=bUseRcs, bEnableInterpValidation=false, ...
        bAdd3rdBodiesPosition=false, bAdd3rdBodiesAttitude=false);
    strAttitude = strOutput.strMainData.strAttData;
    objTestCase.verifyFalse(isfield(strAttitude, 'dAngVel_IN'));
    objTestCase.verifyFalse(isfield(strAttitude, 'dNominalAngVel_IN'));
    objTestCase.verifyFalse(isfield(strAttitude, 'dAngVelTimegrid'));
    objTestCase.verifyEqual(strAttitude.charConsumerTag, strPreviousContext.charConsumerTag);
end
end

function testRejectsNonfiniteRatesInBothLayouts(objTestCase)
dTimegrid = linspace(100.0,800.0,16);
for bUseRcs = [false, true]
    for dInvalidRate = [NaN, Inf]
        strSource = BuildSource_(dTimegrid);
        strSource.dAngVel_IN = zeros(3,16);
        strSource.dAngVel_IN(2,7) = dInvalidRate;
        objTestCase.verifyError(@() EphemeridesDataFactory(dTimegrid, 3.0, 3.0, ...
            struct('strMainData', struct(), 'strBody3rdData', struct()), strSource, [], ...
            bUseInterpFcnFromRCS1=bUseRcs, bEnableInterpValidation=false, ...
            bAdd3rdBodiesPosition=false, bAdd3rdBodiesAttitude=false), ...
            'EphemeridesDataFactory:InvalidTargetAngularVelocity');
    end
end
end

function strSource = BuildSource_(dTimegrid)
%% DESCRIPTION
% Keep sampled attitude fixed so the rate tests require source transport, not numerical differencing.
arguments (Input)
    dTimegrid (1,:) double
end
arguments (Output)
    strSource (1,1) struct
end
strSource = struct('dDCM_INfromTB', repmat(eye(3),1,1,numel(dTimegrid)), ...
    'dSunPosition_IN', repmat([1.4e8;0.0;0.0],1,numel(dTimegrid)));
end
