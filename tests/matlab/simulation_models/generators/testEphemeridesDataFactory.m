function tests = testEphemeridesDataFactory
%% SIGNATURE
% tests = testEphemeridesDataFactory
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that target-attitude ephemerides retain source-provided angular velocity and carry the validated typed
% angular-velocity regime without probing optional nominal-rate fields.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB unit-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% 22-08-2026  Pietro Califano, Codex              Require explicit constant-rate capability metadata.
% 23-08-2026  Pietro Califano, Codex              Replace nominal-rate field presence with a typed regime.
% 29-09-2026  Pietro Califano, Codex gpt-6      Cover both layouts, time domains and absent-rate reuse.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EphemeridesDataFactory, EnumTargetAngularVelocityRegime, RotationVectorToDCM.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testCarriesSourceAngularVelocity(testCase)
dEphemerisTimegrid = linspace(100.0, 800.0, 16);
dConstantAngVel_IN = [0.7e-4; -1.1e-4; 1.9e-4];
dInitialDCM_INfromTB = RotationVectorToDCM([0.23; -0.17; 0.09]);
dDCM_INfromTB = zeros(3, 3, numel(dEphemerisTimegrid));
for ui32TimeIdx = uint32(1):uint32(numel(dEphemerisTimegrid))
    dElapsedTime = dEphemerisTimegrid(ui32TimeIdx) - dEphemerisTimegrid(1);
    dDCM_INfromTB(:, :, ui32TimeIdx) = RotationVectorToDCM( ...
        dElapsedTime .* dConstantAngVel_IN) * dInitialDCM_INfromTB;
end

strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = dDCM_INfromTB;
strMainBodyRefData.dAngVel_IN = repmat(dConstantAngVel_IN, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL;
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
verifyEqual(testCase, strAttitudeData.enumAngularVelocityRegime, ...
    EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL);
verifyFalse(testCase, isfield(strAttitudeData, 'dNominalAngVel_IN'));
verifyEqual(testCase, strAttitudeData.dAngVel_IN, strMainBodyRefData.dAngVel_IN, 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.dAngVelTimegrid, dEphemerisTimegrid, 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.dTargetSpinAxis_TB, [0.0; 1.0; 0.0], 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.charTargetSpinAxisSource, "SCENARIO_DECLARED");
end

function testVariableRateOmitsNominalCapability(testCase)
dEphemerisTimegrid = linspace(0.0, 70.0, 8);
strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.dAngVel_IN = [linspace(0.01, 0.02, numel(dEphemerisTimegrid)); ...
                                 linspace(-0.03, 0.01, numel(dEphemerisTimegrid)); ...
                                 linspace(0.02, 0.04, numel(dEphemerisTimegrid))];
strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.TIME_VARYING;
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 0.0; 0.0], ...
    1, numel(dEphemerisTimegrid));
strDynParams = struct('strMainData', struct(), 'strBody3rdData', struct());

strDynParams = EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false);

verifyEqual(testCase, ...
    strDynParams.strMainData.strAttData.enumAngularVelocityRegime, ...
    EnumTargetAngularVelocityRegime.TIME_VARYING);
verifyFalse(testCase, isfield(strDynParams.strMainData.strAttData, ...
    'dNominalAngVel_IN'));
verifyEqual(testCase, strDynParams.strMainData.strAttData.dAngVel_IN, ...
    strMainBodyRefData.dAngVel_IN, 'AbsTol', 0.0);
verifyFalse(testCase, isfield(strDynParams.strMainData.strAttData, ...
    'dTargetSpinAxis_TB'));
verifyFalse(testCase, isfield(strDynParams.strMainData.strAttData, ...
    'charTargetSpinAxisSource'));
end

function testVariableRateClearsSeededNominalCapability(testCase)
dEphemerisTimegrid = linspace(0.0, 70.0, 8);
strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.dAngVel_IN = [linspace(0.01, 0.02, numel(dEphemerisTimegrid)); ...
                                 linspace(-0.03, 0.01, numel(dEphemerisTimegrid)); ...
                                 linspace(0.02, 0.04, numel(dEphemerisTimegrid))];
strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.TIME_VARYING;
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 0.0; 0.0], ...
    1, numel(dEphemerisTimegrid));
strSeededAttitudeData = struct( ...
    'dAngVel_IN', repmat([9.0; 8.0; 7.0], 1, numel(dEphemerisTimegrid)), ...
    'dAngVelTimegrid', dEphemerisTimegrid + 1000.0, ...
    'dAngVel_TB', repmat([6.0; 5.0; 4.0], 1, numel(dEphemerisTimegrid)), ...
    'dNominalAngVel_IN', [9.0; 8.0; 7.0], ...
    'dTargetSpinAxis_TB', [1.0; 0.0; 0.0], ...
    'charTargetSpinAxisSource', "STALE_SOURCE");
strDynParams = struct('strMainData', struct('strAttData', strSeededAttitudeData), ...
                      'strBody3rdData', struct());

strDynParams = EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false);

strAttitudeData = strDynParams.strMainData.strAttData;
verifyEqual(testCase, strAttitudeData.enumAngularVelocityRegime, ...
    EnumTargetAngularVelocityRegime.TIME_VARYING);
verifyFalse(testCase, isfield(strAttitudeData, 'dNominalAngVel_IN'));
verifyFalse(testCase, isfield(strAttitudeData, 'dAngVel_TB'));
verifyFalse(testCase, isfield(strAttitudeData, 'dTargetSpinAxis_TB'));
verifyFalse(testCase, isfield(strAttitudeData, 'charTargetSpinAxisSource'));
verifyEqual(testCase, strAttitudeData.dAngVel_IN, ...
    strMainBodyRefData.dAngVel_IN, 'AbsTol', 0.0);
verifyEqual(testCase, strAttitudeData.dAngVelTimegrid, ...
    dEphemerisTimegrid, 'AbsTol', 0.0);
end

function testRejectsInconsistentAngularVelocityRegime(testCase)
dEphemerisTimegrid = linspace(0.0, 70.0, 8);
strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.dAngVel_IN = [linspace(0.01, 0.02, numel(dEphemerisTimegrid)); ...
                                 zeros(1, numel(dEphemerisTimegrid)); ...
                                 zeros(1, numel(dEphemerisTimegrid))];
strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL;
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 0.0; 0.0], ...
    1, numel(dEphemerisTimegrid));
strDynParams = struct('strMainData', struct(), 'strBody3rdData', struct());

verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InconsistentAngularVelocityRegime');

strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.UNAVAILABLE;
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InconsistentAngularVelocityRegime');
end

function testRejectsMalformedAngularVelocity(testCase)
dEphemerisTimegrid = linspace(0.0, 70.0, 8);
strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.dAngVel_IN = zeros(3, numel(dEphemerisTimegrid) - 1);
strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.TIME_VARYING;
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 0.0; 0.0], ...
    1, numel(dEphemerisTimegrid));
strDynParams = struct('strMainData', struct(), 'strBody3rdData', struct());

verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetAngularVelocity');

strMainBodyRefData.dAngVel_IN = repmat('a', 3, numel(dEphemerisTimegrid));
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetAngularVelocity');

strMainBodyRefData.dAngVel_IN = [linspace(0.01, 0.02, numel(dEphemerisTimegrid)); ...
                                 zeros(1, numel(dEphemerisTimegrid)); ...
                                 zeros(1, numel(dEphemerisTimegrid))];
strMainBodyRefData.dAngVel_TB = complex( ...
    zeros(3, numel(dEphemerisTimegrid)), ...
    ones(3, numel(dEphemerisTimegrid)));
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetBodyAngularVelocity');
end

function testRejectsIncompleteSpinAxisMetadata(testCase)
dEphemerisTimegrid = linspace(0.0, 70.0, 8);
strMainBodyRefData = struct();
strMainBodyRefData.dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dEphemerisTimegrid));
strMainBodyRefData.enumAngularVelocityRegime = ...
    EnumTargetAngularVelocityRegime.UNAVAILABLE;
strMainBodyRefData.dSunPosition_IN = repmat([1.4e8; 0.0; 0.0], ...
    1, numel(dEphemerisTimegrid));
strMainBodyRefData.dSpinAxis_TB = [0.0; 0.0; 1.0];
strDynParams = struct('strMainData', struct(), 'strBody3rdData', struct());

verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:IncompleteTargetSpinAxisMetadata');

strMainBodyRefData = rmfield(strMainBodyRefData, 'dSpinAxis_TB');
strMainBodyRefData.charSpinAxisSource = "SCENARIO_DECLARED";
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:IncompleteTargetSpinAxisMetadata');

strMainBodyRefData.dSpinAxis_TB = [1.0i; 0.0; 0.0];
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetSpinAxis');

strMainBodyRefData.dSpinAxis_TB = [0.0; 0.0; 1.0];
strMainBodyRefData.charSpinAxisSource = 42;
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetSpinAxis');

strMainBodyRefData.charSpinAxisSource = "   ";
verifyError(testCase, @() EphemeridesDataFactory(dEphemerisTimegrid, uint32(3), uint32(3), ...
    strDynParams, strMainBodyRefData, [], 'bEnableInterpValidation', false, ...
    'bAdd3rdBodiesPosition', false, 'bAdd3rdBodiesAttitude', false), ...
    'EphemeridesDataFactory:InvalidTargetSpinAxis');
end

function testRateMetadataAcrossLayoutsAndTimeDomains(objTestCase)
assumeTrue(objTestCase, exist('EphCoeffsGeneration', 'file') == 2, ...
    'Load the real external RCS ephemeris helpers before running this integration check.');
dTimegrid = linspace(100.0, 800.0, 16);
cellRateSequences = {zeros(3, 16), ...
    [linspace(0.01, 0.02, 16); linspace(-0.03, 0.01, 16); linspace(0.02, 0.04, 16)]};

% Verify rate transport independently of coefficient layout and time-domain selection.
for bUseRcs = [false, true]
    for bAbsoluteTime = [false, true]
        for bScaleToDays = [false, true]
            for ui32SequenceIdx = uint32(1):uint32(numel(cellRateSequences))
                strSource = BuildSource_(dTimegrid);
                strSource.dAngVel_IN = cellRateSequences{ui32SequenceIdx};
                strSource.enumAngularVelocityRegime = EnumTargetAngularVelocityRegime.Classify(strSource.dAngVel_IN);
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
                objTestCase.verifyEqual(strAttitude.enumAngularVelocityRegime, strSource.enumAngularVelocityRegime);
                objTestCase.verifyFalse(isfield(strAttitude, 'dNominalAngVel_IN'));
                objTestCase.verifyEqual(strAttitude.dAngVelTimegrid, dExpectedGrid);
            end
        end
    end
end
end

function testAbsentRatesClearOnlyOwnedContext(objTestCase)
assumeTrue(objTestCase, exist('EphCoeffsGeneration', 'file') == 2, ...
    'Load the real external RCS ephemeris helpers before running this integration check.');
dTimegrid = linspace(100.0, 800.0, 16);
strSource = BuildSource_(dTimegrid);
strPreviousContext = struct('dAngVel_IN', ones(3, 16), ...
    'dAngVelTimegrid', dTimegrid, 'dNominalAngVel_IN', ones(3, 1), 'charConsumerTag', "retained");

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
dTimegrid = linspace(100.0, 800.0, 16);
for bUseRcs = [false, true]
    for dInvalidRate = [NaN, Inf]
        strSource = BuildSource_(dTimegrid);
        strSource.dAngVel_IN = zeros(3, 16);
        strSource.dAngVel_IN(2, 7) = dInvalidRate;
        strSource.enumAngularVelocityRegime = EnumTargetAngularVelocityRegime.TIME_VARYING;
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
strSource = struct('dDCM_INfromTB', repmat(eye(3), 1, 1, numel(dTimegrid)), ...
    'dSunPosition_IN', repmat([1.4e8; 0.0; 0.0], 1, numel(dTimegrid)), ...
    'enumAngularVelocityRegime', EnumTargetAngularVelocityRegime.UNAVAILABLE);
end
