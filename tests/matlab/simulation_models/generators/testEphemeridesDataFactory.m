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
