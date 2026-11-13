function tests = testRenameLegacyTargetAttitudeFields
%% SIGNATURE
% tests = testRenameLegacyTargetAttitudeFields
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Check that saved attitude structs with the old target field names read back with the current names, that
% current and unrelated fields stay as they are, and that a struct with both names of one field is rejected.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    function-based test suite
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-11-2026  Pietro Califano, Claude Opus 5.5   First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RenameLegacyTargetAttitudeFields, EnumTargetAngularVelocityRegime.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testRenamesEveryOldField(testCase)
dRates_IN = [1.0, 2.0; 3.0, 4.0; 5.0, 6.0];
dRates_TB = [7.0, 8.0; 9.0, 10.0; 11.0, 12.0];
dTimegrid = [0.0, 10.0];
strSaved = struct('dAngVel_IN', dRates_IN, ...
                  'dAngVel_TB', dRates_TB, ...
                  'dAngVelTimegrid', dTimegrid, ...
                  'enumAngularVelocityRegime', EnumTargetAngularVelocityRegime.TIME_VARYING, ...
                  'dSpinAxis_TB', [0.0; 0.0; 1.0], ...
                  'charSpinAxisSource', "SCENARIO_DECLARED", ...
                  'dChbvPolycoeffs', ones(4, 3));

strAttitude = RenameLegacyTargetAttitudeFields(strSaved);

verifyEqual(testCase, sort(fieldnames(strAttitude)), sort({'dTargetAngVel_IN'; 'dTargetAngVel_TB'; ...
    'dTargetAngVelTimegrid'; 'enumTargetAngularVelocityRegime'; 'dTargetSpinAxis_TB'; ...
    'charTargetSpinAxisSource'; 'dChbvPolycoeffs'}));
verifyEqual(testCase, strAttitude.dTargetAngVel_IN, dRates_IN);
verifyEqual(testCase, strAttitude.dTargetAngVel_TB, dRates_TB);
verifyEqual(testCase, strAttitude.dTargetAngVelTimegrid, dTimegrid);
verifyEqual(testCase, strAttitude.enumTargetAngularVelocityRegime, EnumTargetAngularVelocityRegime.TIME_VARYING);
verifyEqual(testCase, strAttitude.dTargetSpinAxis_TB, [0.0; 0.0; 1.0]);
verifyEqual(testCase, strAttitude.charTargetSpinAxisSource, "SCENARIO_DECLARED");
verifyEqual(testCase, strAttitude.dChbvPolycoeffs, ones(4, 3));
end

function testKeepsCurrentNames(testCase)
strCurrent = struct('dTargetAngVel_IN', zeros(3, 2), ...
                    'enumTargetAngularVelocityRegime', EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL, ...
                    'ui32PolyDeg', uint32(5));

verifyEqual(testCase, RenameLegacyTargetAttitudeFields(strCurrent), strCurrent);
end

function testRejectsBothNames(testCase)
strMixed = struct('dAngVel_IN', zeros(3, 2), 'dTargetAngVel_IN', ones(3, 2));

verifyError(testCase, @() RenameLegacyTargetAttitudeFields(strMixed), ...
    'RenameLegacyTargetAttitudeFields:DuplicateField');
end
