function objTests = testComputeSolarRadPressure()
%% SIGNATURE
% objTests = testComputeSolarRadPressure()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify reference-pressure scaling, physical unit equivalence, legacy calls, and invalid inputs.
% Use synthetic pressures rather than scenario configuration values.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Add SimulationGears sources through SetupSimGears before running the suite.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests   MATLAB unit tests for the shared solar-pressure contract.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026    Codex    Validate configured solar pressure independently of scenario profiles.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputeSolarRadPressure
% -------------------------------------------------------------------------------------------------------------

% Let functiontests inspect the test functions without argument-validation blocks.
objTests = functiontests(localfunctions);
end

function TestReferenceAtOneAU_(objTest)
dReferencePressure = 7.3e-6;
[dPressure, dReturnedReference] = ComputeSolarRadPressure(1 / 1.495978707e11, ...
                                                        false, dReferencePressure);

objTest.verifyEqual(dPressure, dReferencePressure, 'RelTol', 1e-14);
objTest.verifyEqual(dReturnedReference, dReferencePressure);
end

function TestInverseSquareDistance_(objTest)
dReferencePressure = 2.7e-6;
dDistance = 1.73 * 1.495978707e11;
dPressure = ComputeSolarRadPressure(1 / dDistance, false, dReferencePressure);
dFartherPressure = ComputeSolarRadPressure(1 / (2 * dDistance), false, dReferencePressure);

objTest.verifyEqual(dPressure, dReferencePressure / 1.73^2, 'RelTol', 1e-14);
objTest.verifyEqual(dFartherPressure, dPressure / 4, 'RelTol', 1e-14);
end

function TestConfiguredPressureScaling_(objTest)
dInvDistance = 1 / (0.81 * 1.495978707e11);
dReferencePressure = 8.1e-6;
dPressure = ComputeSolarRadPressure(dInvDistance, false, dReferencePressure);
[dScaledPressure, dScaledReference] = ComputeSolarRadPressure(dInvDistance, ...
                                                            false, 2.4 * dReferencePressure);

objTest.verifyEqual(dScaledPressure, 2.4 * dPressure, 'RelTol', 1e-14);
objTest.verifyEqual(dScaledReference, 2.4 * dReferencePressure);
end

function TestMetreKilometreEquivalence_(objTest)
dInvDistance = 1 / (1.91 * 1.495978707e11);
dReferencePressure = 3.2e-6;
[dPressureSI, dReferenceSI] = ComputeSolarRadPressure(dInvDistance, ...
                                                    false, dReferencePressure);
[dPressureKm, dReferenceKm] = ComputeSolarRadPressure(1e3 * dInvDistance, ...
                                                    true, 1e3 * dReferencePressure);

% Compare the same physical pressure after converting kilometre dynamics units to SI.
objTest.verifyEqual(dPressureKm / 1e3, dPressureSI, 'RelTol', 1e-14);
objTest.verifyEqual(dReferenceKm / 1e3, dReferenceSI, 'RelTol', 1e-14);
end

function TestZeroReferencePressure_(objTest)
[dPressure, dReference] = ComputeSolarRadPressure(1 / 1.495978707e11, false, 0);
objTest.verifyEqual(dPressure, 0);
objTest.verifyEqual(dReference, 0);
end

function TestLegacyNominalCalls_(objTest)
dInvDistance = 1 / (1.27 * 1.495978707e11);
dNominalReference = 1367 / 299792458;
[dPressureDefault, dReferenceDefault] = ComputeSolarRadPressure(dInvDistance);
[dPressureSI, dReferenceSI] = ComputeSolarRadPressure(dInvDistance, false);
[dPressureKm, dReferenceKm] = ComputeSolarRadPressure(1e3 * dInvDistance, true);

% Preserve the legacy nominal model without relying on any campaign settings.
objTest.verifyEqual(dPressureDefault, dNominalReference / 1.27^2, 'RelTol', 1e-14);
objTest.verifyEqual(dReferenceDefault, dNominalReference, 'RelTol', 1e-14);
objTest.verifyEqual(dPressureSI, dPressureDefault);
objTest.verifyEqual(dReferenceSI, dReferenceDefault);
objTest.verifyEqual(dPressureKm / 1e3, dPressureDefault, 'RelTol', 1e-14);
objTest.verifyEqual(dReferenceKm / 1e3, dReferenceDefault, 'RelTol', 1e-14);
end

function TestInvalidReferencePressure_(objTest)
dInvDistance = 1 / 1.495978707e11;
objTest.verifyError(@() ComputeSolarRadPressure(dInvDistance, false, -1), ...
                    'MATLAB:validators:mustBeNonnegative');
objTest.verifyError(@() ComputeSolarRadPressure(dInvDistance, false, Inf), ...
                    'MATLAB:validators:mustBeFinite');
objTest.verifyError(@() ComputeSolarRadPressure(dInvDistance, false, NaN), ...
                    'MATLAB:validators:mustBeFinite');
end

function TestInvalidInverseDistance_(objTest)
objTest.verifyError(@() ComputeSolarRadPressure(0, false, 2.3e-6), ...
                    'MATLAB:validators:mustBePositive');
objTest.verifyError(@() ComputeSolarRadPressure(-1, false, 2.3e-6), ...
                    'MATLAB:validators:mustBePositive');
objTest.verifyError(@() ComputeSolarRadPressure(Inf, false, 2.3e-6), ...
                    'MATLAB:validators:mustBeFinite');
objTest.verifyError(@() ComputeSolarRadPressure(NaN, false, 2.3e-6), ...
                    'MATLAB:validators:mustBeFinite');
end
