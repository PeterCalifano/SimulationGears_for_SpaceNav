function tests = testTargetRotationalStateModels
%% SIGNATURE
% tests = testTargetRotationalStateModels
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify typed target rotational-state configuration, source and generated history construction, torque-free
% rigid-body invariants and closed-form solutions, and deterministic application of externally sampled
% perturbations.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB unit-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% 23-08-2026  Pietro Califano, Codex     Enforce the declared body-to-inertial attitude direction.
% 24-08-2026  Pietro Califano, Codex     Verify explicit backend selection and fail-fast MEX behavior.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% 12-11-2026  Pietro Califano, Claude Opus 5.5   Add the axisymmetric and Jacobi elliptic torque-free tests;
%                                                expect the shared realization and inertia error identifiers.
% 13-11-2026  Pietro Califano, Claude Opus 5.5   Prefix the target rate and spin-axis fields with Target.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% STargetRotationalStateConfig, BuildTargetRotationalStateHistory,
% ApplyTargetRotationalStatePerturbation, RotationVectorToDCM, ellipj, ellipke.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testConfigRoundTripAndStrictValidation(testCase)
% Parse a torque-free configuration and serialize it twice
strConfig = BuildTorqueFreeConfigStruct_();
objConfig = STargetRotationalStateConfig.FromConfigStruct(strConfig);
objRoundTrip = STargetRotationalStateConfig.FromConfigStruct(objConfig.toConfigStruct());

verifyEqual(testCase, objConfig.enumSource, EnumTargetRotationalStateSource.GENERATED);
verifyEqual(testCase, objConfig.enumDynamicsModel, EnumTargetRotationalDynamicsModel.RIGID_BODY_TORQUE_FREE);
verifyEqual(testCase, objConfig.dInitialQuat_INfromTB, [1.0; 0.0; 0.0; 0.0], 'AbsTol', 0.0);
verifyEqual(testCase, objConfig.dInitialAngVel_TB, [0.11; -0.07; 0.24], 'AbsTol', 0.0);
verifyEqual(testCase, objRoundTrip.toConfigStruct(), objConfig.toConfigStruct());

% Reject unknown fields, mismatched source and model, and generated fields on source-provided rotation
strUnknownConfig = strConfig;
strUnknownConfig.unowned_policy = true;
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strUnknownConfig), ...
    'STargetRotationalStateConfig:UnknownField');

strBadSourceModel = struct('source', 'spice', ...
    'dynamics_model', 'rigid_body_torque_free');
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strBadSourceModel), ...
    'STargetRotationalStateConfig:InvalidSourceModel');

strSourceWithGeneratedStep = struct('source', 'dataset', ...
    'dynamics_model', 'source_provided', ...
    'max_integration_step_s', 0.25);
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strSourceWithGeneratedStep), ...
    'STargetRotationalStateConfig:UnexpectedGeneratedField');

objProgrammaticSourceConfig = STargetRotationalStateConfig.FromConfigStruct( ...
    struct('source', 'spice', 'dynamics_model', 'source_provided'));
objProgrammaticSourceConfig.dMaxIntegrationStep = 0.25;
verifyError(testCase, @() objProgrammaticSourceConfig.toConfigStruct(), ...
    'STargetRotationalStateConfig:UnexpectedGeneratedField');

% Reject a nonphysical inertia and a nonnumeric step
strInvalidInertia = strConfig;
strInvalidInertia.inertia_tensor_TB_kg_m2 = diag([1.0, 1.0, 3.0]);
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strInvalidInertia), ...
    'ValidatePhysicalInertia:InvalidInertiaTensor');

strNonnumericIntegrationStep = strConfig;
strNonnumericIntegrationStep.max_integration_step_s = '1';
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strNonnumericIntegrationStep), ...
    'STargetRotationalStateConfig:InvalidMaxIntegrationStep');

% A perturbed configuration round-trips with its provenance
objAppliedConfig = ApplyTargetRotationalStatePerturbation(objConfig, BuildPerturbationRealization_());
strPerturbedConfig = objAppliedConfig.toConfigStruct();
objPerturbedConfig = STargetRotationalStateConfig.FromConfigStruct(strPerturbedConfig);
verifyEqual(testCase, objPerturbedConfig.toConfigStruct(), strPerturbedConfig);

% Reject provenance that is mismatched, incomplete, extended, missing or empty
strMismatchedProvenance = strPerturbedConfig;
strMismatchedProvenance.perturbation_provenance.ui32Seed = uint32(1235);
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strMismatchedProvenance), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strIncompleteProvenance = strPerturbedConfig;
strIncompleteProvenance.perturbation_provenance = rmfield(strIncompleteProvenance.perturbation_provenance, ...
    'dInertiaChartError');
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strIncompleteProvenance), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strUnknownProvenance = strPerturbedConfig;
strUnknownProvenance.perturbation_provenance.dUndocumentedError = zeros(3, 1);
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strUnknownProvenance), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strSeedWithoutProvenance = strConfig;
strSeedWithoutProvenance.realization_seed = uint32(1234);
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strSeedWithoutProvenance), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strZeroSeedWithoutProvenance = strConfig;
strZeroSeedWithoutProvenance.realization_seed = uint32(0);
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strZeroSeedWithoutProvenance), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strEmptyRealization = strConfig;
strEmptyRealization.realization_seed = uint32(0);
strEmptyRealization.perturbation_provenance = struct();
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strEmptyRealization), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

% Constant-rate truth cannot carry an inertia chart error
strConstantPerturbedConfig = struct('source', 'generated', ...
    'dynamics_model', 'kinematic_constant_inertial_rate', ...
    'initial_quaternion_INfromTB', [1.0, 0.0, 0.0, 0.0], ...
    'initial_angular_velocity_IN_rad_s', [0.01, 0.02, 0.03], ...
    'realization_seed', uint32(1234), ...
    'perturbation_provenance', BuildPerturbationRealization_());
verifyError(testCase, @() STargetRotationalStateConfig.FromConfigStruct(strConstantPerturbedConfig), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');
end

function testGeneratedConstantInertialRateHistory(testCase)
dTimegrid = [100.0, 108.0, 121.0, 145.0];
dInitialDCM_INfromTB = RotationVectorToDCM([0.21; -0.14; 0.08]);
dInitialQuat_INfromTB = DCM2quatSeq(dInitialDCM_INfromTB, false);
dInitialAngVel_IN = [0.012; -0.017; 0.023];

objConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'generated', ...
    'dynamics_model', 'kinematic_constant_inertial_rate', ...
    'initial_quaternion_INfromTB', transpose(dInitialQuat_INfromTB), ...
    'initial_angular_velocity_IN_rad_s', transpose(dInitialAngVel_IN)));
strSerializedConfig = objConfig.toConfigStruct();
verifyEqual(testCase, strSerializedConfig.dynamics_model, 'kinematic_constant_inertial_rate');
strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid);

% Rotate the initial attitude by the constant inertial rate from the first epoch
dExpectedDCM = zeros(3, 3, numel(dTimegrid));
for ui32TimeIdx = uint32(1):uint32(numel(dTimegrid))
    dElapsedTime = dTimegrid(ui32TimeIdx) - dTimegrid(1);
    dExpectedDCM(:, :, ui32TimeIdx) = RotationVectorToDCM(dElapsedTime * dInitialAngVel_IN) * dInitialDCM_INfromTB;
end
dExpectedAngVel_TB = dInitialDCM_INfromTB.' * dInitialAngVel_IN;

verifyEqual(testCase, strHistory.dDCM_INfromTB, dExpectedDCM, 'AbsTol', 2.0e-13);
verifyEqual(testCase, strHistory.dTargetAngVel_IN, repmat(dInitialAngVel_IN, 1, numel(dTimegrid)), 'AbsTol', 0.0);
verifyEqual(testCase, strHistory.dTargetAngVel_TB, repmat(dExpectedAngVel_TB, 1, numel(dTimegrid)), 'AbsTol', 2.0e-13);
verifyEqual(testCase, strHistory.enumTargetAngularVelocityRegime, EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL);
verifyFalse(testCase, isfield(strHistory, 'dNominalAngVel_IN'));
verifyEqual(testCase, strHistory.dTargetSpinAxis_TB, dExpectedAngVel_TB / norm(dExpectedAngVel_TB), 'AbsTol', 2.0e-13);
verifyEqual(testCase, strHistory.charTargetSpinAxisSource, "GENERATED_CONSTANT_RATE");
end

function testHistoryCarriesTypedAngularVelocityRegime(testCase)
dTimegrid = [0.0, 1.0, 2.0];
objConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'dataset', ...
    'dynamics_model', 'source_provided'));
strSourceHistory = struct('dDCM_INfromTB', repmat(eye(3), 1, 1, numel(dTimegrid)));

strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strSourceHistory);

bHasTypedRegime = isfield(strHistory, 'enumTargetAngularVelocityRegime') && ...
    isa(strHistory.enumTargetAngularVelocityRegime, 'EnumTargetAngularVelocityRegime');
verifyTrue(testCase, bHasTypedRegime);
end

function testSourceRateCapabilityRequiresConstantSequence(testCase)
dTimegrid = [0.0, 4.0, 11.0, 19.0];
dDCM_INfromTB = repmat(eye(3), 1, 1, numel(dTimegrid));
dVariableAngVel_IN = [0.01, 0.012, 0.009, 0.013; ...
                      0.02, 0.018, 0.022, 0.019; ...
                     -0.03, -0.028, -0.031, -0.027];
objConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'dataset', ...
    'dynamics_model', 'source_provided'));

% A varying rate sequence is time varying
strVariableSource = struct('dDCM_INfromTB', dDCM_INfromTB, ...
    'dTargetAngVel_IN', dVariableAngVel_IN);
strVariableHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strVariableSource);
verifyEqual(testCase, strVariableHistory.enumTargetAngularVelocityRegime, EnumTargetAngularVelocityRegime.TIME_VARYING);
verifyFalse(testCase, isfield(strVariableHistory, 'dNominalAngVel_IN'));
verifyEqual(testCase, strVariableHistory.dTargetAngVel_TB, dVariableAngVel_IN, 'AbsTol', 0.0);

% A constant sequence is constant, and attitude alone leaves the rates unavailable
dConstantAngVel_IN = repmat([0.01; -0.02; 0.03], 1, numel(dTimegrid));
strConstantSource = struct('dDCM_INfromTB', dDCM_INfromTB, ...
    'dTargetAngVel_IN', dConstantAngVel_IN);
strConstantHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strConstantSource);
verifyEqual(testCase, strConstantHistory.enumTargetAngularVelocityRegime, ...
    EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL);
verifyFalse(testCase, isfield(strConstantHistory, 'dNominalAngVel_IN'));

strAttitudeOnlySource = struct('dDCM_INfromTB', dDCM_INfromTB);
strAttitudeOnlyHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strAttitudeOnlySource);
verifyEqual(testCase, strAttitudeOnlyHistory.enumTargetAngularVelocityRegime, ...
    EnumTargetAngularVelocityRegime.UNAVAILABLE);

% A declared spin axis is copied with its provenance
strAxisSource = strConstantSource;
strAxisSource.dTargetSpinAxis_TB = [0.0; 0.0; 1.0];
strAxisSource.charTargetSpinAxisSource = "SOURCE_DECLARED";
strAxisHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strAxisSource);
verifyEqual(testCase, strAxisHistory.dTargetSpinAxis_TB, [0.0; 0.0; 1.0], 'AbsTol', 0.0);
verifyEqual(testCase, strAxisHistory.charTargetSpinAxisSource, "SOURCE_DECLARED");

% Reject the nominal-rate field and malformed spin-axis metadata
strLegacyNominalSource = strConstantSource;
strLegacyNominalSource.dNominalAngVel_IN = dConstantAngVel_IN(:, 1);
verifyError(testCase, @() BuildTargetRotationalStateHistory(objConfig, dTimegrid, strLegacyNominalSource), ...
    'BuildTargetRotationalStateHistory:UnsupportedNominalAngularVelocityField');

strIncompleteSpinAxisSource = strConstantSource;
strIncompleteSpinAxisSource.dTargetSpinAxis_TB = [0.0; 0.0; 1.0];
verifyError(testCase, @() BuildTargetRotationalStateHistory(objConfig, dTimegrid, strIncompleteSpinAxisSource), ...
    'BuildTargetRotationalStateHistory:IncompleteSpinAxisMetadata');

strComplexSpinAxisSource = strConstantSource;
strComplexSpinAxisSource.dTargetSpinAxis_TB = [1.0i; 0.0; 0.0];
strComplexSpinAxisSource.charTargetSpinAxisSource = "SOURCE_DECLARED";
verifyError(testCase, @() BuildTargetRotationalStateHistory(objConfig, dTimegrid, strComplexSpinAxisSource), ...
    'BuildTargetRotationalStateHistory:InvalidSpinAxisMetadata');

strNumericSpinAxisProvenance = strConstantSource;
strNumericSpinAxisProvenance.dTargetSpinAxis_TB = [0.0; 0.0; 1.0];
strNumericSpinAxisProvenance.charTargetSpinAxisSource = 42;
verifyError(testCase, @() BuildTargetRotationalStateHistory(objConfig, dTimegrid, strNumericSpinAxisProvenance), ...
    'BuildTargetRotationalStateHistory:InvalidSpinAxisMetadata');

strEmptySpinAxisProvenance = strConstantSource;
strEmptySpinAxisProvenance.dTargetSpinAxis_TB = [0.0; 0.0; 1.0];
strEmptySpinAxisProvenance.charTargetSpinAxisSource = "   ";
verifyError(testCase, @() BuildTargetRotationalStateHistory(objConfig, dTimegrid, strEmptySpinAxisProvenance), ...
    'BuildTargetRotationalStateHistory:InvalidSpinAxisMetadata');
end

function testTorqueFreeAcceptsTwoEpochOutputGrid(testCase)
dTimegrid = [25.0, 29.0];
strConfig = BuildTorqueFreeConfigStruct_();
strConfig.inertia_tensor_TB_kg_m2 = 2.0 * eye(3);
strConfig.max_integration_step_s = 10.0;
objConfig = STargetRotationalStateConfig.FromConfigStruct(strConfig);

strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid);

% A spherical inertia keeps the body rate constant
verifySize(testCase, strHistory.dDCM_INfromTB, [3, 3, 2]);
verifySize(testCase, strHistory.dTargetAngVel_IN, [3, 2]);
verifySize(testCase, strHistory.dTargetAngVel_TB, [3, 2]);
verifyEqual(testCase, strHistory.dTargetAngVel_TB, repmat(objConfig.dInitialAngVel_TB, 1, 2), 'AbsTol', 2.0e-13);
end

function testTorqueFreeMatlabBackendDoesNotCreateMexRoot(testCase)
%% SIGNATURE
% testTorqueFreeMatlabBackendDoesNotCreateMexRoot(testCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that explicit MATLAB execution never resolves or creates generated-artifact storage. This catches
% accidental reintroduction of path-presence dispatch or unconditional builder setup.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% testCase    Active MATLAB function-test case.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTargetRotationalStateHistory, EnumTargetRotationalStateBackend.
% -------------------------------------------------------------------------------------------------------------

charAbsentMexRoot = [tempname(), '_target_rotation_mex'];
objConfig = STargetRotationalStateConfig.FromConfigStruct(BuildTorqueFreeConfigStruct_());

strHistory = BuildTargetRotationalStateHistory(objConfig, ...
    [0.0, 0.2, 0.7], ...
    struct(), ...
    "enumBackend", EnumTargetRotationalStateBackend.MATLAB, ...
    "charMexOutputRoot", charAbsentMexRoot);

verifyFalse(testCase, isfolder(charAbsentMexRoot));
verifySize(testCase, strHistory.dDCM_INfromTB, [3, 3, 3]);
verifySize(testCase, strHistory.dTargetAngVel_TB, [3, 3]);
end

function testRequestedMexBuildFailureDoesNotFallback(testCase)
%% SIGNATURE
% testRequestedMexBuildFailureDoesNotFallback(testCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that an explicitly requested MEX backend reports a build-boundary failure instead of running the
% MATLAB reference kernel. A regular file is supplied as the artifact root so the failure occurs before code
% generation.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% testCase    Active MATLAB function-test case.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RunTorqueFreeBodyHistory, EnumTargetRotationalStateBackend.
% -------------------------------------------------------------------------------------------------------------

% Block the artifact root with a regular file
charBlockedMexRoot = tempname();
i32FileId = fopen(charBlockedMexRoot, 'w');
assert(i32FileId > 0, 'Unable to create the blocked MEX-root fixture.');
fclose(i32FileId);
objFixtureCleanup = onCleanup(@() DeleteFileIfPresent_(charBlockedMexRoot));

objConfig = STargetRotationalStateConfig.FromConfigStruct(BuildTorqueFreeConfigStruct_());
verifyError(testCase, @() RunTorqueFreeBodyHistory([0.0, 0.2], ...
    [1.0; 0.0; 0.0; 0.0], ...
    objConfig.dInitialAngVel_TB, ...
    objConfig.dInertiaTensor_TB, ...
    objConfig.dMaxIntegrationStep, ...
    "enumBackend", EnumTargetRotationalStateBackend.MEX, ...
    "charMexOutputRoot", charBlockedMexRoot), ...
    'RunTorqueFreeBodyHistory:MexBuildFailed');
end

function testMexBackendRequiresTorqueFreeModel(testCase)
%% SIGNATURE
% testMexBackendRequiresTorqueFreeModel(testCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that selecting the generated integration backend for an analytic rotational model is rejected rather
% than silently ignored.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% testCase    Active MATLAB function-test case.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTargetRotationalStateHistory, EnumTargetRotationalStateBackend.
% -------------------------------------------------------------------------------------------------------------

objConstantRateConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'generated', ...
    'dynamics_model', 'kinematic_constant_inertial_rate', ...
    'initial_quaternion_INfromTB', [1.0, 0.0, 0.0, 0.0], ...
    'initial_angular_velocity_IN_rad_s', [0.01, 0.02, 0.03]));

verifyError(testCase, @() BuildTargetRotationalStateHistory(objConstantRateConfig, ...
    [0.0, 1.0], ...
    struct(), ...
    "enumBackend", EnumTargetRotationalStateBackend.MEX), ...
    'BuildTargetRotationalStateHistory:BackendUnsupportedForModel');
end

function testStatelessTorqueFreeKernelMatchesLegacyIntegrator(testCase)
dTimegrid = [20.0, 20.17, 20.53, 21.2];
dRelativeTimegrid = dTimegrid - dTimegrid(1);
dInitialDCM_INfromTB = RotationVectorToDCM([0.29; -0.17; 0.23]);
dInitialQuat_TBfromIN = DCM2quatSeq(transpose(dInitialDCM_INfromTB), false);
dInitialAngVel_TB = [0.14; -0.09; 0.27];
dInertiaTensor_TB = [2.1, 0.08, -0.03; ...
                     0.08, 2.7, 0.05; ...
                    -0.03, 0.05, 3.4];
dMaxStep = 0.03;

% Run the MathCore class integrator and the stateless kernel on the same grid
objLegacyIntegrator = CRigidBodyDynamicsIntegrator(dInertiaTensor_TB);
[dLegacyQuatHistory, dLegacyAngVelHistory] = objLegacyIntegrator.integrate(dRelativeTimegrid, ...
    dInitialQuat_TBfromIN, ...
    dInitialAngVel_TB, ...
    zeros(3, 1), ...
    dMaxStep, ...
    'rk4_rkmk4', ...
    true, ...
    dMaxStep);
[dStatelessQuatHistory, dStatelessAngVelHistory] = PropagateTorqueFreeBodyHistory(dRelativeTimegrid, ...
    dInitialQuat_TBfromIN, ...
    dInitialAngVel_TB, ...
    dInertiaTensor_TB, ...
    dMaxStep);

verifyEqual(testCase, dStatelessAngVelHistory, dLegacyAngVelHistory, 'AbsTol', 2.0e-14);
verifyEqual(testCase, dStatelessQuatHistory, dLegacyQuatHistory, 'AbsTol', 2.0e-14);
verifyEqual(testCase, vecnorm(dStatelessQuatHistory, 2, 1), ones(1, numel(dTimegrid)), 'AbsTol', 2.0e-14);
end

function testStatelessTorqueFreeKernelRejectsInvalidInputs(testCase)
dValidTimegrid = [0.0, 1.0];
dValidQuaternion = [1.0; 0.0; 0.0; 0.0];
dValidAngVel = [0.1; -0.2; 0.3];
dValidInertia = diag([1.8, 2.3, 3.1]);

% Reject a repeated epoch, a zero quaternion, a nonphysical inertia and too many internal steps
verifyError(testCase, @() PropagateTorqueFreeBodyHistory([0.0, 1.0, 1.0], ...
    dValidQuaternion, ...
    dValidAngVel, ...
    dValidInertia, ...
    0.1), ...
    'PropagateTorqueFreeBodyHistory:InvalidTimegrid');
verifyError(testCase, @() PropagateTorqueFreeBodyHistory(dValidTimegrid, ...
    zeros(4, 1), ...
    dValidAngVel, ...
    dValidInertia, ...
    0.1), ...
    'PropagateTorqueFreeBodyHistory:InvalidQuaternion');
verifyError(testCase, @() PropagateTorqueFreeBodyHistory(dValidTimegrid, ...
    dValidQuaternion, ...
    dValidAngVel, ...
    diag([1.0, 1.0, 3.0]), ...
    0.1), ...
    'ValidatePhysicalInertia:InvalidInertiaTensor');
verifyError(testCase, @() PropagateTorqueFreeBodyHistory(dValidTimegrid, ...
    dValidQuaternion, ...
    dValidAngVel, ...
    dValidInertia, ...
    1.0e-12), ...
    'PropagateTorqueFreeBodyHistory:TooManyInternalSteps');
end

function testStatelessKernelCanonicalizesAcceptedInertia(testCase)
dTimegrid = [0.0, 0.17, 0.53];
dInitialQuaternion = CQuatKinematicsIntegrator.NormalizeSeq([0.91; -0.14; 0.27; 0.19]);
dInitialAngVel = [0.14; -0.09; 0.27];
dSymmetricInertia = [2.1, 0.08, -0.03; ...
                     0.08, 2.7, 0.05; ...
                    -0.03, 0.05, 3.4];

% Perturb the off-diagonal pair antisymmetrically within the symmetry tolerance
dNearlySymmetricInertia = dSymmetricInertia;
dNearlySymmetricInertia(1, 2) = dNearlySymmetricInertia(1, 2) + 5.0e-13;
dNearlySymmetricInertia(2, 1) = dNearlySymmetricInertia(2, 1) - 5.0e-13;

[dExpectedQuatHistory, dExpectedAngVelHistory] = PropagateTorqueFreeBodyHistory(dTimegrid, ...
    dInitialQuaternion, ...
    dInitialAngVel, ...
    dSymmetricInertia, ...
    0.03);
[dActualQuatHistory, dActualAngVelHistory] = PropagateTorqueFreeBodyHistory(dTimegrid, ...
    dInitialQuaternion, ...
    dInitialAngVel, ...
    dNearlySymmetricInertia, ...
    0.03);

verifyEqual(testCase, dActualQuatHistory, dExpectedQuatHistory, 'AbsTol', 0.0);
verifyEqual(testCase, dActualAngVelHistory, dExpectedAngVelHistory, 'AbsTol', 0.0);
end

function testTorqueFreeMatchesConstantRateForSphericalInertia(testCase)
dTimegrid = [80.0, 81.25, 83.0];
dInitialDCM_INfromTB = RotationVectorToDCM([0.37; -0.24; 0.19]);
dInitialQuat_INfromTB = DCM2quatSeq(dInitialDCM_INfromTB, false);
dInitialAngVel_TB = [0.11; -0.07; 0.24];

% Configure the same initial state for torque-free spherical and constant-rate truth
strTorqueFreeConfig = BuildTorqueFreeConfigStruct_();
strTorqueFreeConfig.initial_quaternion_INfromTB = transpose(dInitialQuat_INfromTB);
strTorqueFreeConfig.initial_angular_velocity_TB_rad_s = transpose(dInitialAngVel_TB);
strTorqueFreeConfig.inertia_tensor_TB_kg_m2 = 2.0 * eye(3);
objTorqueFreeConfig = STargetRotationalStateConfig.FromConfigStruct(strTorqueFreeConfig);

objConstantRateConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'generated', ...
    'dynamics_model', 'kinematic_constant_inertial_rate', ...
    'initial_quaternion_INfromTB', transpose(dInitialQuat_INfromTB), ...
    'initial_angular_velocity_IN_rad_s', transpose(dInitialDCM_INfromTB * dInitialAngVel_TB), ...
    'max_integration_step_s', strTorqueFreeConfig.max_integration_step_s));

strTorqueFreeHistory = BuildTargetRotationalStateHistory(objTorqueFreeConfig, dTimegrid);
strConstantRateHistory = BuildTargetRotationalStateHistory(objConstantRateConfig, dTimegrid);

verifyEqual(testCase, strTorqueFreeHistory.dDCM_INfromTB, strConstantRateHistory.dDCM_INfromTB, 'AbsTol', 2.0e-10);
verifyEqual(testCase, strTorqueFreeHistory.dTargetAngVel_IN, strConstantRateHistory.dTargetAngVel_IN, 'AbsTol', 2.0e-10);
verifyEqual(testCase, strTorqueFreeHistory.dTargetAngVel_TB, strConstantRateHistory.dTargetAngVel_TB, 'AbsTol', 2.0e-12);
end

function testTorqueFreeHistoryPreservesRigidBodyInvariants(testCase)
dTimegrid = 500.0:0.5:520.0;
objConfig = STargetRotationalStateConfig.FromConfigStruct(BuildTorqueFreeConfigStruct_());
strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid);

% Collect energy and angular momentum, and check each attitude sample and inertial rate
dKineticEnergy = zeros(1, numel(dTimegrid));
dAngularMomentumNorm = zeros(1, numel(dTimegrid));
dAngularMomentum_IN = zeros(3, numel(dTimegrid));
for ui32TimeIdx = uint32(1):uint32(numel(dTimegrid))
    dTargetAngVel_TB = strHistory.dTargetAngVel_TB(:, ui32TimeIdx);
    dAngularMomentum_TB = objConfig.dInertiaTensor_TB * dTargetAngVel_TB;
    dKineticEnergy(ui32TimeIdx) = 0.5 * transpose(dTargetAngVel_TB) * dAngularMomentum_TB;
    dAngularMomentumNorm(ui32TimeIdx) = norm(dAngularMomentum_TB);

    dDCM = strHistory.dDCM_INfromTB(:, :, ui32TimeIdx);
    dAngularMomentum_IN(:, ui32TimeIdx) = dDCM * dAngularMomentum_TB;
    verifyEqual(testCase, transpose(dDCM) * dDCM, eye(3), 'AbsTol', 2.0e-12);
    verifyEqual(testCase, det(dDCM), 1.0, 'AbsTol', 2.0e-12);
    verifyEqual(testCase, strHistory.dTargetAngVel_IN(:, ui32TimeIdx), dDCM * dTargetAngVel_TB, 'AbsTol', 2.0e-12);
end

% Energy, momentum norm and inertial momentum are conserved
verifyLessThan(testCase, max(abs(dKineticEnergy - dKineticEnergy(1))), 2.0e-10);
verifyLessThan(testCase, max(abs(dAngularMomentumNorm - dAngularMomentumNorm(1))), 2.0e-10);
verifyLessThan(testCase, max(vecnorm(dAngularMomentum_IN - dAngularMomentum_IN(:, 1), 2, 1)), 2.0e-10);
verifyEqual(testCase, strHistory.enumTargetAngularVelocityRegime, EnumTargetAngularVelocityRegime.TIME_VARYING);
verifyFalse(testCase, isfield(strHistory, 'dNominalAngVel_IN'));
verifyFalse(testCase, isfield(strHistory, 'dTargetSpinAxis_TB'));
verifyFalse(testCase, isfield(strHistory, 'charTargetSpinAxisSource'));
end

function testTorqueFreeAxisymmetricMatchesClosedForm(testCase)
%% SIGNATURE
% testTorqueFreeAxisymmetricMatchesClosedForm(testCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compare the torque-free history of an axisymmetric body, J = diag(I1, I1, I3), with the closed-form solution
% over about three nutation periods. The body rate turns about the symmetry axis e3 at the nutation rate
% lambda = (I3 - I1) * omega3 / I1, so omega_TB(t) = Exp(skew(e3) * lambda * t) * omega_TB(0). The attitude is
% R_INfromTB(t) = Exp(skew(h_IN) * |H| / I1 * t) * R_INfromTB(0) * Exp(-skew(e3) * lambda * t), with h_IN the
% unit inertial angular momentum. The matrix exponentials use expm, independent of the MathCore rotation code.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% testCase    Active MATLAB function-test case.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-11-2026  Pietro Califano, Claude Opus 5.5   First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% STargetRotationalStateConfig, BuildTargetRotationalStateHistory, DCM2quatSeq.
% -------------------------------------------------------------------------------------------------------------

% Integrate an axisymmetric body from a non-identity attitude over 200 s
dTimegrid = 0.0:2.0:200.0;
dTransverseInertia = 2.0;
dAxialInertia = 3.0;
dInertiaTensor_TB = diag([dTransverseInertia, dTransverseInertia, dAxialInertia]);
dInitialDCM_INfromTB = expm(Skew_([0.37; -0.24; 0.19]));
dInitialAngVel_TB = [0.05; -0.03; 0.2];

objConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'generated', ...
    'dynamics_model', 'rigid_body_torque_free', ...
    'initial_quaternion_INfromTB', transpose(DCM2quatSeq(dInitialDCM_INfromTB, false)), ...
    'initial_angular_velocity_TB_rad_s', transpose(dInitialAngVel_TB), ...
    'inertia_tensor_TB_kg_m2', dInertiaTensor_TB, ...
    'max_integration_step_s', 0.01));
strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid);

% Evaluate the closed-form body rate and attitude at each epoch
dNutationRate = (dAxialInertia - dTransverseInertia) * dInitialAngVel_TB(3) / dTransverseInertia;
dAngularMomentum_IN = dInitialDCM_INfromTB * dInertiaTensor_TB * dInitialAngVel_TB;
dPrecessionRate = norm(dAngularMomentum_IN) / dTransverseInertia;
dMomentumAxis_IN = dAngularMomentum_IN / norm(dAngularMomentum_IN);

dExpectedAngVel_TB = zeros(3, numel(dTimegrid));
dExpectedDCM_INfromTB = zeros(3, 3, numel(dTimegrid));
for ui32TimeIdx = uint32(1):uint32(numel(dTimegrid))
    dElapsedTime = dTimegrid(ui32TimeIdx) - dTimegrid(1);
    dBodyRotation = expm(Skew_([0.0; 0.0; dNutationRate * dElapsedTime]));
    dExpectedAngVel_TB(:, ui32TimeIdx) = dBodyRotation * dInitialAngVel_TB;
    dExpectedDCM_INfromTB(:, :, ui32TimeIdx) = expm(Skew_(dPrecessionRate * dElapsedTime * dMomentumAxis_IN)) * ...
        dInitialDCM_INfromTB * transpose(dBodyRotation);
end

% The grid spans about three nutation periods, and the history follows the closed form
verifyGreaterThan(testCase, dNutationRate * (dTimegrid(end) - dTimegrid(1)), 6.0 * pi);
verifyEqual(testCase, strHistory.dTargetAngVel_TB, dExpectedAngVel_TB, 'AbsTol', 1.0e-9);
verifyEqual(testCase, strHistory.dDCM_INfromTB, dExpectedDCM_INfromTB, 'AbsTol', 1.0e-9);
end

function testTorqueFreeTriaxialRateMatchesJacobiEllipticSolution(testCase)
%% SIGNATURE
% testTorqueFreeTriaxialRateMatchesJacobiEllipticSolution(testCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compare the body rate of a triaxial body, I1 < I2 < I3, with the Jacobi elliptic solution of the Euler
% equations over about four nutation periods (Landau and Lifshitz, Mechanics, section 37). The initial rate
% has omega2 = 0, omega1 > 0 and omega3 > 0, with |H|^2 > 2 E I2 (rotation about the major axis). Then
% omega1 = a1 cn(tau, m), omega2 = a2 sn(tau, m), omega3 = a3 dn(tau, m), with
% tau = t sqrt((I3 - I2) (|H|^2 - 2 E I1) / (I1 I2 I3)), m = (I2 - I1) (2 E I3 - |H|^2) / ((I3 - I2)
% (|H|^2 - 2 E I1)), a1 = sqrt((2 E I3 - |H|^2) / (I1 (I3 - I1))), a2 = sqrt((2 E I3 - |H|^2) / (I2 (I3 - I2)))
% and a3 = sqrt((|H|^2 - 2 E I1) / (I3 (I3 - I1))).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% testCase    Active MATLAB function-test case.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-11-2026  Pietro Califano, Claude Opus 5.5   First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% STargetRotationalStateConfig, BuildTargetRotationalStateHistory, ellipj.
% -------------------------------------------------------------------------------------------------------------

% Integrate a triaxial body spinning near its major axis over 200 s
dTimegrid = 0.0:2.0:200.0;
dPrincipalMoments = [1.8; 2.3; 3.1];
dInitialAngVel_TB = [0.1; 0.0; 0.25];

objConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'generated', ...
    'dynamics_model', 'rigid_body_torque_free', ...
    'initial_quaternion_INfromTB', [1.0, 0.0, 0.0, 0.0], ...
    'initial_angular_velocity_TB_rad_s', transpose(dInitialAngVel_TB), ...
    'inertia_tensor_TB_kg_m2', diag(dPrincipalMoments), ...
    'max_integration_step_s', 0.01));
strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid);

% Evaluate the Jacobi elliptic solution from twice the kinetic energy and the squared angular momentum
dI1 = dPrincipalMoments(1);
dI2 = dPrincipalMoments(2);
dI3 = dPrincipalMoments(3);
dTwiceEnergy = sum(dPrincipalMoments .* dInitialAngVel_TB .^ 2);
dMomentumSquared = sum((dPrincipalMoments .* dInitialAngVel_TB) .^ 2);

dTimeScale = sqrt((dI3 - dI2) * (dMomentumSquared - dTwiceEnergy * dI1) / (dI1 * dI2 * dI3));
dEllipticParameter = (dI2 - dI1) * (dTwiceEnergy * dI3 - dMomentumSquared) / ...
    ((dI3 - dI2) * (dMomentumSquared - dTwiceEnergy * dI1));
dAmplitudes = [sqrt((dTwiceEnergy * dI3 - dMomentumSquared) / (dI1 * (dI3 - dI1))); ...
    sqrt((dTwiceEnergy * dI3 - dMomentumSquared) / (dI2 * (dI3 - dI2))); ...
    sqrt((dMomentumSquared - dTwiceEnergy * dI1) / (dI3 * (dI3 - dI1)))];
[dSn, dCn, dDn] = ellipj(dTimeScale * (dTimegrid - dTimegrid(1)), dEllipticParameter);
dExpectedAngVel_TB = dAmplitudes .* [dCn; dSn; dDn];

% The case is rotation about the major axis, the solution starts at the initial rate, the grid spans about
% four nutation periods, and the history follows the solution
verifyGreaterThan(testCase, dMomentumSquared, dTwiceEnergy * dI2);
verifyEqual(testCase, dExpectedAngVel_TB(:, 1), dInitialAngVel_TB, 'AbsTol', 1.0e-15);
verifyGreaterThan(testCase, dTimeScale * (dTimegrid(end) - dTimegrid(1)), 12.0 * ellipke(dEllipticParameter));
verifyEqual(testCase, strHistory.dTargetAngVel_TB, dExpectedAngVel_TB, 'AbsTol', 1.0e-9);
end

function testAppliesDeterministicPhysicalPerturbations(testCase)
objNominalConfig = STargetRotationalStateConfig.FromConfigStruct(BuildTorqueFreeConfigStruct_());
strRealization = BuildPerturbationRealization_();

% Apply the same realization twice
objPerturbedA = ApplyTargetRotationalStatePerturbation(objNominalConfig, strRealization);
objPerturbedB = ApplyTargetRotationalStatePerturbation(objNominalConfig, strRealization);
dPerturbedMoments = eig(objPerturbedA.dInertiaTensor_TB);

verifyEqual(testCase, objPerturbedA.toConfigStruct(), objPerturbedB.toConfigStruct());
verifyEqual(testCase, objPerturbedA.dInitialAngVel_TB, ...
    objNominalConfig.dInitialAngVel_TB + strRealization.dAngularVelocityError, ...
    'AbsTol', 0.0);
verifyGreaterThan(testCase, norm(objPerturbedA.dInitialQuat_INfromTB - objNominalConfig.dInitialQuat_INfromTB), ...
    1.0e-15);
verifyGreaterThan(testCase, min(dPerturbedMoments), 0.0);
verifyTrue(testCase, all(dPerturbedMoments < sum(dPerturbedMoments) - dPerturbedMoments));
verifyFalse(testCase, isfield(objPerturbedA.strPerturbationProvenance, 'ui32AttemptId'));
verifySize(testCase, objPerturbedA.strPerturbationProvenance.dInertiaChartError, [6, 1]);
verifyEqual(testCase, objPerturbedA.strPerturbationProvenance.strSamplingProvenance, struct());

% Reject an unapplied realization with errors, a zero seed and an out-of-range inertia chart
strInconsistentInactiveRealization = strRealization;
strInconsistentInactiveRealization.bApplied = false;
verifyError(testCase, @() ApplyTargetRotationalStatePerturbation(objNominalConfig, ...
    strInconsistentInactiveRealization), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strZeroSeedRealization = strRealization;
strZeroSeedRealization.ui32Seed = uint32(0);
verifyError(testCase, @() ApplyTargetRotationalStatePerturbation(objNominalConfig, strZeroSeedRealization), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

strInvalidInertiaRealization = strRealization;
strInvalidInertiaRealization.dInertiaChartError(1) = 1000.0;
verifyError(testCase, @() ApplyTargetRotationalStatePerturbation(objNominalConfig, ...
    strInvalidInertiaRealization), ...
    'PerturbInertiaTensor:ChartOutOfRange');

% Reject inertia errors on constant-rate truth and any perturbation of source-provided truth
objConstantConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'generated', ...
    'dynamics_model', 'kinematic_constant_inertial_rate', ...
    'initial_quaternion_INfromTB', [1.0, 0.0, 0.0, 0.0], ...
    'initial_angular_velocity_IN_rad_s', [0.01, 0.02, 0.03]));
verifyError(testCase, @() ApplyTargetRotationalStatePerturbation(objConstantConfig, strRealization), ...
    'STargetRotationalStateConfig:InvalidPerturbationProvenance');

objSourceConfig = STargetRotationalStateConfig.FromConfigStruct(struct('source', 'spice', ...
    'dynamics_model', 'source_provided'));
verifyError(testCase, @() ApplyTargetRotationalStatePerturbation(objSourceConfig, ...
    BuildPerturbationRealization_()), ...
    'ApplyTargetRotationalStatePerturbation:UnsupportedSource');
end

function strConfig = BuildTorqueFreeConfigStruct_()
%% SIGNATURE
% strConfig = BuildTorqueFreeConfigStruct_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build one valid generated torque-free target rotational-state configuration.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strConfig    (1,1) struct public configuration payload
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    strConfig (1,1) struct
end

strConfig = struct('source', 'generated', ...
    'dynamics_model', 'rigid_body_torque_free', ...
    'initial_quaternion_INfromTB', [1.0, 0.0, 0.0, 0.0], ...
    'initial_angular_velocity_TB_rad_s', [0.11, -0.07, 0.24], ...
    'inertia_tensor_TB_kg_m2', diag([1.8, 2.3, 3.1]), ...
    'max_integration_step_s', 0.01);
end

function strRealization = BuildPerturbationRealization_()
%% SIGNATURE
% strRealization = BuildPerturbationRealization_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build one deterministic complete perturbation realization.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strRealization    (1,1) struct perturbation payload
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    strRealization (1,1) struct
end

strRealization = struct('bApplied', true, ...
    'ui32Seed', uint32(1234), ...
    'dInitialAttitudeTangent_IN', [0.01; -0.02; 0.03], ...
    'dAngularVelocityError', [1.0e-3; -2.0e-3; 3.0e-3], ...
    'dInertiaChartError', [0.01; -0.015; 0.02; ...
                           -0.02; 0.01; 0.015], ...
    'strSamplingProvenance', struct());
end

function DeleteFileIfPresent_(charFilePath)
%% SIGNATURE
% DeleteFileIfPresent_(charFilePath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Delete one temporary test fixture when it exists.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charFilePath    (1,:) char temporary file path
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

if isfile(charFilePath)
    delete(charFilePath);
end
end

function dSkew = Skew_(dVector)
%% SIGNATURE
% dSkew = Skew_(dVector)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build the cross-product matrix of a 3-vector, so that Skew_(a) * b = cross(a, b).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dVector    (3,1) double vector
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSkew      (3,3) double cross-product matrix
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-11-2026  Pietro Califano, Claude Opus 5.5   First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dVector (3,1) double
end
arguments (Output)
    dSkew (3,3) double
end

dSkew = [0.0, -dVector(3), dVector(2); ...
         dVector(3), 0.0, -dVector(1); ...
        -dVector(2), dVector(1), 0.0];
end
