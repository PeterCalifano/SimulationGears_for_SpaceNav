function objPerturbedConfig = ApplyTargetRotationalStatePerturbation(objNominalConfig, strRealization)
%% SIGNATURE
% objPerturbedConfig = ApplyTargetRotationalStatePerturbation(objNominalConfig, strRealization)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply one externally sampled target rotational-state perturbation without using random-number state.
% Attitude error is a left tangent in the inertial frame. Angular-velocity error uses the model's defining
% frame. Torque-free inertia errors use the deterministic six-coordinate physical-inertia chart owned by
% PerturbInertiaTensor. The realization schema and the quaternion sign rule belong to
% STargetRotationalStateConfig.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objNominalConfig    (1,1) STargetRotationalStateConfig nominal generated target truth
% strRealization      (1,1) struct complete deterministic perturbation realization
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objPerturbedConfig  (1,1) STargetRotationalStateConfig perturbed physical configuration
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% 23-08-2026  Pietro Califano, Codex     Use direct physical-inertia chart with no sampling attempts.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% 12-11-2026  Pietro Califano, Claude Opus 5.5   Validate the realization and canonicalize the quaternion
%                                                through STargetRotationalStateConfig.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% DCM2quatSeq, PerturbInertiaTensor, QuatSeq2DCM, RotationVectorToDCM, STargetRotationalStateConfig.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    objNominalConfig (1,1) STargetRotationalStateConfig
    strRealization (1,1) struct
end
arguments (Output)
    objPerturbedConfig (1,1) STargetRotationalStateConfig
end

% Return the nominal configuration unchanged when the realization is not applied
strRealization = STargetRotationalStateConfig.ValidateRealization(strRealization, ...
    objNominalConfig.enumDynamicsModel);
objPerturbedConfig = objNominalConfig;
if ~strRealization.bApplied
    return
end

% Accept generated truth only
if objNominalConfig.enumSource ~= EnumTargetRotationalStateSource.GENERATED
    error('ApplyTargetRotationalStatePerturbation:UnsupportedSource', ...
        'Target rotational-state perturbations require source GENERATED.');
end

% Apply the attitude error on the inertial side of R_INfromTB, then store the canonical quaternion
dInitialDCM_INfromTB = QuatSeq2DCM(objNominalConfig.dInitialQuat_INfromTB, false);
dPerturbedDCM_INfromTB = RotationVectorToDCM(strRealization.dInitialAttitudeTangent_IN) * dInitialDCM_INfromTB;
objPerturbedConfig.dInitialQuat_INfromTB = STargetRotationalStateConfig.CanonicalizeQuaternion( ...
    DCM2quatSeq(dPerturbedDCM_INfromTB, false));

% Apply the rate error in the model's defining frame, and the inertia error for torque-free truth
switch objNominalConfig.enumDynamicsModel
    case EnumTargetRotationalDynamicsModel.KINEMATIC_CONST_INERTIAL_RATE
        objPerturbedConfig.dInitialAngVel_IN = objNominalConfig.dInitialAngVel_IN + ...
            strRealization.dAngularVelocityError;

    case EnumTargetRotationalDynamicsModel.RIGID_BODY_TORQUE_FREE
        objPerturbedConfig.dInitialAngVel_TB = objNominalConfig.dInitialAngVel_TB + ...
            strRealization.dAngularVelocityError;
        objPerturbedConfig.dInertiaTensor_TB = PerturbInertiaTensor(objNominalConfig.dInertiaTensor_TB, ...
            strRealization.dInertiaChartError);

    otherwise
        error('ApplyTargetRotationalStatePerturbation:UnsupportedDynamicsModel', ...
            'Source-provided target rotation cannot be perturbed through generated-state parameters.');
end

% Record the realization seed and the realization itself as provenance, and check the complete configuration
objPerturbedConfig.ui32RealizationSeed = strRealization.ui32Seed;
objPerturbedConfig.strPerturbationProvenance = strRealization;
objPerturbedConfig.validate();

end
