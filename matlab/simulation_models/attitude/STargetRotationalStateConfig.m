classdef STargetRotationalStateConfig < CBaseDatastruct
    %% SIGNATURE
    % objConfig = STargetRotationalStateConfig()
    % objConfig = STargetRotationalStateConfig.FromConfigStruct(strConfig)
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Typed configuration for source-provided or generated target rotational truth. The class owns strict
    % public-key parsing, the source and model pairing rules, physical initial-state validation, the perturbation
    % realization schema and the canonical quaternion sign, while leaving history generation and perturbation
    % application to dedicated stateless routines.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None for default construction; FromConfigStruct accepts one scalar public configuration struct.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % objConfig    (1,1) STargetRotationalStateConfig
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 22-08-2026  Pietro Califano, Codex     First implementation.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
    % 12-11-2026  Pietro Califano, Claude Opus 5.5   Make validate, ValidateRealization and
    %                                                CanonicalizeQuaternion public for the perturbation and
    %                                                history functions; check the inertia with
    %                                                ValidatePhysicalInertia.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CBaseDatastruct, EnumTargetRotationalStateSource,
    % EnumTargetRotationalDynamicsModel, ValidatePhysicalInertia.
    % -------------------------------------------------------------------------------------------------------------

    properties (Access = public)
        enumSource (1,1) EnumTargetRotationalStateSource = EnumTargetRotationalStateSource.SPICE
        enumDynamicsModel (1,1) EnumTargetRotationalDynamicsModel = ...
            EnumTargetRotationalDynamicsModel.SOURCE_PROVIDED
        dInitialQuat_INfromTB (4,1) double {mustBeReal} = nan(4, 1)
        dInitialAngVel_IN (3,1) double {mustBeReal} = nan(3, 1)
        dInitialAngVel_TB (3,1) double {mustBeReal} = nan(3, 1)
        dInertiaTensor_TB (3,3) double {mustBeReal} = nan(3, 3)
        dMaxIntegrationStep (1,1) double {mustBeReal} = 1.0
        ui32RealizationSeed (1,1) uint32 = uint32(0)
        strPerturbationProvenance (1,1) struct = struct()
    end

    methods
        function self = STargetRotationalStateConfig()
            %% SIGNATURE
            % self = STargetRotationalStateConfig()
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Construct the backward-compatible SPICE/source-provided default.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % None.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self    (1,1) STargetRotationalStateConfig
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CBaseDatastruct.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
            end

            self = self@CBaseDatastruct();
        end

        function strConfig = toConfigStruct(self)
            %% SIGNATURE
            % strConfig = self.toConfigStruct()
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Serialize the typed configuration to canonical public YAML-compatible fields.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    (1,1) STargetRotationalStateConfig
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strConfig    (1,1) struct canonical public configuration payload
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EnumTargetRotationalStateSource.ToConfigValue,
            % EnumTargetRotationalDynamicsModel.ToConfigValue.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                self (1,1) STargetRotationalStateConfig
            end
            arguments (Output)
                strConfig (1,1) struct
            end

            self.validate();

            % Write the source and the dynamics model
            strConfig = struct();
            strConfig.source = EnumTargetRotationalStateSource.ToConfigValue(self.enumSource);
            strConfig.dynamics_model = EnumTargetRotationalDynamicsModel.ToConfigValue(self.enumDynamicsModel);

            % Write the initial state and step of generated truth, with the fields of its dynamics model only
            if self.enumSource == EnumTargetRotationalStateSource.GENERATED
                strConfig.initial_quaternion_INfromTB = transpose(self.dInitialQuat_INfromTB);
                if self.enumDynamicsModel == EnumTargetRotationalDynamicsModel.KINEMATIC_CONST_INERTIAL_RATE
                    strConfig.initial_angular_velocity_IN_rad_s = transpose(self.dInitialAngVel_IN);
                else
                    strConfig.initial_angular_velocity_TB_rad_s = transpose(self.dInitialAngVel_TB);
                    strConfig.inertia_tensor_TB_kg_m2 = self.dInertiaTensor_TB;
                end
                strConfig.max_integration_step_s = self.dMaxIntegrationStep;
            end

            % Write the realization seed and provenance when either is set
            if self.ui32RealizationSeed > uint32(0) || ~isempty(fieldnames(self.strPerturbationProvenance))
                strConfig.realization_seed = self.ui32RealizationSeed;
                strConfig.perturbation_provenance = self.strPerturbationProvenance;
            end
        end

        function validate(self)
            %% SIGNATURE
            % self.validate()
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check the perturbation provenance, the source and model pairing, and the initial-state fields of
            % the selected model. Raise an error on the first violation.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    (1,1) STargetRotationalStateConfig
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % 12-11-2026  Pietro Califano, Claude Opus 5.5   Make public for the perturbation and history
            %                                                functions.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % STargetRotationalStateConfig.ValidateRealization, ValidatePhysicalInertia.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                self (1,1) STargetRotationalStateConfig
            end

            % Treat perturbation provenance as one atomic public contract so user-authored YAML cannot retain a
            % seed without its realization, or carry a realization inconsistent with the serialized truth.
            bHasPerturbationProvenance = ~isempty(fieldnames(self.strPerturbationProvenance));
            if xor(self.ui32RealizationSeed > uint32(0), bHasPerturbationProvenance)
                error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                    ['realization_seed and perturbation_provenance must either ', ...
                    'both be present or both be absent.']);
            end
            if bHasPerturbationProvenance
                if self.enumSource ~= EnumTargetRotationalStateSource.GENERATED
                    error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                        'Perturbation provenance is supported only for GENERATED truth.');
                end
                strProvenance = STargetRotationalStateConfig.ValidateRealization(self.strPerturbationProvenance, ...
                    self.enumDynamicsModel);
                if strProvenance.ui32Seed ~= self.ui32RealizationSeed
                    error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                        'perturbation_provenance.ui32Seed must match realization_seed.');
                end
            end

            % Enforce source/model ownership before validating model-specific physical inputs.
            if self.enumSource ~= EnumTargetRotationalStateSource.GENERATED
                if self.enumDynamicsModel ~= EnumTargetRotationalDynamicsModel.SOURCE_PROVIDED
                    error('STargetRotationalStateConfig:InvalidSourceModel', ...
                        'SPICE and DATASET sources require dynamics_model source_provided.');
                end
                if any(isfinite(self.dInitialQuat_INfromTB)) || ...
                        any(isfinite(self.dInitialAngVel_IN)) || ...
                        any(isfinite(self.dInitialAngVel_TB)) || ...
                        any(isfinite(self.dInertiaTensor_TB), 'all') || ...
                        self.dMaxIntegrationStep ~= 1.0
                    error('STargetRotationalStateConfig:UnexpectedGeneratedField', ...
                        ['Source-provided rotation cannot declare generated ', ...
                        'initial-state or integration fields.']);
                end
                return
            end

            % Generated truth needs a generated model, a unit initial quaternion and a positive step
            if self.enumDynamicsModel == EnumTargetRotationalDynamicsModel.SOURCE_PROVIDED
                error('STargetRotationalStateConfig:InvalidSourceModel', ...
                    'GENERATED source requires a generated rotational dynamics model.');
            end
            if any(~isfinite(self.dInitialQuat_INfromTB))
                error('STargetRotationalStateConfig:MissingGeneratedField', ...
                    'Generated rotation requires initial_quaternion_INfromTB.');
            end
            if abs(norm(self.dInitialQuat_INfromTB) - 1.0) > 1.0e-12
                error('STargetRotationalStateConfig:InvalidQuaternion', ...
                    'Generated initial quaternion must have unit norm.');
            end
            if ~isfinite(self.dMaxIntegrationStep) || self.dMaxIntegrationStep <= 0.0
                error('STargetRotationalStateConfig:InvalidMaxIntegrationStep', ...
                    'max_integration_step_s must be one positive finite scalar.');
            end

            % Constant-rate truth needs the inertial rate and no torque-free fields
            if self.enumDynamicsModel == EnumTargetRotationalDynamicsModel.KINEMATIC_CONST_INERTIAL_RATE
                if any(~isfinite(self.dInitialAngVel_IN))
                    error('STargetRotationalStateConfig:MissingGeneratedField', ...
                        ['Constant-inertial-rate rotation requires ', ...
                        'initial_angular_velocity_IN_rad_s.']);
                end
                if any(isfinite(self.dInitialAngVel_TB)) || any(isfinite(self.dInertiaTensor_TB), 'all')
                    error('STargetRotationalStateConfig:UnexpectedGeneratedField', ...
                        ['Constant-inertial-rate rotation cannot declare torque-free ', ...
                        'body-rate or inertia fields.']);
                end
                return
            end

            % Torque-free truth needs the body rate and a physical inertia, and no inertial rate
            if any(~isfinite(self.dInitialAngVel_TB))
                error('STargetRotationalStateConfig:MissingGeneratedField', ...
                    'Torque-free rotation requires initial_angular_velocity_TB_rad_s.');
            end
            if any(isfinite(self.dInitialAngVel_IN))
                error('STargetRotationalStateConfig:UnexpectedGeneratedField', ...
                    'Torque-free rotation cannot declare a constant inertial angular velocity.');
            end
            ValidatePhysicalInertia(self.dInertiaTensor_TB);
        end
    end

    methods (Static)
        function objConfig = FromConfigStruct(strConfig)
            %% SIGNATURE
            % objConfig = STargetRotationalStateConfig.FromConfigStruct(strConfig)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Parse a strict public configuration struct into a validated typed configuration.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % strConfig    (1,1) struct public target rotational-state configuration
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objConfig    (1,1) STargetRotationalStateConfig
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EnumTargetRotationalStateSource.FromConfigValue,
            % EnumTargetRotationalDynamicsModel.FromConfigValue.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                strConfig (1,1) struct
            end
            arguments (Output)
                objConfig (1,1) STargetRotationalStateConfig
            end

            % Reject unknown fields
            cellAllowedFields = {'source', ...
                'dynamics_model', ...
                'initial_quaternion_INfromTB', ...
                'initial_angular_velocity_IN_rad_s', ...
                'initial_angular_velocity_TB_rad_s', ...
                'inertia_tensor_TB_kg_m2', ...
                'max_integration_step_s', ...
                'realization_seed', ...
                'perturbation_provenance'};
            cellUnknownFields = setdiff(fieldnames(strConfig), cellAllowedFields, 'stable');
            if ~isempty(cellUnknownFields)
                error('STargetRotationalStateConfig:UnknownField', ...
                    'Unknown target rotational-state field "%s".', cellUnknownFields{1});
            end

            % Preserve explicit field presence at the serialization boundary. A zero seed is a valid numeric
            % value but is not a materialized realization identifier, so it must not be silently discarded.
            bHasRealizationSeed = isfield(strConfig, 'realization_seed');
            bHasPerturbationProvenance = isfield(strConfig, 'perturbation_provenance');
            ui32RealizationSeed = uint32(0);
            if xor(bHasRealizationSeed, bHasPerturbationProvenance)
                error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                    ['realization_seed and perturbation_provenance must either ', ...
                    'both be present or both be absent.']);
            end
            if bHasRealizationSeed
                ui32RealizationSeed = STargetRotationalStateConfig.ReadSeed_(strConfig.realization_seed, ...
                    'realization_seed');
                if ui32RealizationSeed == uint32(0) || ...
                        ~isstruct(strConfig.perturbation_provenance) || ...
                        ~isscalar(strConfig.perturbation_provenance) || ...
                        isempty(fieldnames(strConfig.perturbation_provenance))
                    error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                        ['A materialized realization requires one positive seed ', ...
                        'and one nonempty scalar provenance struct.']);
                end
            end

            % Read the source and the dynamics model, with SPICE and source_provided as defaults
            objConfig = STargetRotationalStateConfig();
            objConfig.enumSource = EnumTargetRotationalStateSource.FromConfigValue( ...
                STargetRotationalStateConfig.GetField_(strConfig, 'source', 'spice'));
            objConfig.enumDynamicsModel = EnumTargetRotationalDynamicsModel.FromConfigValue( ...
                STargetRotationalStateConfig.GetField_(strConfig, 'dynamics_model', 'source_provided'));

            % Reject generated-model controls at the source boundary so a parse/serialize cycle cannot silently
            % discard accepted input.
            cellGeneratedOnlyFields = {'initial_quaternion_INfromTB', ...
                'initial_angular_velocity_IN_rad_s', ...
                'initial_angular_velocity_TB_rad_s', ...
                'inertia_tensor_TB_kg_m2', ...
                'max_integration_step_s'};
            if objConfig.enumSource ~= EnumTargetRotationalStateSource.GENERATED && ...
                    any(isfield(strConfig, cellGeneratedOnlyFields))
                error('STargetRotationalStateConfig:UnexpectedGeneratedField', ...
                    ['SPICE and DATASET target rotation cannot declare ', ...
                    'generated-model initial-state or integration fields.']);
            end

            % Read the generated initial state; the quaternion is normalized to one sign representative
            if isfield(strConfig, 'initial_quaternion_INfromTB')
                objConfig.dInitialQuat_INfromTB = STargetRotationalStateConfig.ReadVector_( ...
                    strConfig.initial_quaternion_INfromTB, ...
                    uint32(4), ...
                    'initial_quaternion_INfromTB', ...
                    'STargetRotationalStateConfig:InvalidVector');
                objConfig.dInitialQuat_INfromTB = STargetRotationalStateConfig.CanonicalizeQuaternion( ...
                    objConfig.dInitialQuat_INfromTB);
            end
            if isfield(strConfig, 'initial_angular_velocity_IN_rad_s')
                objConfig.dInitialAngVel_IN = STargetRotationalStateConfig.ReadVector_( ...
                    strConfig.initial_angular_velocity_IN_rad_s, ...
                    uint32(3), ...
                    'initial_angular_velocity_IN_rad_s', ...
                    'STargetRotationalStateConfig:InvalidVector');
            end
            if isfield(strConfig, 'initial_angular_velocity_TB_rad_s')
                objConfig.dInitialAngVel_TB = STargetRotationalStateConfig.ReadVector_( ...
                    strConfig.initial_angular_velocity_TB_rad_s, ...
                    uint32(3), ...
                    'initial_angular_velocity_TB_rad_s', ...
                    'STargetRotationalStateConfig:InvalidVector');
            end
            if isfield(strConfig, 'inertia_tensor_TB_kg_m2')
                objConfig.dInertiaTensor_TB = STargetRotationalStateConfig.ReadMatrix3_( ...
                    strConfig.inertia_tensor_TB_kg_m2, ...
                    'inertia_tensor_TB_kg_m2');
            end

            % Read the maximum integration step as one positive finite real scalar
            if isfield(strConfig, 'max_integration_step_s')
                varMaxIntegrationStep = strConfig.max_integration_step_s;
                if ~isnumeric(varMaxIntegrationStep) || ...
                        ~isreal(varMaxIntegrationStep) || ...
                        ~isscalar(varMaxIntegrationStep) || ...
                        ~isfinite(varMaxIntegrationStep) || ...
                        varMaxIntegrationStep <= 0.0
                    error('STargetRotationalStateConfig:InvalidMaxIntegrationStep', ...
                        'max_integration_step_s must be one positive finite real numeric scalar.');
                end
                objConfig.dMaxIntegrationStep = double(varMaxIntegrationStep);
            end

            % Copy the realization seed and provenance checked above
            if isfield(strConfig, 'realization_seed')
                objConfig.ui32RealizationSeed = ui32RealizationSeed;
            end
            if isfield(strConfig, 'perturbation_provenance')
                objConfig.strPerturbationProvenance = strConfig.perturbation_provenance;
            end

            objConfig.validate();
            objConfig.bDefaultConstructed = false;
        end

        function strRealization = ValidateRealization(strRealization, enumDynamicsModel)
            %% SIGNATURE
            % strRealization = STargetRotationalStateConfig.ValidateRealization(strRealization, ...
            %     enumDynamicsModel)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Validate one perturbation realization of the fixed-field schema shared by COSMICA
            % materialization, ApplyTargetRotationalStatePerturbation and perturbation_provenance, and return
            % it with a uint32 seed and double column error vectors. An unapplied realization must carry zero
            % errors, an applied one a positive seed, and constant-rate truth a zero inertia chart error.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % strRealization      (1,1) struct candidate perturbation realization
            % enumDynamicsModel   (1,1) EnumTargetRotationalDynamicsModel model the realization applies to
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strRealization      (1,1) struct realization with uint32 seed and double column vectors
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % 12-11-2026  Pietro Califano, Claude Opus 5.5   Merge with the realization check of
            %                                                ApplyTargetRotationalStatePerturbation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                strRealization (1,1) struct
                enumDynamicsModel (1,1) EnumTargetRotationalDynamicsModel
            end
            arguments (Output)
                strRealization (1,1) struct
            end

            charErrorId = 'STargetRotationalStateConfig:InvalidPerturbationProvenance';

            % Require exactly the documented fields and a logical scalar flag
            cellRequiredFields = {'bApplied', ...
                'ui32Seed', ...
                'dInitialAttitudeTangent_IN', ...
                'dAngularVelocityError', ...
                'dInertiaChartError', ...
                'strSamplingProvenance'};
            cellMissingFields = setdiff(cellRequiredFields, fieldnames(strRealization), 'stable');
            cellUnknownFields = setdiff(fieldnames(strRealization), cellRequiredFields, 'stable');
            if ~isempty(cellMissingFields) || ~isempty(cellUnknownFields)
                error(charErrorId, ...
                    'A perturbation realization must contain exactly the documented fixed fields.');
            end

            if ~islogical(strRealization.bApplied) || ~isscalar(strRealization.bApplied)
                error(charErrorId, ...
                    'bApplied must be one logical scalar.');
            end

            % Read the seed and the three error vectors
            strRealization.ui32Seed = STargetRotationalStateConfig.ReadSeed_(strRealization.ui32Seed, ...
                'ui32Seed');
            strRealization.dInitialAttitudeTangent_IN = STargetRotationalStateConfig.ReadVector_( ...
                strRealization.dInitialAttitudeTangent_IN, ...
                uint32(3), ...
                'dInitialAttitudeTangent_IN', ...
                charErrorId);
            strRealization.dAngularVelocityError = STargetRotationalStateConfig.ReadVector_( ...
                strRealization.dAngularVelocityError, ...
                uint32(3), ...
                'dAngularVelocityError', ...
                charErrorId);
            strRealization.dInertiaChartError = STargetRotationalStateConfig.ReadVector_( ...
                strRealization.dInertiaChartError, ...
                uint32(6), ...
                'dInertiaChartError', ...
                charErrorId);

            % Preserve caller-owned sampling metadata without interpreting its schema or coupling
            % SimulationGears to campaign policy.
            if ~isstruct(strRealization.strSamplingProvenance) || ~isscalar(strRealization.strSamplingProvenance)
                error(charErrorId, ...
                    'strSamplingProvenance must be one scalar struct.');
            end

            % An unapplied realization carries zero errors, and an applied one a positive seed
            bHasNonzeroError = any(strRealization.dInitialAttitudeTangent_IN ~= 0.0) || ...
                any(strRealization.dAngularVelocityError ~= 0.0) || ...
                any(strRealization.dInertiaChartError ~= 0.0);
            if ~strRealization.bApplied && bHasNonzeroError
                error(charErrorId, ...
                    'An unapplied realization must have zero perturbation values.');
            end
            if strRealization.bApplied && strRealization.ui32Seed == uint32(0)
                error(charErrorId, ...
                    'An applied realization must identify a positive realization seed.');
            end

            % Constant-rate truth has no inertia model, so a realization cannot carry an inertia chart error
            if enumDynamicsModel == EnumTargetRotationalDynamicsModel.KINEMATIC_CONST_INERTIAL_RATE && ...
                    any(strRealization.dInertiaChartError ~= 0.0)
                error(charErrorId, ...
                    'Constant-inertial-rate truth has no inertia model. The inertia chart error must be zero.');
            end
        end

        function dQuaternion = CanonicalizeQuaternion(dQuaternion)
            %% SIGNATURE
            % dQuaternion = STargetRotationalStateConfig.CanonicalizeQuaternion(dQuaternion)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Normalize a finite nonzero scalar-first quaternion and make its largest-magnitude component
            % positive, the sign representative stored in dInitialQuat_INfromTB.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % dQuaternion    (4,1) double candidate quaternion
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dQuaternion    (4,1) double normalized quaternion
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % 12-11-2026  Pietro Califano, Claude Opus 5.5   Make public for
            %                                                ApplyTargetRotationalStatePerturbation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                dQuaternion (4,1) double {mustBeReal}
            end
            arguments (Output)
                dQuaternion (4,1) double
            end

            dQuaternionNorm = norm(dQuaternion);
            if ~isfinite(dQuaternionNorm) || dQuaternionNorm <= 1.0e-14
                error('STargetRotationalStateConfig:InvalidQuaternion', ...
                    'initial_quaternion_INfromTB must be finite and nonzero.');
            end
            dQuaternion = dQuaternion / dQuaternionNorm;

            % Select one deterministic representative of the q/-q equivalence class.
            [~, dLargestComponentIdx] = max(abs(dQuaternion));
            if dQuaternion(dLargestComponentIdx) < 0.0
                dQuaternion = -dQuaternion;
            end
        end
    end

    methods (Static, Access = private)
        function varValue = GetField_(strConfig, charFieldName, varDefault)
            %% SIGNATURE
            % varValue = STargetRotationalStateConfig.GetField_(strConfig, charFieldName, varDefault)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return the named field of strConfig, or varDefault when the field is absent.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % strConfig       (1,1) struct public configuration
            % charFieldName   (1,:) char field name
            % varDefault      value returned for an absent field
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % varValue        field value or default
            % -------------------------------------------------------------------------------------------------

            if isfield(strConfig, charFieldName)
                varValue = strConfig.(charFieldName);
            else
                varValue = varDefault;
            end
        end

        function dVector = ReadVector_(varValue, ui32ExpectedLength, charFieldName, charErrorId)
            %% SIGNATURE
            % dVector = STargetRotationalStateConfig.ReadVector_(varValue, ui32ExpectedLength, charFieldName, ...
            %     charErrorId)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Read one finite real numeric vector of the expected length as a double column. Raise charErrorId
            % on an invalid value.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % varValue             numeric candidate vector
            % ui32ExpectedLength   (1,1) uint32 required number of entries
            % charFieldName        (1,:) char field name for diagnostics
            % charErrorId          (1,:) char error identifier
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dVector              (:,1) double validated vector
            % -------------------------------------------------------------------------------------------------

            if ~isnumeric(varValue) || ~isreal(varValue) || numel(varValue) ~= double(ui32ExpectedLength)
                error(charErrorId, ...
                    '%s must contain exactly %u real numeric entries.', ...
                    charFieldName, ...
                    ui32ExpectedLength);
            end

            dVector = reshape(double(varValue), [], 1);
            if any(~isfinite(dVector))
                error(charErrorId, ...
                    '%s must contain only finite values.', charFieldName);
            end
        end

        function dMatrix = ReadMatrix3_(varValue, charFieldName)
            %% SIGNATURE
            % dMatrix = STargetRotationalStateConfig.ReadMatrix3_(varValue, charFieldName)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Read one finite real numeric 3x3 matrix as double.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % varValue        numeric candidate matrix
            % charFieldName   (1,:) char field name for diagnostics
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dMatrix         (3,3) double validated matrix
            % -------------------------------------------------------------------------------------------------

            if ~isnumeric(varValue) || ~isreal(varValue) || ~isequal(size(varValue), [3, 3])
                error('STargetRotationalStateConfig:InvalidMatrix', ...
                    '%s must be one real numeric 3-by-3 matrix.', charFieldName);
            end

            dMatrix = double(varValue);
            if any(~isfinite(dMatrix), 'all')
                error('STargetRotationalStateConfig:InvalidMatrix', ...
                    '%s must contain only finite values.', charFieldName);
            end
        end

        function ui32Seed = ReadSeed_(varSeed, charFieldName)
            %% SIGNATURE
            % ui32Seed = STargetRotationalStateConfig.ReadSeed_(varSeed, charFieldName)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Read one nonnegative integer within the uint32 range as uint32.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % varSeed         numeric candidate scalar
            % charFieldName   (1,:) char field name for diagnostics
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % ui32Seed        (1,1) uint32 validated seed
            % -------------------------------------------------------------------------------------------------

            if ~isnumeric(varSeed) || ~isreal(varSeed) || ~isscalar(varSeed)
                error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                    '%s must be one nonnegative uint32-compatible integer.', charFieldName);
            end

            dSeed = double(varSeed);
            if ~isfinite(dSeed) || dSeed < 0.0 || dSeed ~= fix(dSeed) || dSeed > double(intmax('uint32'))
                error('STargetRotationalStateConfig:InvalidPerturbationProvenance', ...
                    '%s must be one nonnegative uint32-compatible integer.', charFieldName);
            end
            ui32Seed = uint32(dSeed);
        end
    end
end
