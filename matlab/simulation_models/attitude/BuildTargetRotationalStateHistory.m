function strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strSourceHistory, kwargs)
%% SIGNATURE
% strHistory = BuildTargetRotationalStateHistory(objConfig, dTimegrid, strSourceHistory, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate and normalize a source-provided target rotational history, generate a constant-inertial-rate history,
% or integrate torque-free rigid-body kino-dynamics. All outputs use scalar-first Hamilton/passive quaternions
% internally, R_INfromTB attitude matrices, inertial and body-frame angular velocities when available, and one
% shared timegrid.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objConfig          (1,1) STargetRotationalStateConfig source and dynamics configuration
% dTimegrid          (1,N) double strictly increasing output epochs [s]
% strSourceHistory   (1,1) struct optional source attitude/rate/spin-axis history
% kwargs.enumBackend (1,1) EnumTargetRotationalStateBackend explicit MATLAB/MEX integration backend
% kwargs.charMexOutputRoot (1,:) char optional generated-artifact root used only in MEX mode
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHistory         (1,1) struct normalized target rotational-state history
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% 23-08-2026  Pietro Califano, Codex     Adapt MathCore passive attitudes to the public R_INfromTB contract.
% 24-08-2026  Pietro Califano, Codex     Add explicit opt-in MEX history integration.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% 12-11-2026  Pietro Califano, Claude Opus 5.5   Check the configuration with
%                                                STargetRotationalStateConfig.validate; check DCMs with ValidateDCM
%                                                and drop the quaternion sign steps, which do not change the DCMs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CQuatKinematicsIntegrator.NormalizeSeq, DCM2quatSeq, RunTorqueFreeBodyHistory,
% EnumTargetRotationalStateBackend, EnumTargetAngularVelocityRegime, QuatSeq2DCM, RotationVectorToDCM,
% STargetRotationalStateConfig.validate, ValidateDCM.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    objConfig (1,1) STargetRotationalStateConfig
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
    strSourceHistory (1,1) struct = struct()
    kwargs.enumBackend (1,1) EnumTargetRotationalStateBackend = EnumTargetRotationalStateBackend.MATLAB
    kwargs.charMexOutputRoot (1,:) char = ''
end
arguments (Output)
    strHistory (1,1) struct
end

% Require a valid configuration, which pairs each dynamics model with its source, and at least two strictly
% increasing epochs
objConfig.validate();
if numel(dTimegrid) < 2 || any(diff(dTimegrid) <= 0.0)
    error('BuildTargetRotationalStateHistory:InvalidTimegrid', ...
        'Target rotational-state timegrid must contain at least two strictly increasing finite epochs.');
end

% Generated execution accelerates the numerical torque-free integrator only. Reject model combinations that
% would otherwise ignore an explicit request.
if kwargs.enumBackend == EnumTargetRotationalStateBackend.MEX && ...
        objConfig.enumDynamicsModel ~= EnumTargetRotationalDynamicsModel.RIGID_BODY_TORQUE_FREE
    error('BuildTargetRotationalStateHistory:BackendUnsupportedForModel', ...
        ['The MEX target rotational-state backend requires ', ...
        'RIGID_BODY_TORQUE_FREE dynamics.']);
end

% Build the history of the configured dynamics model
switch objConfig.enumDynamicsModel
    case EnumTargetRotationalDynamicsModel.SOURCE_PROVIDED
        strHistory = BuildSourceProvidedHistory_(strSourceHistory, dTimegrid);

    case EnumTargetRotationalDynamicsModel.KINEMATIC_CONST_INERTIAL_RATE
        strHistory = BuildConstantInertialRateHistory_(objConfig, dTimegrid);

    case EnumTargetRotationalDynamicsModel.RIGID_BODY_TORQUE_FREE
        strHistory = BuildTorqueFreeHistory_(objConfig, ...
            dTimegrid, ...
            kwargs.enumBackend, ...
            kwargs.charMexOutputRoot);
end

end

function strHistory = BuildSourceProvidedHistory_(strSourceHistory, dTimegrid)
%% SIGNATURE
% strHistory = BuildSourceProvidedHistory_(strSourceHistory, dTimegrid)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Normalize one source attitude history, reconcile optional inertial/body angular velocities, and classify the
% complete rate sequence through the typed angular-velocity regime.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strSourceHistory    (1,1) struct source target rotational-state fields
% dTimegrid           (1,N) double output epochs [s]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHistory          (1,1) struct normalized source history
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% NormalizeDcmHistory_, ResolveAngularVelocityHistory_, CopySpinAxisMetadata_.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strSourceHistory (1,1) struct
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
end
arguments (Output)
    strHistory (1,1) struct
end

% Require the source attitude
if ~isfield(strSourceHistory, 'dDCM_INfromTB')
    error('BuildTargetRotationalStateHistory:MissingSourceAttitude', ...
        'Source-provided target rotation requires dDCM_INfromTB.');
end

% Normalize the attitude, then add the rates and the spin-axis metadata
strHistory = struct();
strHistory.dDCM_INfromTB = NormalizeDcmHistory_(strSourceHistory.dDCM_INfromTB, uint32(numel(dTimegrid)));
strHistory = ResolveAngularVelocityHistory_(strHistory, strSourceHistory, dTimegrid);
strHistory = CopySpinAxisMetadata_(strHistory, strSourceHistory);

end

function strHistory = BuildConstantInertialRateHistory_(objConfig, dTimegrid)
%% SIGNATURE
% strHistory = BuildConstantInertialRateHistory_(objConfig, dTimegrid)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate the analytic attitude and rate history for one constant inertial angular velocity.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objConfig    (1,1) STargetRotationalStateConfig generated constant-rate configuration
% dTimegrid    (1,N) double output epochs [s]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHistory   (1,1) struct generated target rotational-state history
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CQuatKinematicsIntegrator.NormalizeSeq, QuatSeq2DCM, RotationVectorToDCM, NormalizeDcmHistory_.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    objConfig (1,1) STargetRotationalStateConfig
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
end
arguments (Output)
    strHistory (1,1) struct
end

% Rotate the initial attitude by the constant inertial rate at each epoch
dInitialQuaternion = CQuatKinematicsIntegrator.NormalizeSeq(objConfig.dInitialQuat_INfromTB);
dInitialDCM_INfromTB = QuatSeq2DCM(dInitialQuaternion, false);
dInitialAngVel_IN = objConfig.dInitialAngVel_IN;
ui32NumTimes = uint32(numel(dTimegrid));

dDCM_INfromTB = zeros(3, 3, double(ui32NumTimes));
for ui32TimeIdx = uint32(1):ui32NumTimes
    dElapsedTime = dTimegrid(ui32TimeIdx) - dTimegrid(1);
    dDCM_INfromTB(:, :, ui32TimeIdx) = RotationVectorToDCM(dElapsedTime * dInitialAngVel_IN) * dInitialDCM_INfromTB;
end
dDCM_INfromTB = NormalizeDcmHistory_(dDCM_INfromTB, ui32NumTimes);

% Express the constant inertial rate in the body frame
dAngVel_IN = repmat(dInitialAngVel_IN, 1, double(ui32NumTimes));
dAngVel_TB = zeros(3, double(ui32NumTimes));
for ui32TimeIdx = uint32(1):ui32NumTimes
    dAngVel_TB(:, ui32TimeIdx) = transpose(dDCM_INfromTB(:, :, ui32TimeIdx)) * dInitialAngVel_IN;
end

strHistory = struct();
strHistory.dDCM_INfromTB = dDCM_INfromTB;
strHistory.dAngVel_IN = dAngVel_IN;
strHistory.dAngVel_TB = dAngVel_TB;
strHistory.enumAngularVelocityRegime = EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL;

% A nonzero constant rate defines one actual body-fixed spin axis. Zero angular velocity has no unique axis and
% therefore carries no axis metadata.
dBodyRateNorm = norm(dAngVel_TB(:, 1));
if dBodyRateNorm > 1.0e-14
    strHistory.dSpinAxis_TB = dAngVel_TB(:, 1) / dBodyRateNorm;
    strHistory.charSpinAxisSource = "GENERATED_CONSTANT_RATE";
end

end

function strHistory = BuildTorqueFreeHistory_(objConfig, dTimegrid, enumBackend, charMexOutputRoot)
%% SIGNATURE
% strHistory = BuildTorqueFreeHistory_(objConfig, dTimegrid, enumBackend, charMexOutputRoot)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Integrate zero-torque rigid-body angular velocity and quaternion kino-dynamics on a relative nonnegative
% timegrid. The requested maximum internal step is passed explicitly to avoid dependence on integrator step
% deduction.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objConfig    (1,1) STargetRotationalStateConfig generated torque-free configuration
% dTimegrid    (1,N) double output epochs [s]
% enumBackend  (1,1) EnumTargetRotationalStateBackend explicit integration backend
% charMexOutputRoot (1,:) char optional generated-artifact root
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHistory   (1,1) struct generated torque-free target rotational-state history
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CQuatKinematicsIntegrator.NormalizeSeq, DCM2quatSeq, QuatSeq2DCM, RunTorqueFreeBodyHistory.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    objConfig (1,1) STargetRotationalStateConfig
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
    enumBackend (1,1) EnumTargetRotationalStateBackend
    charMexOutputRoot (1,:) char
end
arguments (Output)
    strHistory (1,1) struct
end

dRelativeTimegrid = dTimegrid - dTimegrid(1);

% MathCore propagates the passive inertial-to-body attitude paired with body-frame Euler rates. Convert the
% public body-to-inertial attitude only at this boundary so neither library exposes a frame-selection flag.
dInitialQuaternion_INfromTB = CQuatKinematicsIntegrator.NormalizeSeq(objConfig.dInitialQuat_INfromTB);
dInitialDCM_INfromTB = QuatSeq2DCM(dInitialQuaternion_INfromTB, false);
dInitialQuaternion_TBfromIN = DCM2quatSeq(transpose(dInitialDCM_INfromTB), false);

% SimulationGears owns the complete history loop; MathCore owns only the stateless numerical steps used
% internally.
[dQuaternionHistory_TBfromIN, dAngVel_TB] = RunTorqueFreeBodyHistory(dRelativeTimegrid, ...
    dInitialQuaternion_TBfromIN, ...
    objConfig.dInitialAngVel_TB, ...
    objConfig.dInertiaTensor_TB, ...
    objConfig.dMaxIntegrationStep, ...
    "enumBackend", enumBackend, ...
    "charMexOutputRoot", charMexOutputRoot);

% Convert the history back to R_INfromTB and express the body rates in the inertial frame
dQuaternionHistory_TBfromIN = CQuatKinematicsIntegrator.NormalizeSeq(dQuaternionHistory_TBfromIN);
dDCM_TBfromIN = QuatSeq2DCM(dQuaternionHistory_TBfromIN, false);
dDCM_INfromTB = permute(dDCM_TBfromIN, [2, 1, 3]);
dAngVel_IN = zeros(size(dAngVel_TB));
for ui32TimeIdx = uint32(1):uint32(numel(dTimegrid))
    dAngVel_IN(:, ui32TimeIdx) = dDCM_INfromTB(:, :, ui32TimeIdx) * dAngVel_TB(:, ui32TimeIdx);
end

strHistory = struct();
strHistory.dDCM_INfromTB = dDCM_INfromTB;
strHistory.dAngVel_IN = dAngVel_IN;
strHistory.dAngVel_TB = dAngVel_TB;
strHistory.enumAngularVelocityRegime = EnumTargetAngularVelocityRegime.Classify(dAngVel_IN);

end

function dDCM_INfromTB = NormalizeDcmHistory_(dDCM_INfromTB, ui32ExpectedTimes)
%% SIGNATURE
% dDCM_INfromTB = NormalizeDcmHistory_(dDCM_INfromTB, ui32ExpectedTimes)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate a finite proper-orthogonal 3-by-3-by-N DCM sequence and reconstruct it through normalized
% scalar-first Hamilton/passive quaternions.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dDCM_INfromTB      (3,3,N) double target attitude sequence
% ui32ExpectedTimes  (1,1) uint32 expected number of samples
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDCM_INfromTB      (3,3,N) double normalized target attitude sequence
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CQuatKinematicsIntegrator.NormalizeSeq, DCM2quatSeq, QuatSeq2DCM, ValidateDCM.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dDCM_INfromTB double {mustBeReal}
    ui32ExpectedTimes (1,1) uint32 {mustBePositive}
end
arguments (Output)
    dDCM_INfromTB double
end

% Require a finite 3x3xN array aligned with the timegrid
if size(dDCM_INfromTB, 1) ~= 3 || size(dDCM_INfromTB, 2) ~= 3 || ...
        size(dDCM_INfromTB, 3) ~= double(ui32ExpectedTimes) || ...
        any(~isfinite(dDCM_INfromTB), 'all')
    error('BuildTargetRotationalStateHistory:InvalidDcmHistory', ...
        'Target attitude history must be one finite 3-by-3-by-N sequence aligned with the timegrid.');
end

% Require every sample to be a proper rotation matrix within 1e-10
for ui32TimeIdx = uint32(1):ui32ExpectedTimes
    if ~ValidateDCM(dDCM_INfromTB(:, :, ui32TimeIdx), 1.0e-10, false)
        error('BuildTargetRotationalStateHistory:InvalidDcmHistory', ...
            'Target attitude sample %u is not a proper orthogonal DCM.', ui32TimeIdx);
    end
end

% Rebuild the DCMs from normalized quaternions. The quaternion sign does not change the DCM.
dQuaternionHistory = DCM2quatSeq(dDCM_INfromTB, false);
dQuaternionHistory = CQuatKinematicsIntegrator.NormalizeSeq(dQuaternionHistory);
dDCM_INfromTB = QuatSeq2DCM(dQuaternionHistory, false);

end

function strHistory = ResolveAngularVelocityHistory_(strHistory, strSourceHistory, dTimegrid)
%% SIGNATURE
% strHistory = ResolveAngularVelocityHistory_(strHistory, strSourceHistory, dTimegrid)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate optional source angular velocities, derive the complementary frame representation, and classify the
% complete inertial-rate sequence through the typed capability enum.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strHistory        (1,1) struct normalized attitude history
% strSourceHistory  (1,1) struct optional source angular-velocity histories
% dTimegrid         (1,N) double output epochs [s]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHistory        (1,1) struct history with reconciled rate fields
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strHistory (1,1) struct
    strSourceHistory (1,1) struct
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
end
arguments (Output)
    strHistory (1,1) struct
end

% Reject dNominalAngVel_IN, which is not part of the history contract
if isfield(strSourceHistory, 'dNominalAngVel_IN')
    error('BuildTargetRotationalStateHistory:UnsupportedNominalAngularVelocityField', ...
        ['dNominalAngVel_IN is not part of the canonical history contract. ', ...
        'Use enumAngularVelocityRegime after complete-history validation.']);
end

% Mark the rates unavailable when the source carries neither frame
ui32NumTimes = uint32(numel(dTimegrid));
bHasInertialRate = isfield(strSourceHistory, 'dAngVel_IN');
bHasBodyRate = isfield(strSourceHistory, 'dAngVel_TB');

if ~bHasInertialRate && ~bHasBodyRate
    strHistory.enumAngularVelocityRegime = EnumTargetAngularVelocityRegime.UNAVAILABLE;
    return
end

% Derive the other frame from the inertial rates when present, else from the body rates
if bHasInertialRate
    dAngVel_IN = ValidateRateHistory_(strSourceHistory.dAngVel_IN, ui32NumTimes, 'dAngVel_IN');
    dAngVel_TB = zeros(3, double(ui32NumTimes));
    for ui32TimeIdx = uint32(1):ui32NumTimes
        dAngVel_TB(:, ui32TimeIdx) = transpose(strHistory.dDCM_INfromTB(:, :, ui32TimeIdx)) * ...
            dAngVel_IN(:, ui32TimeIdx);
    end
else
    dAngVel_TB = ValidateRateHistory_(strSourceHistory.dAngVel_TB, ui32NumTimes, 'dAngVel_TB');
    dAngVel_IN = zeros(3, double(ui32NumTimes));
    for ui32TimeIdx = uint32(1):ui32NumTimes
        dAngVel_IN(:, ui32TimeIdx) = strHistory.dDCM_INfromTB(:, :, ui32TimeIdx) * dAngVel_TB(:, ui32TimeIdx);
    end
end

% With both frames given, require the source body rates to match the derived ones
if bHasBodyRate && bHasInertialRate
    dSourceAngVel_TB = ValidateRateHistory_(strSourceHistory.dAngVel_TB, ui32NumTimes, 'dAngVel_TB');
    dRateTolerance = 1.0e-12 * max(1.0, max(abs(dAngVel_TB), [], 'all'));
    if max(abs(dSourceAngVel_TB - dAngVel_TB), [], 'all') > dRateTolerance
        error('BuildTargetRotationalStateHistory:InconsistentAngularVelocity', ...
            'Source inertial and body-frame angular-velocity histories are inconsistent with attitude.');
    end
end

strHistory.dAngVel_IN = dAngVel_IN;
strHistory.dAngVel_TB = dAngVel_TB;
strHistory.enumAngularVelocityRegime = EnumTargetAngularVelocityRegime.Classify(dAngVel_IN);

end

function dRateHistory = ValidateRateHistory_(dRateHistory, ui32ExpectedTimes, charFieldName)
%% SIGNATURE
% dRateHistory = ValidateRateHistory_(dRateHistory, ui32ExpectedTimes, charFieldName)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate one finite 3-by-N angular-velocity sequence.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRateHistory       double candidate angular-velocity sequence
% ui32ExpectedTimes  (1,1) uint32 expected number of samples
% charFieldName      (1,:) char source field name for diagnostics
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dRateHistory       (3,N) double validated angular-velocity sequence
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dRateHistory double {mustBeReal}
    ui32ExpectedTimes (1,1) uint32 {mustBePositive}
    charFieldName (1,:) char
end
arguments (Output)
    dRateHistory double
end

if ~isequal(size(dRateHistory), [3, double(ui32ExpectedTimes)]) || any(~isfinite(dRateHistory), 'all')
    error('BuildTargetRotationalStateHistory:InvalidAngularVelocity', ...
        '%s must be one finite 3-by-N sequence aligned with the timegrid.', charFieldName);
end

end

function strHistory = CopySpinAxisMetadata_(strHistory, strSourceHistory)
%% SIGNATURE
% strHistory = CopySpinAxisMetadata_(strHistory, strSourceHistory)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Copy one explicitly paired finite unit body spin axis and its source provenance.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strHistory        (1,1) struct normalized target history
% strSourceHistory  (1,1) struct optional spin-axis metadata
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHistory        (1,1) struct history with validated optional spin-axis metadata
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strHistory (1,1) struct
    strSourceHistory (1,1) struct
end
arguments (Output)
    strHistory (1,1) struct
end

% Require the axis and its provenance together; copy nothing when both are absent
bHasSpinAxis = isfield(strSourceHistory, 'dSpinAxis_TB');
bHasSpinAxisSource = isfield(strSourceHistory, 'charSpinAxisSource');
if bHasSpinAxis ~= bHasSpinAxisSource
    error('BuildTargetRotationalStateHistory:IncompleteSpinAxisMetadata', ...
        'Source spin-axis value and provenance must be supplied together.');
end
if ~bHasSpinAxis
    return
end

% Check the value types: a real numeric 3-vector and a char row or string scalar
varSpinAxis_TB = strSourceHistory.dSpinAxis_TB;
varSpinAxisSource = strSourceHistory.charSpinAxisSource;
bValidSpinAxisSourceType = (ischar(varSpinAxisSource) && isrow(varSpinAxisSource)) || ...
    (isstring(varSpinAxisSource) && isscalar(varSpinAxisSource));
if ~isnumeric(varSpinAxis_TB) || ~isreal(varSpinAxis_TB) || numel(varSpinAxis_TB) ~= 3 || ...
        ~bValidSpinAxisSourceType
    error('BuildTargetRotationalStateHistory:InvalidSpinAxisMetadata', ...
        ['Source spin axis must be one finite real numeric unit 3-vector ', ...
        'with nonempty char or string-scalar provenance.']);
end

% Check the values: a finite unit axis and a nonblank provenance
dSpinAxis_TB = reshape(double(varSpinAxis_TB), [], 1);
charSpinAxisSource = string(varSpinAxisSource);
if any(~isfinite(dSpinAxis_TB)) || ...
        abs(norm(dSpinAxis_TB) - 1.0) > 1.0e-12 || ...
        strlength(strip(charSpinAxisSource)) == 0
    error('BuildTargetRotationalStateHistory:InvalidSpinAxisMetadata', ...
        ['Source spin axis must be one finite real numeric unit 3-vector ', ...
        'with nonempty char or string-scalar provenance.']);
end

strHistory.dSpinAxis_TB = dSpinAxis_TB;
strHistory.charSpinAxisSource = charSpinAxisSource;

end
