function strDynParams = EphemeridesDataFactory(dEphemTimegrid, ...
                                                ui32EphemerisPolyDeg, ...
                                                ui32AttitudePolyDeg, ...
                                                strDynParams, ...
                                                strMainBodyRefData, ...
                                                str3rdBodyRefData, ...
                                                kwargs)
arguments (Input)
    dEphemTimegrid
    ui32EphemerisPolyDeg
    ui32AttitudePolyDeg
    strDynParams
    strMainBodyRefData
    str3rdBodyRefData = []
end
arguments (Input)
    kwargs.bGroundTruthEphemerides  (1,1) logical = true
    kwargs.bEnableInterpValidation  (1,1) logical = true
    kwargs.bAdd3rdBodiesPosition    (1,1) logical = true
    kwargs.bAdd3rdBodiesAttitude    (1,1) logical = false
    kwargs.bUseInterpFcnFromRCS1    (1,1) logical = false
    kwargs.bScaleTimeToDays         (1,1) logical = false
    kwargs.bUseAbsoluteTimegrid     (1,1) logical = false;
end
arguments (Output)
    strDynParams (1,1) struct
end
%% SIGNATURE
% strDynParams = EphemeridesDataFactory(dEphemTimegrid, ui32EphemerisPolyDeg, ui32AttitudePolyDeg, ...
%     strDynParams, strMainBodyRefData, str3rdBodyRefData, kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Fit target and third-body ephemeris products into the runtime dynamics structure. When the source provides target
% attitude-model rate, preserve it as source data aligned with the attitude ephemeris; never recover it by differencing
% sampled attitudes. The model rate satisfies R_INfromTB(t) = Exp(+omega_IN*t) R_INfromTB(0). Rates are in rad/s and
% their timegrid is in the selected interpolation domain (relative or absolute seconds, or absolute days), for both
% coefficient layouts. The typed angular-velocity regime alone decides whether rates exist: UNAVAILABLE carries no rate
% samples, CONSTANT_INERTIAL and TIME_VARYING need a complete validated sequence. This builder expects the field
% contract produced by DefineEnvironmentProperties and the target-history builders; it is not a standalone source loader.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dEphemTimegrid               Ephemeris sample epochs [s].
% ui32EphemerisPolyDeg         Position interpolation polynomial degree.
% ui32AttitudePolyDeg          Attitude interpolation polynomial degree.
% strDynParams                 Dynamics structure to populate.
% strMainBodyRefData           Target attitude, attitude-model rate, and Sun-position source data.
% str3rdBodyRefData            Optional third-body source data.
% kwargs                       Interpolation, validation, time-domain, and third-body options.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strDynParams                 Dynamics structure containing fitted ephemeris products.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-02-2025    Pietro Califano     First version copy-pasting previous implementation
% 22-07-2025    Pietro Califano     Extend to support definition of 3rd body attitude and position
%                                   ephemerides from input reference data; minor updates
% 18-08-2025    Pietro Califano     Update implementation to generalize RCS1 alternative code branch
% 13-08-2026    Pietro Califano, Codex gpt-5.6     Preserve source-provided target angular velocity.
% 22-08-2026    Pietro Califano, Codex              Separate variable-rate history from constant-rate capability.
% 23-08-2026    Pietro Califano, Codex              Replace optional nominal-rate capability with a typed regime.
% 11-09-2026  Pietro Califano, Codex gpt-6    Remove unused runtime sign-switch metadata.
% 29-09-2026  Pietro Califano, Codex gpt-6    Preserve target rates in both interpolation layouts.
% 13-11-2026  Pietro Califano, Claude Opus 5.5   Prefix the target rate and spin-axis fields with Target.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% DCM2quatSeq, EnumTargetAngularVelocityRegime, fitAttQuatChbvPolynmials, fitChbvPolynomials.
% -------------------------------------------------------------------------------------------------------------

%% Common data
% Build interpolant timegrid
if not(kwargs.bUseInterpFcnFromRCS1) && not(kwargs.bUseAbsoluteTimegrid)
    % nav-system implementation (relative timestamps)
    dInterpDomain = dEphemTimegrid - dEphemTimegrid(1);
    dDomainLB = min(dInterpDomain);
    dDomainUB = max(dInterpDomain);

else
    % RCS-1 implementation (absolute timestamps) or if kwargs.bUseAbsoluteTimegrid is true
    dInterpDomain = dEphemTimegrid;

    if kwargs.bScaleTimeToDays
        dInterpDomain = dInterpDomain / 86400.0;
    end

    dDomainLB = min(dInterpDomain);
    dDomainUB = max(dInterpDomain);


end

%% Target body attitude data
% Convert attitude DCMs to quaternion
dQuat_WfromTB = DCM2quatSeq(strMainBodyRefData.dDCM_INfromTB, false);

% Persist a resolved body-fixed spin axis only as a complete value/provenance
% pair. An absent pair means that no single body-fixed axis is known.
bHasTargetSpinAxis = isfield(strMainBodyRefData, 'dTargetSpinAxis_TB');
bHasTargetSpinAxisSource = isfield(strMainBodyRefData, 'charTargetSpinAxisSource');
if bHasTargetSpinAxis ~= bHasTargetSpinAxisSource
    error('EphemeridesDataFactory:IncompleteTargetSpinAxisMetadata', ...
        'Target spin-axis value and provenance must be supplied together.');
end

if bHasTargetSpinAxis
    varTargetSpinAxis = strMainBodyRefData.dTargetSpinAxis_TB;
    varTargetSpinAxisSource = strMainBodyRefData.charTargetSpinAxisSource;

    % Validate the raw values before conversion, so that a complex axis or a
    % numeric source cannot pass as valid metadata.
    bValidTargetSpinAxisSourceType = ...
        (ischar(varTargetSpinAxisSource) && isrow(varTargetSpinAxisSource)) || ...
        (isstring(varTargetSpinAxisSource) && isscalar(varTargetSpinAxisSource));
    if ~isnumeric(varTargetSpinAxis) || ~isreal(varTargetSpinAxis) || ...
            not(isequal(size(varTargetSpinAxis), [3, 1])) || ...
            any(not(isfinite(varTargetSpinAxis))) || ...
            ~bValidTargetSpinAxisSourceType
        error('EphemeridesDataFactory:InvalidTargetSpinAxis', ...
            ['The resolved target spin axis must be one finite real numeric unit ', ...
             '3-vector with nonempty char or string-scalar provenance.']);
    end

    dTargetSpinAxis_TB = double(varTargetSpinAxis);
    charTargetSpinAxisSource = string(varTargetSpinAxisSource);
    if abs(norm(dTargetSpinAxis_TB) - 1.0) > 1.0e-12 || ...
            strlength(strip(charTargetSpinAxisSource)) == 0
        error('EphemeridesDataFactory:InvalidTargetSpinAxis', ...
            ['The resolved target spin axis must be one finite real numeric unit ', ...
             '3-vector with nonempty char or string-scalar provenance.']);
    end
end

% Dispatch on the typed regime, never on field presence. Rate-bearing regimes
% need source rates; UNAVAILABLE must carry none.
enumTargetAngularVelocityRegime = strMainBodyRefData.enumTargetAngularVelocityRegime;
if ~isa(enumTargetAngularVelocityRegime, 'EnumTargetAngularVelocityRegime') || ...
        ~isscalar(enumTargetAngularVelocityRegime)
    error('EphemeridesDataFactory:InvalidAngularVelocityRegime', ...
        ['Target angular-velocity capability must be one scalar ', ...
         'EnumTargetAngularVelocityRegime value.']);
end

bHasTargetAngVel_IN = isfield(strMainBodyRefData, 'dTargetAngVel_IN');
bHasTargetAngVel_TB = isfield(strMainBodyRefData, 'dTargetAngVel_TB');
switch enumTargetAngularVelocityRegime
    case EnumTargetAngularVelocityRegime.UNAVAILABLE
        if bHasTargetAngVel_IN || bHasTargetAngVel_TB
            error('EphemeridesDataFactory:InconsistentAngularVelocityRegime', ...
                ['UNAVAILABLE target angular velocity cannot carry inertial ', ...
                 'or body-frame rate samples.']);
        end

    case {EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL, ...
          EnumTargetAngularVelocityRegime.TIME_VARYING}
        if ~bHasTargetAngVel_IN
            error('EphemeridesDataFactory:InvalidTargetAngularVelocity', ...
                'A rate-bearing target regime requires dTargetAngVel_IN.');
        end
        varTargetAngVel_IN = strMainBodyRefData.dTargetAngVel_IN;
        if ~isnumeric(varTargetAngVel_IN) || ~isreal(varTargetAngVel_IN) || ...
                not(isequal(size(varTargetAngVel_IN), [3, numel(dEphemTimegrid)])) || ...
                any(not(isfinite(varTargetAngVel_IN)), 'all')
            error('EphemeridesDataFactory:InvalidTargetAngularVelocity', ...
                ['Source-provided target angular velocity must be a finite real ', ...
                 'numeric 3-by-N sequence aligned with the ephemeris timegrid.']);
        end
        dTargetAngVel_IN = double(varTargetAngVel_IN);

        % Require the declared regime to agree with every rate sample.
        enumClassifiedRegime = ...
            EnumTargetAngularVelocityRegime.Classify(dTargetAngVel_IN);
        if enumTargetAngularVelocityRegime ~= enumClassifiedRegime
            error('EphemeridesDataFactory:InconsistentAngularVelocityRegime', ...
                ['The declared target angular-velocity regime is inconsistent ', ...
                 'with the complete inertial-rate history.']);
        end

    otherwise
        error('EphemeridesDataFactory:InvalidAngularVelocityRegime', ...
            'Unsupported target angular-velocity regime.');
end

if bHasTargetAngVel_TB
    varTargetAngVel_TB = strMainBodyRefData.dTargetAngVel_TB;
    if ~isnumeric(varTargetAngVel_TB) || ~isreal(varTargetAngVel_TB) || ...
            not(isequal(size(varTargetAngVel_TB), [3, numel(dEphemTimegrid)])) || ...
            any(not(isfinite(varTargetAngVel_TB)), 'all')
        error('EphemeridesDataFactory:InvalidTargetBodyAngularVelocity', ...
            ['Source-provided target body angular velocity must be a finite ', ...
             'real numeric 3-by-N sequence aligned with the ephemeris timegrid.']);
    end
    dTargetAngVel_TB = double(varTargetAngVel_TB);
end

% Replace the factory-owned rate and spin-axis fields instead of merging them,
% so that a reused dynamics payload keeps no stale source metadata.
if isfield(strDynParams.strMainData, 'strAttData')
    cellAttitudeContextFields = {'dTargetAngVel_IN', 'dTargetAngVelTimegrid', ...
                                 'dNominalAngVel_IN', 'dTargetAngVel_TB', ...
                                 'enumTargetAngularVelocityRegime', ...
                                 'dTargetSpinAxis_TB', ...
                                 'charTargetSpinAxisSource'};
    cellPresentAttitudeFields = intersect(cellAttitudeContextFields, ...
        fieldnames(strDynParams.strMainData.strAttData), 'stable');
    if ~isempty(cellPresentAttitudeFields)
        strDynParams.strMainData.strAttData = rmfield( ...
            strDynParams.strMainData.strAttData, ...
            cellPresentAttitudeFields);
    end
end

if not(kwargs.bUseInterpFcnFromRCS1)

    % Use nav-system implementation
    [dTmpChbvCoeffs, ~, ~, ...
            strTmpFitStats] = fitAttQuatChbvPolynmials(ui32AttitudePolyDeg, ...
                                                        dInterpDomain, ...
                                                        dQuat_WfromTB, ...
                                                        dDomainLB, ...
                                                        dDomainUB, ...
                                                        kwargs.bEnableInterpValidation); %#ok<ASGLU>

    strDynParams.strMainData.strAttData.ui32PolyDeg          = ui32AttitudePolyDeg;
    strDynParams.strMainData.strAttData.dChbvPolycoeffs      = dTmpChbvCoeffs;
    strDynParams.strMainData.strAttData.dTimeLowBound        = dDomainLB;
    strDynParams.strMainData.strAttData.dTimeUpBound         = dDomainUB;

else
    % Use implementation for RCS-1
    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
    % hardcoded inside the evaluation funtion!).

    % Apply discontinuity fix
    [dQuat_WfromTB, bIsSignSwitched, ui8HowManySwitches, ...
        bSignSwitchDetectionMask] = fixQuatSignDiscontinuity(dQuat_WfromTB'); %#ok<ASGLU>

    strDynParams.strMainData.d_gnc_eph_target_att_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                                dQuat_WfromTB', ...
                                                                                ui32AttitudePolyDeg);
    
    strDynParams.strMainData.d_gnc_eph_target_att_tbounds = [dInterpDomain(1), dInterpDomain(end)];
    strDynParams.strMainData.ui32CoeffsSizePtr = size(strDynParams.strMainData.d_gnc_eph_target_att_coeffs, 2);

end

% The typed regime is always present in the runtime attitude payload. Only
% rate-bearing regimes carry aligned samples and their interpolation domain.
strDynParams.strMainData.strAttData.enumTargetAngularVelocityRegime = ...
    enumTargetAngularVelocityRegime;
if enumTargetAngularVelocityRegime ~= EnumTargetAngularVelocityRegime.UNAVAILABLE
    strDynParams.strMainData.strAttData.dTargetAngVel_IN = ...
        dTargetAngVel_IN;
    strDynParams.strMainData.strAttData.dTargetAngVelTimegrid = ...
        dInterpDomain;
end
if bHasTargetAngVel_TB
    strDynParams.strMainData.strAttData.dTargetAngVel_TB = dTargetAngVel_TB;
end

% Keep known spin-axis analysis provenance available in both interpolation
% layouts. Unknown or time-varying axes remain absent.
if bHasTargetSpinAxis
    strDynParams.strMainData.strAttData.dTargetSpinAxis_TB = ...
        dTargetSpinAxis_TB;
    strDynParams.strMainData.strAttData.charTargetSpinAxisSource = ...
        charTargetSpinAxisSource;
end

%% Sun position
if not(kwargs.bUseInterpFcnFromRCS1)
    [dTmpChbvCoeffs, ~] = fitChbvPolynomials(ui32EphemerisPolyDeg, ...
                                                dInterpDomain, ...
                                                strMainBodyRefData.dSunPosition_IN, ...
                                                dDomainLB, ...
                                                dDomainUB, ...
                                                kwargs.bEnableInterpValidation);

    strDynParams.strBody3rdData(1).strOrbitData.ui32PolyDeg      = ui32EphemerisPolyDeg;
    strDynParams.strBody3rdData(1).strOrbitData.dChbvPolycoeffs  = dTmpChbvCoeffs;
    strDynParams.strBody3rdData(1).strOrbitData.dTimeLowBound    = dDomainLB;
    strDynParams.strBody3rdData(1).strOrbitData.dTimeUpBound     = dDomainUB;

else
    % Use implementation for RCS-1

    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
    % hardcoded inside the evaluation funtion!).
    strDynParams.strBody3rdData(1).d_gnc_eph_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                        strMainBodyRefData.dSunPosition_IN', ...
                                                                        ui32EphemerisPolyDeg);

    strDynParams.strBody3rdData(1).d_gnc_eph_time_bounds = [dInterpDomain(1), dInterpDomain(end)];
    strDynParams.strBody3rdData(1).ui32CoeffsSizePtr = size(strDynParams.strBody3rdData(1).d_gnc_eph_coeffs, 2);
end

%% Add 3rd bodies
if not(isempty(str3rdBodyRefData)) && length(strDynParams.strBody3rdData) > 1 && ( kwargs.bAdd3rdBodiesPosition || kwargs.bAdd3rdBodiesAttitude)
    for ui32BodyIdx = uint32(1):uint32(length(str3rdBodyRefData))
        % Get position data
        if isfield(str3rdBodyRefData, "strOrbitData")
            try
                dPosition_W = str3rdBodyRefData(ui32BodyIdx).strOrbitData.dPosition_W;

                if not(kwargs.bUseInterpFcnFromRCS1)
                    % Use nav-system implementation
                    [dChbvCoeffs, ~, ~] = fitChbvPolynomials(ui32EphemerisPolyDeg, ...
                                                            dInterpDomain, ...
                                                            dPosition_W, ...
                                                            dDomainLB, ...
                                                            dDomainUB, ...
                                                            kwargs.bEnableInterpValidation);

                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.ui32PolyDeg      = ui32EphemerisPolyDeg;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.dChbvPolycoeffs  = dChbvCoeffs;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.dTimeLowBound    = dDomainLB;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.dTimeUpBound     = dDomainUB;


                else
                    % Use implementation for RCS-1

                    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
                    % hardcoded inside the evaluation funtion!).

                    strDynParams.strBody3rdData(ui32BodyIdx+1).d_gnc_eph_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                                            dPosition_W', ...
                                                                                            ui32EphemerisPolyDeg);
                    strDynParams.strBody3rdData(ui32BodyIdx+1).d_gnc_eph_time_bounds = [dInterpDomain(1), dInterpDomain(end)] ;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).ui32CoeffsSizePtr = size(strDynParams.strBody3rdData(ui32BodyIdx+1).d_gnc_eph_coeffs, 2);
                end

            catch objError

                warning("EphemeridesDataFactory: Failed to fit orbit data for 3rd body %d due to error: %s. \nSkipping ephemeris fitting.", ui32BodyIdx, string(objError.message));
                if not(kwargs.bUseInterpFcnFromRCS1)
                    % Use nav-system implementation
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.ui32PolyDeg      = 0;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.dChbvPolycoeffs  = [];
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.dTimeLowBound    = [];
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strOrbitData.dTimeUpBound     = [];

                else
                    % Use implementation for RCS-1
                    strDynParams.strBody3rdData(ui32BodyIdx+1).d_gnc_eph_coeffs = [];
                    strDynParams.strBody3rdData(ui32BodyIdx+1).d_gnc_eph_time_bounds = [];
                    strDynParams.strBody3rdData(ui32BodyIdx+1).ui32CoeffsSizePtr = [];
                end
            end
        else
            warning("EphemeridesDataFactory: No orbit data found for 3rd body %d. Skipping ephemeris fitting.", ui32BodyIdx);
        end

        if isfield(str3rdBodyRefData(ui32BodyIdx), "strAttData") && kwargs.bAdd3rdBodiesAttitude
            try
                % Fit attitude data (convert to quaternion scalar first)
                dQuat_WfromTB = DCM2quatSeq(str3rdBodyRefData(ui32BodyIdx).strAttData.dDCM_WfromTB , false);

                if not(kwargs.bUseInterpFcnFromRCS1)
                    % Use nav-system implementation
                    [dTmpChbvCoeffs, ~, ~, ...
                        strTmpFitStats] = fitAttQuatChbvPolynmials(ui32AttitudePolyDeg, ...
                                                                    dInterpDomain, ...
                                                                    dQuat_WfromTB, ...
                                                                    dDomainLB, ...
                                                                    dDomainUB, ...
                                                                    kwargs.bEnableInterpValidation); %#ok<ASGLU>

                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.ui32PolyDeg          = ui32AttitudePolyDeg;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.dChbvPolycoeffs      = dTmpChbvCoeffs;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.dTimeLowBound        = dDomainLB;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.dTimeUpBound         = dDomainUB;

                else
                    % Use implementation for RCS-1

                    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
                    % hardcoded inside the evaluation funtion!).

                    % Apply discontinuity fix
                    [dQuat_WfromTB, bIsSignSwitched, ui8HowManySwitches, ...
                        bSignSwitchDetectionMask] = fixQuatSignDiscontinuity(dQuat_WfromTB'); %#ok<ASGLU>

                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.d_gnc_eph_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                                                    dQuat_WfromTB', ...
                                                                                                    ui32EphemerisPolyDeg);
                    
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.d_gnc_eph_time_bounds = [dInterpDomain(1), dInterpDomain(end)] ;
                    strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.ui32CoeffsSizePtr = size(strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.d_gnc_eph_coeffs, 2);
                end
            catch objError
                warning("EphemeridesDataFactory: Failed to fit attitude data for 3rd body %d due to error: %s. \nSkipping attitude fitting.", ui32BodyIdx, string(objError.message));
                strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.ui32PolyDeg          = 0;
                strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.dChbvPolycoeffs      = [];
                strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.dTimeLowBound        = [];
                strDynParams.strBody3rdData(ui32BodyIdx+1).strAttData.dTimeUpBound         = [];
            end
        end
    
    end

elseif length(strDynParams.strBody3rdData) > 1
    warning("EphemeridesDataFactory: No 3rd body reference data provided but >1 3rd bodies defined in strDynParams. No ephmerides will be available for these. NOTE: 1st entry is reserved for the Sun.");
end

end
