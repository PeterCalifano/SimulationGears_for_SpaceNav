function strDynParams = EphemeridesDataFactory(dEphemTimegrid, ...
                                                ui32EphemerisPolyDeg,...
                                                ui32AttitudePolyDeg, ...
                                                strDynParams, ...
                                                strMainBodyRefData, ...
                                                str3rdBodyRefData, ...
                                                kwargs)
arguments
    dEphemTimegrid
    ui32EphemerisPolyDeg
    ui32AttitudePolyDeg
    strDynParams
    strMainBodyRefData
    str3rdBodyRefData = []
end
arguments
    kwargs.bGroundTruthEphemerides  (1,1) logical = true
    kwargs.bEnableInterpValidation  (1,1) logical = true
    kwargs.bAdd3rdBodiesPosition    (1,1) logical = true
    kwargs.bAdd3rdBodiesAttitude    (1,1) logical = false
    kwargs.bUseInterpFcnFromRCS1    (1,1) logical = false
    kwargs.bScaleTimeToDays         (1,1) logical = false
    kwargs.bUseAbsoluteTimegrid     (1,1) logical = false;
end
%% SIGNATURE
% strDynParams = EphemeridesDataFactory(dEphemTimegrid, ui32EphemerisPolyDeg, ui32AttitudePolyDeg, ...
%     strDynParams, strMainBodyRefData, str3rdBodyRefData, kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Fit target and third-body ephemeris products into the runtime dynamics structure. When the source provides target
% attitude-model rate, preserve it as source data aligned with the attitude ephemeris; never recover it by differencing
% sampled attitudes. The model rate satisfies R_INfromTB(t) = Exp(-omega_IN*t) R_INfromTB(0). This builder expects the
% field contract produced by DefineEnvironmentProperties and is not a standalone source loader.
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
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% DCM2quatSeq, fitAttQuatChbvPolynmials, fitChbvPolynomials.
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

% Preserve the angular velocity delivered by the ephemeris source. This is
% source data, not a numerical derivative of the sampled attitude sequence.
if isfield(strMainBodyRefData, 'dAngVel_IN')
    dTargetAngVel_IN = strMainBodyRefData.dAngVel_IN;
    if not(isequal(size(dTargetAngVel_IN), [3, numel(dEphemTimegrid)])) || ...
            any(not(isfinite(dTargetAngVel_IN)), 'all')
        error('EphemeridesDataFactory:InvalidTargetAngularVelocity', ...
            ['Source-provided target angular velocity must be a finite 3-by-N sequence aligned with the ' ...
             'ephemeris timegrid.']);
    end
end

if not(kwargs.bUseInterpFcnFromRCS1)

    % Use nav-system implementation
    [dTmpChbvCoeffs, ~, dTmpSwitchIntervals, ...
            strTmpFitStats] = fitAttQuatChbvPolynmials(ui32AttitudePolyDeg, ...
                                                        dInterpDomain, ...
                                                        dQuat_WfromTB, ...
                                                        dDomainLB, ...
                                                        dDomainUB, ...
                                                        kwargs.bEnableInterpValidation); %#ok<ASGLU>

    strDynParams.strMainData.strAttData.ui32PolyDeg          = ui32AttitudePolyDeg;
    strDynParams.strMainData.strAttData.dChbvPolycoeffs      = dTmpChbvCoeffs;
    strDynParams.strMainData.strAttData.dsignSwitchIntervals = dTmpSwitchIntervals;
    strDynParams.strMainData.strAttData.dTimeLowBound        = dDomainLB;
    strDynParams.strMainData.strAttData.dTimeUpBound         = dDomainUB;
    if isfield(strMainBodyRefData, 'dAngVel_IN')
        strDynParams.strMainData.strAttData.dNominalAngVel_IN = dTargetAngVel_IN(:,1);
        strDynParams.strMainData.strAttData.dAngVel_IN = dTargetAngVel_IN;
        strDynParams.strMainData.strAttData.dAngVelTimegrid = dInterpDomain;
    end

else
    % Use implementation for RCS-1
    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
    % hardcoded inside the evaluation funtion!).

    % Apply discontinuity fix
    [dQuat_WfromTB, bIsSignSwitched, ui8howManySwitches, ...
        bsignSwitchDetectionMask] = fixQuatSignDiscontinuity(dQuat_WfromTB'); %#ok<ASGLU>

    strDynParams.strMainData.d_gnc_eph_target_att_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                                dQuat_WfromTB', ...
                                                                                ui32AttitudePolyDeg);
    
    strDynParams.strMainData.d_gnc_eph_target_att_tbounds = [dInterpDomain(1), dInterpDomain(end)];
    strDynParams.strMainData.ui32CoeffsSizePtr = size(strDynParams.strMainData.d_gnc_eph_target_att_coeffs, 2);

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
    for idB = 1:length(str3rdBodyRefData)
        % Get position data
        if isfield(str3rdBodyRefData, "strOrbitData")
            try
                dPosition_W = str3rdBodyRefData(idB).strOrbitData.dPosition_W;

                if not(kwargs.bUseInterpFcnFromRCS1)
                    % Use nav-system implementation
                    [dChbvCoeffs, ~, ~] = fitChbvPolynomials(ui32EphemerisPolyDeg, ...
                                                            dInterpDomain, ...
                                                            dPosition_W, ...
                                                            dDomainLB, ...
                                                            dDomainUB, ...
                                                            kwargs.bEnableInterpValidation);

                    strDynParams.strBody3rdData(idB+1).strOrbitData.ui32PolyDeg      = ui32EphemerisPolyDeg;
                    strDynParams.strBody3rdData(idB+1).strOrbitData.dChbvPolycoeffs  = dChbvCoeffs;
                    strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeLowBound    = dDomainLB;
                    strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeUpBound     = dDomainUB;


                else
                    % Use implementation for RCS-1

                    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
                    % hardcoded inside the evaluation funtion!).

                    strDynParams.strBody3rdData(idB+1).d_gnc_eph_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                                            dPosition_W', ...
                                                                                            ui32EphemerisPolyDeg);
                    strDynParams.strBody3rdData(idB+1).d_gnc_eph_time_bounds = [dInterpDomain(1), dInterpDomain(end)] ;
                    strDynParams.strBody3rdData(idB+1).ui32CoeffsSizePtr = size(strDynParams.strBody3rdData(idB+1).d_gnc_eph_coeffs, 2);
                end

            catch ME

                warning("EphemeridesDataFactory: Failed to fit orbit data for 3rd body %d due to error: %s. \nSkipping ephemeris fitting.", idB, string(ME.message));
                if not(kwargs.bUseInterpFcnFromRCS1)
                    % Use nav-system implementation
                    strDynParams.strBody3rdData(idB+1).strOrbitData.ui32PolyDeg      = 0;
                    strDynParams.strBody3rdData(idB+1).strOrbitData.dChbvPolycoeffs  = [];
                    strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeLowBound    = [];
                    strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeUpBound     = [];

                else
                    % Use implementation for RCS-1
                    strDynParams.strBody3rdData(idB+1).d_gnc_eph_coeffs = [];
                    strDynParams.strBody3rdData(idB+1).d_gnc_eph_time_bounds = [];
                    strDynParams.strBody3rdData(idB+1).ui32CoeffsSizePtr = [];
                end
            end
        else
            warning("EphemeridesDataFactory: No orbit data found for 3rd body %d. Skipping ephemeris fitting.", idB);
        end

        if isfield(str3rdBodyRefData(idB), "strAttData") && kwargs.bAdd3rdBodiesAttitude
            try
                % Fit attitude data (convert to quaternion scalar first)
                dQuat_WfromTB = DCM2quatSeq(str3rdBodyRefData(idB).strAttData.dDCM_WfromTB , false);

                if not(kwargs.bUseInterpFcnFromRCS1)
                    % Use nav-system implementation
                    [dTmpChbvCoeffs, ~, dTmpSwitchIntervals, ...
                        strTmpFitStats] = fitAttQuatChbvPolynmials(ui32AttitudePolyDeg, ...
                                                                    dInterpDomain, ...
                                                                    dQuat_WfromTB, ...
                                                                    dDomainLB, ...
                                                                    dDomainUB, ...
                                                                    kwargs.bEnableInterpValidation); %#ok<ASGLU>

                    strDynParams.strBody3rdData(idB+1).strAttData.ui32PolyDeg          = ui32AttitudePolyDeg;
                    strDynParams.strBody3rdData(idB+1).strAttData.dChbvPolycoeffs      = dTmpChbvCoeffs;
                    strDynParams.strBody3rdData(idB+1).strAttData.dsignSwitchIntervals = dTmpSwitchIntervals;
                    strDynParams.strBody3rdData(idB+1).strAttData.dTimeLowBound        = dDomainLB;
                    strDynParams.strBody3rdData(idB+1).strAttData.dTimeUpBound         = dDomainUB;

                else
                    % Use implementation for RCS-1

                    % DEVNOTE: interpolation time domain is assumed to be expressed in days (conversion is
                    % hardcoded inside the evaluation funtion!).

                    % Apply discontinuity fix
                    [dQuat_WfromTB, bIsSignSwitched, ui8howManySwitches, ...
                        bsignSwitchDetectionMask] = fixQuatSignDiscontinuity(dQuat_WfromTB'); %#ok<ASGLU>

                    strDynParams.strBody3rdData(idB+1).strAttData.d_gnc_eph_coeffs = EphCoeffsGeneration(dInterpDomain, ...
                                                                                                    dQuat_WfromTB', ...
                                                                                                    ui32EphemerisPolyDeg);
                    
                    strDynParams.strBody3rdData(idB+1).strAttData.d_gnc_eph_time_bounds = [dInterpDomain(1), dInterpDomain(end)] ;
                    strDynParams.strBody3rdData(idB+1).strAttData.ui32CoeffsSizePtr = size(strDynParams.strBody3rdData(idB+1).strAttData.d_gnc_eph_coeffs, 2);
                end
            catch ME
                warning("EphemeridesDataFactory: Failed to fit attitude data for 3rd body %d due to error: %s. \nSkipping attitude fitting.", idB, string(ME.message));
                strDynParams.strBody3rdData(idB+1).strAttData.ui32PolyDeg          = 0;
                strDynParams.strBody3rdData(idB+1).strAttData.dChbvPolycoeffs      = [];
                strDynParams.strBody3rdData(idB+1).strAttData.dsignSwitchIntervals = [];
                strDynParams.strBody3rdData(idB+1).strAttData.dTimeLowBound        = [];
                strDynParams.strBody3rdData(idB+1).strAttData.dTimeUpBound         = [];
            end
        end
    
    end

elseif length(strDynParams.strBody3rdData) > 1
    warning("EphemeridesDataFactory: No 3rd body reference data provided but >1 3rd bodies defined in strDynParams. No ephmerides will be available for these. NOTE: 1st entry is reserved for the Sun.");
end

end
