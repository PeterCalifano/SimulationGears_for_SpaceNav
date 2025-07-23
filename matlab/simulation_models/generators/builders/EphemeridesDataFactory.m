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
    kwargs.bGroundTruthEphemerides  (1,1) logical {isscalar, islogical} = true
    kwargs.bEnableInterpValidation  (1,1) logical {isscalar, islogical} = true
    kwargs.bAdd3rdBodiesPosition    (1,1) logical {isscalar, islogical} = true
    kwargs.bAdd3rdBodiesAttitude    (1,1) logical {isscalar, islogical} = false
end
% TODO
%% SIGNATURE
% [strDynParams, strMainBodyRefData] = EphemeridesDataFactory(dEphemTimegrid, ...
%                                                             ui32EphemerisPolyDeg,...
%                                                             ui32AttitudePolyDeg, ...
%                                                             strDynParams, ...
%                                                             strMainBodyRefData, ...
%                                                             str3rdBodyRefData, ...
%                                                             kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% ACHTUNG: this function assumes that the input structs have the correct fields in place. Not intended for
% standalone usage. DefineEnvironmentProperties() should be called first.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dEphemTimegrid
% ui32EphemerisPolyDeg
% ui32AttitudePolyDeg
% strDynParams
% strMainBodyRefData
% str3rdBodyRefData = []
% kwargs.bGroundTruthEphemerides  (1,1) logical {isscalar, islogical} = true
% kwargs.bEnableInterpValidation  (1,1) logical {isscalar, islogical} = true
% kwargs.bAdd3rdBodiesPosition    (1,1) logical {isscalar, islogical} = true
% kwargs.bAdd3rdBodiesAttitude    (1,1) logical {isscalar, islogical} = false
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strDynParams
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-02-2025    Pietro Califano     First version copy-pasting previous implementation
% 22-07-2025    Pietro Califano     Extend to support definition of 3rd body attitude and position
%                                   ephemerides from input reference data; minor updates
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Common data
% Build interpolant timegrid
dInterpDomain = dEphemTimegrid - dEphemTimegrid(1);
dDomainLB = min(dInterpDomain);
dDomainUB = max(dInterpDomain);

%% Target body attitude data
% Convert attitude DCMs to quaternion
dQuat_INfromTB = DCM2quatSeq(strMainBodyRefData.dDCM_INfromTB, false);

[dTmpChbvCoeffs, ~, dTmpSwitchIntervals, ...
    strTmpFitStats] = fitAttQuatChbvPolynmials(ui32AttitudePolyDeg, ...
                                                dInterpDomain, ...
                                                dQuat_INfromTB, ...
                                                dDomainLB, ...
                                                dDomainUB, ...
                                                kwargs.bEnableInterpValidation); %#ok<ASGLU>

strDynParams.strMainData.strAttData.ui32PolyDeg          = ui32AttitudePolyDeg;
strDynParams.strMainData.strAttData.dChbvPolycoeffs      = dTmpChbvCoeffs;
strDynParams.strMainData.strAttData.dsignSwitchIntervals = dTmpSwitchIntervals;
strDynParams.strMainData.strAttData.dTimeLowBound        = dDomainLB;
strDynParams.strMainData.strAttData.dTimeUpBound         = dDomainUB;


%% Sun position
[dTmpChbvCoeffs, ~, ~] = fitChbvPolynomials(ui32EphemerisPolyDeg, ...
                                         dInterpDomain, ...
                                         strMainBodyRefData.dSunPosition_IN, ...
                                         dDomainLB, ...
                                         dDomainUB, ...
                                         kwargs.bEnableInterpValidation);

strDynParams.strBody3rdData(1).strOrbitData.ui32PolyDeg      = ui32EphemerisPolyDeg;
strDynParams.strBody3rdData(1).strOrbitData.dChbvPolycoeffs  = dTmpChbvCoeffs;
strDynParams.strBody3rdData(1).strOrbitData.dTimeLowBound    = dDomainLB;
strDynParams.strBody3rdData(1).strOrbitData.dTimeUpBound     = dDomainUB;

%% Add 3rd bodies
if not(isempty(str3rdBodyRefData)) && length(strDynParams.strBody3rdData) > 1 && ( kwargs.bAdd3rdBodiesPosition || kwargs.bAdd3rdBodiesAttitude)
    for idB = 1:length(str3rdBodyRefData)
        % Get position data
        if isfield(str3rdBodyRefData, "strOrbitData")
            try
                dPosition_W = str3rdBodyRefData(idB).strOrbitData.dPosition_W;
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

            catch ME
                warning("EphemeridesDataFactory: Failed to fit orbit data for 3rd body %d due to error: %s. \nSkipping ephemeris fitting.", idB, string(ME.message));
                strDynParams.strBody3rdData(idB+1).strOrbitData.ui32PolyDeg      = 0;
                strDynParams.strBody3rdData(idB+1).strOrbitData.dChbvPolycoeffs  = [];
                strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeLowBound    = [];
                strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeUpBound     = [];
            end
        else
            warning("EphemeridesDataFactory: No orbit data found for 3rd body %d. Skipping ephemeris fitting.", idB);
        end

        if isfield(str3rdBodyRefData(idB), "strAttData") && kwargs.bAdd3rdBodiesAttitude
            try
                % Fit attitude data (convert to quaternion scalar first)
                dQuat_WfromTB = DCM2quatSeq(str3rdBodyRefData(idB).strAttData.dDCM_WfromTB , false);

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
