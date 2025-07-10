function [strDynParams, strMainBodyRefData] = EphemeridesDataFactory(dEphemTimegrid, ...
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
    kwargs.bGroundTruthEphemerides (1,1) logical {isscalar, islogical} = true
    kwargs.bEnableInterpValidation (1,1) logical {isscalar, islogical} = true
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
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-02-2025    Pietro Califano     First version copy-pasting previous implementation
% 10-07-2025    Pietro Califano     Extend to support definition of 3rd body attitude and position
%                                   ephemerides from input reference data
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
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
% ACHTUNG: Make sure that unit of measure for distance matches.
dDistFromSunAU = mean(vecnorm(strMainBodyRefData.dSunPosition_IN, 2, 1), "all") /150e9; % Input distance in [m]
strDynParams.strSRPdata.dP_SRP = 1367/physconst('lightspeed') * (1/(dDistFromSunAU)^2); % [N/m^2]

% Printing of key value: Distance in AU from the SUN
fprintf('\nAverage distance from the SUN in AU over ET_SPAN: %3.4f AU\n', dDistFromSunAU);

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
if not(isempty(str3rdBodyRefData))

    for idB = 1:length(str3rdBodyRefData)
        % Get position data

        assert(isfield(str3rdBodyRefData, "strOrbitData"));
        dPosition_IN = str3rdBodyRefData(idB).strOrbitData.dPosition_IN;

        [dChbvCoeffs, ~, ~] = fitChbvPolynomials(ui32PolyDeg, ...
                                                 dInterpDomain, ...
                                                    dPosition_IN, ...
                                                    dDomainLB, ...
                                                    dDomainUB, ...
                                                    bENABLE_AUTO_CHECK);

        strDynParams.strBody3rdData(idB+1).strOrbitData.ui32PolyDeg      = ui32EphemerisPolyDeg;
        strDynParams.strBody3rdData(idB+1).strOrbitData.dChbvPolycoeffs  = dChbvCoeffs;
        strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeLowBound    = dDomainLB;
        strDynParams.strBody3rdData(idB+1).strOrbitData.dTimeUpBound     = dDomainUB;
    
        if isfield(str3rdBodyRefData(idB), "strAttData")

            % Fit attitude data (convert to quaternion first
            dQuat_INfromTB = DCM2quatSeq(str3rdBodyRefData(idB).strAttData.dDCM_INfromTB , false);

            [dTmpChbvCoeffs, ~, dTmpSwitchIntervals, ...
                strTmpFitStats] = fitAttQuatChbvPolynmials(ui32AttitudePolyDeg, ...
                                                            dInterpDomain, ...
                                                            dQuat_INfromTB, ...
                                                            dDomainLB, ...
                                                            dDomainUB, ...
                                                            kwargs.bEnableInterpValidation); %#ok<ASGLU>

            strDynParams.strBody3rdData(idB+1).strAttData.ui32PolyDeg          = ui32AttitudePolyDeg;
            strDynParams.strBody3rdData(idB+1).strAttData.dChbvPolycoeffs      = dTmpChbvCoeffs;
            strDynParams.strBody3rdData(idB+1).strAttData.dsignSwitchIntervals = dTmpSwitchIntervals;
            strDynParams.strBody3rdData(idB+1).strAttData.dTimeLowBound        = dDomainLB;
            strDynParams.strBody3rdData(idB+1).strAttData.dTimeUpBound         = dDomainUB;

        end
    
    end



end



end
