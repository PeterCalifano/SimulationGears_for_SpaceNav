function [strDynParams, strAdditionalData] = DefineEnvironmentProperties(dEphemeridesTimegrid, ...
                                                                          enumScenarioName, ...
                                                                          charInertialFrame, ...
                                                                          kwargs)
arguments
    dEphemeridesTimegrid  (1,:) double 
    enumScenarioName    EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])} = EnumScenarioName.Itokawa
    charInertialFrame   (1,:) char {mustBeA(charInertialFrame, ["string", "char"])} = "J2000"
end
arguments
    kwargs.strDynParams (1,1) {isstruct} = struct() % To provide input struct
    kwargs.bAddNonSphericalGravityCoeffs (1,1) logical {islogical, isscalar} = false;
    kwargs.objDataset = SReferenceMissionDesign()
    kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
end
%% SIGNATURE
% [strDynParams, strAdditionalData] = DefineEnvironmentProperties(dEphemeridesTimegrid, ...
%                                                                 enumScenarioName, ...
%                                                                 charInertialFrame, ...
%                                                                 kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function defining the dynamical properties for the specified scenario. Timegrid and inertial frame are
% specified to query specify or the input dataset object to define Sun and attitude ephemerides as position
% and rotation matrices.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments
%     dEphemeridesTimegrid  (1,:) double 
%     enumScenarioName    EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])} = EnumScenarioName.Itokawa
%     charInertialFrame   (1,:) char {mustBeA(charInertialFrame, ["string", "char"])} = "J2000"
% end
% arguments
%     kwargs.strDynParams (1,1) {isstruct} = struct() % To provide input struct
%     kwargs.bAddNonSphericalGravityCoeffs (1,1) logical {islogical, isscalar} = false;
%     kwargs.objDataset = SReferenceMissionDesign()
%     kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strDynParams
% strAdditionalData
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-02-2025    Pietro Califano     First version copy-pasting previous implementation
% 14-03-2025    Pietro Califano     Move code to CScenarioGenerator static method for standardization
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------

strDynParams = kwargs.strDynParams;

% Checks and info printing
if kwargs.objDataset.bDefaultConstructed
    fprintf("No or default constructed dataset object provided as input. Target attitude and Ephemerides fetching will be attempted using CSPICE.\n")
end

%% Target body data (scenario dependent)
[charTargetName, charTargetFixedFrame, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData(enumScenarioName, ...
                                                                                strDynParams, ...
                                                                                "bAddNonSphericalGravityCoeffs", kwargs.bAddNonSphericalGravityCoeffs, ...
                                                                                "charSpherHarmCoeffInputFileName", kwargs.charSpherHarmCoeffInputFileName);

if kwargs.objDataset.bDefaultConstructed

    if max(dEphemeridesTimegrid) < 1e8
        warning('Minimum time in ephemeris timegrid < 1e6. This is being used to query CSPICE but seems to small. Make sure it is as intended!')
    end

    % Get target fixed frame attitude wrt Inertial frame    
    strAdditionalData.dDCM_INfromTB = cspice_pxform(char(charTargetFixedFrame), char(charInertialFrame), dEphemeridesTimegrid);

    % Get Sun position in Inertial frame
    strAdditionalData.dSunPosition_IN = 1000 * cspice_spkpos('SUN', dEphemeridesTimegrid, ...
                                                charInertialFrame, 'none', charTargetName);

    % Get data from objDataset

else
    % Load ephemeris data from dataset
    assert(length(dEphemeridesTimegrid) == length(kwargs.objDataset.dTimestamps), ...
            "ERROR: objDataset timestamps do not match specified dEphemerisTimegrid.")

    strAdditionalData.dDCM_INfromTB    = pagetranspose(kwargs.objDataset.dDCM_TBfromW);
    strAdditionalData.dSunPosition_IN  = kwargs.objDataset.dSunPosition_W;
end


%% Solar Radiation Pressure data
% ACHTUNG: Make sure that unit of measure for distance matches.
dDistFromSunAU = mean(vecnorm(strAdditionalData.dSunPosition_IN, 2, 1), "all") / 150e9; % Input distance in [m]
strDynParams.strSRPdata.dP_SRP0 = 1367/299792458 * (1/(dDistFromSunAU)^2); % [N/m^2]
fprintf('\nAverage distance from the SUN in AU: %3.4f AU\n', dDistFromSunAU);

% Get Sun gravity parameter
strDynParams.strBody3rdData(1).dGM = 1E+09 * cspice_bodvrd('SUN', 'GM', 1); % Output is in km^3/s^2
% strDynParams.strBody3rdData(1).dGM = cspice_bodvrd('SUN', 'GM', 1); % Output is in km^3/s^2

%% Spacecraft data
strDynParams.strSCdata.dReflCoeff = 1.29;  % Global CR
strDynParams.strSCdata.dSCmass    = 14.8; % 12; % [kg]
strDynParams.strSCdata.dA_SRP     = 0.5329; % [m^2]

end




