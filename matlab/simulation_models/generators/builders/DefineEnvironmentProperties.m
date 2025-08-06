function [strDynParams, strMainBodyRefData, str3rdBodyRefData] = DefineEnvironmentProperties(dEphemeridesTimegrid, ...
                                                                                          enumScenarioName, ...
                                                                                          charInertialFrame, ...
                                                                                          kwargs)
arguments
    dEphemeridesTimegrid  (1,:) double 
    enumScenarioName    EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])} = EnumScenarioName.Itokawa
    charInertialFrame   (1,:) char {mustBeA(charInertialFrame, ["string", "char"])} = "J2000"
end
arguments
    kwargs.strDynParams                     (1,1) {isstruct} = struct()         % Initialization value
    kwargs.str3rdBodyRefData                (1,1) {isstruct} = struct()    % Initialization value
    kwargs.bAddNonSphericalGravityCoeffs    (1,1) logical {islogical, isscalar} = false;
    kwargs.objDataset = SReferenceMissionDesign()
    kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
    kwargs.cellAdditionalBodiesNames       (1,:) string {mustBeA(kwargs.cellAdditionalBodiesNames, ["string", "char"])} = string.empty(0, 1)
    kwargs.bAdd3rdBodiesAttitude           (1,1) logical {islogical, isscalar} = true; % If true, attitude data will be added to str3rdBodyRefData
    kwargs.bUseKilometersScale             (1,1) logical {isscalar, islogical} = false;
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
% 21-07-2025    Pietro Califano     Add support for 3rd body reference data and generalize implementation
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

strDynParams = kwargs.strDynParams;
str3rdBodyRefData = kwargs.str3rdBodyRefData;

if isempty(fieldnames(strDynParams))
    strDynParams.strSCdata = struct();
else
    for idField = 1:length(fieldnames(strDynParams))
        if not(isfield(strDynParams, "strSCdata"))
            strDynParams.strSCdata = struct();
        end
    end
end

% Checks and info printing
if kwargs.objDataset.bDefaultConstructed
    fprintf("No or default constructed dataset object provided as input. Target attitude and Ephemerides fetching will be attempted using CSPICE.\n")
end

%% Target body data (scenario dependent)
[charTargetName, charTargetFixedFrame, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData(enumScenarioName, ...
                                                                                strDynParams, ...
                                                                                "bAddNonSphericalGravityCoeffs", kwargs.bAddNonSphericalGravityCoeffs, ...
                                                                                "charSpherHarmCoeffInputFileName", kwargs.charSpherHarmCoeffInputFileName, ...
                                                                                "bUseKilometersScale", kwargs.bUseKilometersScale);


if kwargs.bUseKilometersScale
    dUnitsScaling = 1.0;
else
    dUnitsScaling = 1E3;
end

if kwargs.objDataset.bDefaultConstructed

    if max(dEphemeridesTimegrid) < 1e8
        warning('Minimum time in ephemeris timegrid < 1e6. This is being used to query CSPICE but seems to small. Make sure it is as intended!')
    end

    % Get target fixed frame attitude wrt Inertial frame    
    strMainBodyRefData.dDCM_INfromTB = cspice_pxform(char(charTargetFixedFrame), ...
                                                    char(charInertialFrame), ...
                                                    dEphemeridesTimegrid);

    % Get Sun position in Inertial frame
    strMainBodyRefData.dSunPosition_IN = dUnitsScaling * cspice_spkpos('SUN', dEphemeridesTimegrid, ...
                                                charInertialFrame, 'none', charTargetName);

    % Get additional bodies data if provided (Sun not included)
    ui32NumOfAdditionalBodies = length(kwargs.cellAdditionalBodiesNames);
    bAdd3rdBodiesAttitude = kwargs.bAdd3rdBodiesAttitude;
    % TODO implement code to fetch from SPICE! --> additional bodies tags specified by kwargs.cellAdditionalBodiesNames
    for idB = 1:ui32NumOfAdditionalBodies

    end

else
    % Load ephemeris data from dataset
    assert(length(dEphemeridesTimegrid) == length(kwargs.objDataset.dTimestamps), ...
            "ERROR: objDataset timestamps do not match specified dEphemerisTimegrid.")

    strMainBodyRefData.dDCM_INfromTB    = pagetranspose(kwargs.objDataset.dDCM_TBfromW);
    strMainBodyRefData.dSunPosition_IN  = kwargs.objDataset.dSunPosition_W;

    % Get additional bodies data if provided (Sun not included)
    ui32NumOfAdditionalBodies = length(kwargs.objDataset.cellAdditionalBodiesTags);
    bAdd3rdBodiesAttitude = not(isempty(kwargs.objDataset.cellAdditionalBodiesDCM_TBfromW)) && kwargs.bAdd3rdBodiesAttitude;

    for idB = 1:ui32NumOfAdditionalBodies

        if not(isempty(kwargs.objDataset.cellAdditionalBodiesPos_W{idB}))
            % Store data in struct for ephemerides factory
            str3rdBodyRefData(idB).strOrbitData.dPosition_W  = kwargs.objDataset.cellAdditionalBodiesPos_W{idB};
        end
        
        if bAdd3rdBodiesAttitude && not(isempty(kwargs.objDataset.cellAdditionalBodiesDCM_TBfromW{idB}))
            str3rdBodyRefData(idB).strAttData.dDCM_WfromTB = pagetranspose(kwargs.objDataset.cellAdditionalBodiesDCM_TBfromW{idB});
        end

        try
            charTargetName = kwargs.objDataset.cellAdditionalBodiesTags{idB};
            fprintf('\nLoading additional body %s data from CScenarioGenerator database...', charTargetName);
            [charTargetName, charTargetFixedFrame, strDynParams_3rdBody] = CScenarioGenerator.LoadDefaultScenarioData(charTargetName, ...
                                                                                                    "bUseKilometersScale", kwargs.bUseKilometersScale);

            % Add GM information if available to scenario generator
            strDynParams.strBody3rdData(idB+1).dGM        = strDynParams_3rdBody.strMainData.dGM;
            strDynParams.strBody3rdData(idB+1).dRefRadius = strDynParams_3rdBody.strMainData.dRefRadius;

        catch ME
            warning('ERROR occurred while assigning data of 3rd bodies: %s.\nBody %s not available in CScenarioGenerator. GM will be set to zero.', string(ME.message), charTargetName);
            % If not available, set to zero
            strDynParams.strBody3rdData(idB+1).dGM = 0.0;
            strDynParams.strBody3rdData(idB+1).dRefRadius = 0.0;
        end
    end

end


%% Solar Radiation Pressure data
% ACHTUNG: Make sure that unit of measure for distance matches.
dDistFromSunAU = mean(vecnorm(strMainBodyRefData.dSunPosition_IN, 2, 1), "all") / (150e6 * dUnitsScaling); % Input distance in [m]
strDynParams.strSRPdata.dP_SRP0 = 1367/(299792.458 * dUnitsScaling) * (1/(dDistFromSunAU)^2); % [N/m^2]
fprintf('\nAverage distance from the SUN in AU: %3.4f AU\n', dDistFromSunAU);

% Add Sun gravity parameter
try
    strDynParams.strBody3rdData(1).dGM = cspice_bodvrd('SUN', 'GM', 1) * (dUnitsScaling^3); 
catch ME
    warning('ERROR occurred while fetching Sun GM: %s. Setting to default value in m^3/s^2.', string(ME.message));
    % If not available, set to default value
    strDynParams.strBody3rdData(1).dGM = (dUnitsScaling^3) * 1.32712440041279419 * 1E11; % [km^3/s^2] or [m^3/s^2] - JPL DE440
end

%% Spacecraft data
dDefaultReflCoeff = 1.29;  % Global CR
dDefaultSCmass    = 14.8; % 12; % [kg]
dDefaultA_SRP     = 0.5329E-6 * (dUnitsScaling^2); % [m^2]

if isempty(strDynParams.strSCdata)
    warning('No spacecraft data provided in strDynParams.strSCdata. Default hardcoded values (RCS-1) will be used.')    
    strDynParams.strSCdata.dReflCoeff = dDefaultReflCoeff;
    strDynParams.strSCdata.dSCmass    = dDefaultSCmass;
    strDynParams.strSCdata.dA_SRP     = dDefaultA_SRP;

else
    % Check for missing values and set defaults where needed
    if not(isfield(strDynParams.strSCdata, "dReflCoeff"))
        warning('No spacecraft reflectivity coefficient provided in strDynParams.strSCdata.dReflCoeff. Default value of 1.29 will be used.')
        strDynParams.strSCdata.dReflCoeff = dDefaultReflCoeff;
    end
    if not(isfield(strDynParams.strSCdata, "dSCmass"))
        warning('No spacecraft mass provided in strDynParams.strSCdata.dSCmass. Default value of 14.8 kg will be used.')
        strDynParams.strSCdata.dSCmass = dDefaultSCmass;
    end
    if not(isfield(strDynParams.strSCdata, "dA_SRP"))
        warning('No spacecraft area provided in strDynParams.strSCdata.dA_SRP. Default value of 0.5329 m^2 will be used.')
        strDynParams.strSCdata.dA_SRP = dDefaultA_SRP;
    end
end


end




