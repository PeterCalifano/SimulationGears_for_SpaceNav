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
    kwargs.strDynParams                     (1,1) struct = struct()    % Initialization value
    kwargs.str3rdBodyRefData                (1,1) struct = struct()    % Initialization value
    kwargs.bAddNonSphericalGravityCoeffs    (1,1) logical = false;
    kwargs.objDataset                       (1,1) {mustBeA(kwargs.objDataset, "SReferenceMissionDesign")} = SReferenceMissionDesign()
    kwargs.charSpherHarmCoeffInputFileName  (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
    kwargs.ui16MaxSHdegree                  (1,1) uint16 = uint16(0)
    kwargs.cellAdditionalBodiesNames        (1,:) string {mustBeA(kwargs.cellAdditionalBodiesNames, ["string", "char"])} = string.empty(0, 1)
    kwargs.bAdd3rdBodiesAttitude            (1,1) logical = true; % If true, attitude data will be added to str3rdBodyRefData
    kwargs.bUseKilometersScale              (1,1) logical = false;
    kwargs.charSCpanelObjFilePath           (1,:) string {mustBeA(kwargs.charSCpanelObjFilePath, ["string", "char"])} = ""
    kwargs.bBuildSCpanelSRPdata             (1,1) logical = false
    kwargs.charSCpanelObjInputUnit          (1,:) string {mustBeA(kwargs.charSCpanelObjInputUnit, ["string", "char"]), mustBeMember(kwargs.charSCpanelObjInputUnit, ["m", "km"])} = "m"
    kwargs.charSCpanelDataUnit              (1,:) string {mustBeA(kwargs.charSCpanelDataUnit, ["string", "char"]), mustBeMember(kwargs.charSCpanelDataUnit, ["m", "km"])} = "m"
    kwargs.dSCpanelDiffuseCoeff             (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0
    kwargs.dSCpanelSpecularCoeff            (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0
    kwargs.dSCpanelMeshSimplifyFactor       (1,1) double {mustBeFinite} = 1.0
    kwargs.charPolyhedronGravityObjFilePath (1,:) string {mustBeA(kwargs.charPolyhedronGravityObjFilePath, ["string", "char"])} = ""
    kwargs.bBuildPolyhedronGravityData      (1,1) logical = false
    kwargs.charPolyhedronGravityInputUnit   (1,:) string {mustBeA(kwargs.charPolyhedronGravityInputUnit, ["string", "char"]), mustBeMember(kwargs.charPolyhedronGravityInputUnit, ["m", "km"])} = "m"
    kwargs.charPolyhedronGravityTargetUnit  (1,:) string {mustBeA(kwargs.charPolyhedronGravityTargetUnit, ["string", "char"]), mustBeMember(kwargs.charPolyhedronGravityTargetUnit, ["", "m", "km"])} = ""
    kwargs.dPolyhedronDensity               (1,1) double = NaN
    kwargs.dPolyhedronGravConst             (1,1) double = NaN
    kwargs.dPolyhedronMeshSimplifyFactor    (1,1) double {mustBeFinite} = 1.0
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
% dEphemeridesTimegrid  (1,:) double
% enumScenarioName    EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])} = EnumScenarioName.Itokawa
% charInertialFrame   (1,:) char {mustBeA(charInertialFrame, ["string", "char"])} = "J2000"
% kwargs.strDynParams                     (1,1) struct = struct()         % Initialization value
% kwargs.str3rdBodyRefData                (1,1) struct = struct()    % Initialization value
% kwargs.bAddNonSphericalGravityCoeffs    (1,1) logical = false;
% kwargs.objDataset                       (1,1) {mustBeA(kwargs.objDataset, "SReferenceMissionDesign")} = SReferenceMissionDesign()
% kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
% kwargs.cellAdditionalBodiesNames       (1,:) string {mustBeA(kwargs.cellAdditionalBodiesNames, ["string", "char"])} = string.empty(0, 1)
% kwargs.bAdd3rdBodiesAttitude           (1,1) logical = true; % If true, attitude data will be added to str3rdBodyRefData
% kwargs.bUseKilometersScale             (1,1) logical = false;
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strDynParams
% strAdditionalData
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-02-2025    Pietro Califano     First version copy-pasting previous implementation
% 14-03-2025    Pietro Califano     Move code to CScenarioGenerator static method for standardization
% 21-07-2025    Pietro Califano     Add support for 3rd body reference data and generalize implementation
% -------------------------------------------------------------------------------------------------------------
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
                                                                                "ui16MaxSHdegree", kwargs.ui16MaxSHdegree, ...
                                                                                "bUseKilometersScale", kwargs.bUseKilometersScale);


if kwargs.bUseKilometersScale
    dUnitsScaling = 1.0;
    charDynamicsLengthUnit = "km";
else
    dUnitsScaling = 1E3;
    charDynamicsLengthUnit = "m";
end

if kwargs.objDataset.bDefaultConstructed % Try to use SPICE kernels

    if max(abs(dEphemeridesTimegrid)) < 365 * 86400
        warning(['Minimum (absolute) time in ephemeris timegrid < 365 * 86400 [s]. ' ...
            'This is being used to query CSPICE but seems too small. Make sure it is as intended!'])
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

    for idB = 1:ui32NumOfAdditionalBodies
        charAdditionalBodyName = char(kwargs.cellAdditionalBodiesNames(idB));

        % Try to load 3rd body positions from SPICE
        try
            str3rdBodyRefData(idB).strOrbitData.dPosition_W = dUnitsScaling * cspice_spkpos( ...
                upper(charAdditionalBodyName), dEphemeridesTimegrid, charInertialFrame, 'none', charTargetName);
        catch ME
            warning('DefineEnvironmentProperties:ThirdBodyPositionUnavailable', ...
                'Failed to fetch %s ephemerides from SPICE: %s', charAdditionalBodyName, string(ME.message));
            str3rdBodyRefData(idB).strOrbitData.dPosition_W = zeros(3, numel(dEphemeridesTimegrid));
        end

        % Load 3rd body dynamics parameters from CScenarioGenerator database if available
        try 
            [~, ~, strDynParams_3rdBody] = CScenarioGenerator.LoadDefaultScenarioData(charAdditionalBodyName, ...
                struct(), "bUseKilometersScale", kwargs.bUseKilometersScale);
            
            strDynParams.strBody3rdData(idB+1).dGM = strDynParams_3rdBody.strMainData.dGM;
            strDynParams.strBody3rdData(idB+1).dRefRadius = strDynParams_3rdBody.strMainData.dRefRadius;

        catch ME
            warning('DefineEnvironmentProperties:ThirdBodyDefaultsUnavailable', ...
                'Failed to load %s default dynamics data: %s. GM/radius set to zero.', ...
                charAdditionalBodyName, string(ME.message));

            strDynParams.strBody3rdData(idB+1).dGM = 0.0;
            strDynParams.strBody3rdData(idB+1).dRefRadius = 0.0;
        end

        if bAdd3rdBodiesAttitude
            % Try to load 3rd body attitude from SPICE
            try
                [~, charAdditionalBodyFixedFrame] = CScenarioGenerator.LoadDefaultScenarioData(charAdditionalBodyName, ...
                    struct(), "bUseKilometersScale", kwargs.bUseKilometersScale);

                str3rdBodyRefData(idB).strAttData.dDCM_WfromTB = cspice_pxform(char(charAdditionalBodyFixedFrame), ...
                    char(charInertialFrame), dEphemeridesTimegrid);
            catch ME
                warning('DefineEnvironmentProperties:ThirdBodyAttitudeUnavailable', ...
                    'Failed to fetch %s attitude from SPICE: %s', charAdditionalBodyName, string(ME.message));
            end
        end
    end

else
    % Load ephemeris data from dataset
    % assert(length(dEphemeridesTimegrid) == length(kwargs.objDataset.dTimestamps), ...
    %         "ERROR: objDataset timestamps do not match specified dEphemerisTimegrid.")

    % Build extraction index grid (dataset indices for each ephemeris time)
    dEphTimegrid  = dEphemeridesTimegrid(:);

    assert(all(diff(kwargs.objDataset.dTimestamps(:)) > 0), 'objDataset.dTimestamps must be strictly increasing.');
    assert(all(diff(dEphTimegrid)  >= 0), 'dEphemeridesTimegrid must be non-decreasing.');

    ui32NumEntriesInDataset = numel(kwargs.objDataset.dTimestamps(:));
    ui32EphemeridesExtractIdx = interp1(kwargs.objDataset.dTimestamps(:), 1:ui32NumEntriesInDataset, dEphTimegrid, 'nearest', 'extrap'); % Nearest index for each ephemeris time
    ui32EphemeridesExtractIdx = uint32(max(1, min(ui32NumEntriesInDataset, round(ui32EphemeridesExtractIdx)))); % Clamp to [1, N]

    strMainBodyRefData.dDCM_INfromTB    = pagetranspose(kwargs.objDataset.dDCM_TBfromW(:,:,ui32EphemeridesExtractIdx));
    strMainBodyRefData.dSunPosition_IN  = kwargs.objDataset.dSunPosition_W(:,ui32EphemeridesExtractIdx);

    % Get additional bodies data if provided (Sun not included)
    ui32NumOfAdditionalBodies = length(kwargs.objDataset.cellAdditionalBodiesTags);
    bAdd3rdBodiesAttitude = not(isempty(kwargs.objDataset.cellAdditionalBodiesDCM_TBfromW)) && kwargs.bAdd3rdBodiesAttitude;

    for idB = 1:ui32NumOfAdditionalBodies

        if not(isempty(kwargs.objDataset.cellAdditionalBodiesPos_W{idB}))
            % Store data in struct for ephemerides factory
            str3rdBodyRefData(idB).strOrbitData.dPosition_W  = kwargs.objDataset.cellAdditionalBodiesPos_W{idB}(:,ui32EphemeridesExtractIdx);
        end
        
        if bAdd3rdBodiesAttitude && not(isempty(kwargs.objDataset.cellAdditionalBodiesDCM_TBfromW{idB}))
            str3rdBodyRefData(idB).strAttData.dDCM_WfromTB = pagetranspose(kwargs.objDataset.cellAdditionalBodiesDCM_TBfromW{idB}(:,:,ui32EphemeridesExtractIdx));
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
dSRPreferenceDistance = 1.495978707E8 * dUnitsScaling;
dDistFromSunAU = mean(vecnorm(strMainBodyRefData.dSunPosition_IN, 2, 1), "all") / dSRPreferenceDistance;

% Initialize SRP data struct using average distance from the Sun to compute reference pressure value
strDynParams.strSRPdata.dP_SRP0 = 1367 / (299792.458 * dUnitsScaling); % SRP Pressure value at 1 AU
strDynParams.strSRPdata.dP_SRP = strDynParams.strSRPdata.dP_SRP0 * (1/(dDistFromSunAU)^2); % [N/m^2]
strDynParams.strSRPdata.dReferenceDistance = dSRPreferenceDistance;

if not(isfield(strDynParams.strSRPdata, "bRecomputePressureFromDistance"))
    strDynParams.strSRPdata.bRecomputePressureFromDistance = false;
end
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
dDefaultSCmass    = 12.5; %14.8; % 12; % [kg]
dDefaultA_SRP     = 0.5329E-6 * (dUnitsScaling^2); % [m^2]

if isempty(strDynParams.strSCdata)
    warning('No spacecraft data provided in strDynParams.strSCdata. Default hardcoded values (RCS-1) will be used.')    
    strDynParams.strSCdata.dReflCoeff = dDefaultReflCoeff;
    strDynParams.strSCdata.dSCmass    = dDefaultSCmass;
    strDynParams.strSCdata.dA_SRP     = dDefaultA_SRP;

else
    % Check for missing values and set defaults where needed
    if not(isfield(strDynParams.strSCdata, "dReflCoeff"))
        warning('No spacecraft reflectivity coefficient provided in strDynParams.strSCdata.dReflCoeff. Default value of %4.4g will be used.', dDefaultReflCoeff)
        strDynParams.strSCdata.dReflCoeff = dDefaultReflCoeff;
    end
    if not(isfield(strDynParams.strSCdata, "dSCmass"))
        warning('No spacecraft mass provided in strDynParams.strSCdata.dSCmass. Default value of %4.4g kg will be used.', dDefaultSCmass)
        strDynParams.strSCdata.dSCmass = dDefaultSCmass;
    end
    if not(isfield(strDynParams.strSCdata, "dA_SRP"))
        warning('No spacecraft area provided in strDynParams.strSCdata.dA_SRP. Default value of %4.4g m^2 will be used.', dDefaultA_SRP)
        strDynParams.strSCdata.dA_SRP = dDefaultA_SRP;
    end
end

% Add spacecraft panel SRP data if requested
if kwargs.bBuildSCpanelSRPdata || strlength(kwargs.charSCpanelObjFilePath) > 0

    assert(strlength(kwargs.charSCpanelObjFilePath) > 0, ...
        'DefineEnvironmentProperties:MissingSCpanelObjFilePath', ...
        'charSCpanelObjFilePath must be provided when building spacecraft panel SRP data.');
   
    % Build SRP panel data struct from input OBJ file and add to strDynParams
    strDynParams.strSCdata.strSRPpanelData = BuildQuadsModelSRPDataFromObj( ...
                                        kwargs.charSCpanelObjFilePath, ...
                                        charInputUnit=kwargs.charSCpanelObjInputUnit, ...
                                        charTargetUnitOutput=kwargs.charSCpanelDataUnit, ...
                                        dDiffuseCoeff=kwargs.dSCpanelDiffuseCoeff, ...
                                        dSpecularCoeff=kwargs.dSCpanelSpecularCoeff, ...
                                        dMeshSimplifyFactor=kwargs.dSCpanelMeshSimplifyFactor);
end

if kwargs.bBuildPolyhedronGravityData || strlength(kwargs.charPolyhedronGravityObjFilePath) > 0

    assert(strlength(kwargs.charPolyhedronGravityObjFilePath) > 0, ...
        'DefineEnvironmentProperties:MissingPolyhedronObjFilePath', ...
        'charPolyhedronGravityObjFilePath must be provided when building polyhedron gravity data.');

    charPolyhedronTargetUnit = kwargs.charPolyhedronGravityTargetUnit;
    if charPolyhedronTargetUnit == ""
        charPolyhedronTargetUnit = charDynamicsLengthUnit;
    end

    % TODO extend to accept CShapeModel directly as input to avoid redundant loading
    % Load polyhedron shape model from input OBJ file
    objPolyhedronShapeModel = CShapeModel("file_obj", kwargs.charPolyhedronGravityObjFilePath, ...
                                kwargs.charPolyhedronGravityInputUnit, charPolyhedronTargetUnit, true, "", true, ...
                                dMeshSimplifyFactor=kwargs.dPolyhedronMeshSimplifyFactor);

    % Build polyhedron gravity data (face/edge dyadics and vertex indices)
    objPolyhedronShapeModel = objPolyhedronShapeModel.BuildPolyhedronGravityData();

    [ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, ui32FacesRows, dVerticesRows] = ...
        objPolyhedronShapeModel.getPolyhedronGravityData();
    dPolyhedronGravConst = kwargs.dPolyhedronGravConst;

    if ~isfinite(dPolyhedronGravConst)
        if strcmpi(charPolyhedronTargetUnit, "km")
            dPolyhedronGravConst = 6.67430e-20;
        else
            dPolyhedronGravConst = 6.67430e-11;
        end
    end

    % Compute polyhedron volume and CoM for gravity computations
    [dPolyhedronVolume, dPolyhedronCoM] = ComputeMeshModelVolumeAndCoM(ui32FacesRows, dVerticesRows);
    dPolyhedronDensity = kwargs.dPolyhedronDensity;

    if ~isfinite(dPolyhedronDensity)
        assert(isfield(strDynParams, "strMainData") && isfield(strDynParams.strMainData, "dGM") && ...
            isfinite(strDynParams.strMainData.dGM) && strDynParams.strMainData.dGM > 0.0, ...
            'DefineEnvironmentProperties:MissingPolyhedronDensity', ...
            'Either dPolyhedronDensity or strDynParams.strMainData.dGM must be available to build polyhedron gravity data.');
        dPolyhedronDensity = strDynParams.strMainData.dGM / (dPolyhedronGravConst * dPolyhedronVolume);
    end

    % Pack gravity data into struct for output
    strDynParams.strMainData.strPolyhedronGravityData = struct();
    strDynParams.strMainData.strPolyhedronGravityData.ui32FaceVertexIds = ui32FacesRows;
    strDynParams.strMainData.strPolyhedronGravityData.dVerticesPos = dVerticesRows;
    strDynParams.strMainData.strPolyhedronGravityData.dDensity = dPolyhedronDensity;
    strDynParams.strMainData.strPolyhedronGravityData.ui32EdgeVertexIds = ui32EdgeVertexIds;
    strDynParams.strMainData.strPolyhedronGravityData.dEdgeDyadics = dEdgeDyadics;
    strDynParams.strMainData.strPolyhedronGravityData.dFaceDyadics = dFaceDyadics;
    strDynParams.strMainData.strPolyhedronGravityData.dGravConst = dPolyhedronGravConst;
    strDynParams.strMainData.strPolyhedronGravityData.dGravParam = dPolyhedronGravConst * dPolyhedronDensity * dPolyhedronVolume;
    strDynParams.strMainData.strPolyhedronGravityData.dVolume = dPolyhedronVolume;
    strDynParams.strMainData.strPolyhedronGravityData.dCoM = dPolyhedronCoM;
    strDynParams.strMainData.strPolyhedronGravityData.charLengthUnit = char(charPolyhedronTargetUnit);
    strDynParams.strMainData.strPolyhedronGravityData.charSourceObjFilePath = char(kwargs.charPolyhedronGravityObjFilePath);
end

end
