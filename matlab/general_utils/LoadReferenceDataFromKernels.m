function [objReferenceMissionData, dStateSC_TargetFixed, dSunPosition_TargetFixed] = LoadReferenceDataFromKernels(varTargetID, ...
                                                                                                                   enumTrajectKernelName, ...
                                                                                                                   dTimegridVect, ...
                                                                                                                   enumWorldFrame, ...
                                                                                                                   varReferenceCentre, ...
                                                                                                                   enumTargetFrame, ...
                                                                                                                   kwargs)
arguments (Input)
    varTargetID             {mustBeA(varTargetID, ["string", "char", "double", "int32", "uint32", "single"])} 
    enumTrajectKernelName   (1,:) {mustBeA(enumTrajectKernelName, ["string", "char", "EnumTrajectoryNames", "EnumTrajectKernelName"])}
    dTimegridVect           (1,:) double {ismatrix, mustBeNumeric}
    enumWorldFrame          (1,1) {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])}  % Enumeration class indicating the W frame in which the data are expressed
    varReferenceCentre      (1,1) {mustBeA(varReferenceCentre, ["SEnumFrameName", "string", "char", "double", "int32", "uint32"])}
    enumTargetFrame         (1,1) {mustBeA(enumTargetFrame, ["SEnumFrameName", "string", "char"])} = enumWorldFrame
end
arguments (Input)
    kwargs.charTrajKernelFolderPath (1,:) char {ischar, isstring, mustBeFolder} = '.'
    kwargs.dDeltaTimeStep           (1,1) double {isscalar, mustBeNumeric} = 60.0
    kwargs.dEphTimes0toFinal        (2,1) double {isvector, mustBeNumeric} = [0;0]
    kwargs.bLoadManoeuvres          {islogical, isscalar} = true;
    kwargs.charKernelLengthUnits    (1,:) char {ischar, isstring, mustBeMember(kwargs.charKernelLengthUnits, ["km", "m"])} = "km";
    kwargs.charOutputLengthUnits    (1,:) char {ischar, isstring, mustBeMember(kwargs.charOutputLengthUnits, ["km", "m"])} = "km";
    kwargs.varTargetBodyID          {mustBeA(kwargs.varTargetBodyID, ["string", "char", "double", "int32", "uint32", "single"])} = varReferenceCentre % Defaults to reference centre in most cases
    kwargs.AdditionalTargetsID      {iscell} = {};
    kwargs.AdditionalTargetNames    {iscell} = {};
end
arguments (Output)
    objReferenceMissionData     (1,1) SReferenceImagesDataset
    dStateSC_TargetFixed        (6,:) double {ismatrix, mustBeNumeric} 
    dSunPosition_TargetFixed    (3,:) double {ismatrix, mustBeNumeric} 
end
%% SIGNATURE
% [objReferenceMissionData, dStateSC_TargetFixed, dSunPosition_TargetFixed] = LoadReferenceDataFromKernels(varTargetID, ...
%                                                                                                           enumTrajectKernelName, ...
%                                                                                                           dTimegridVect, ...
%                                                                                                           enumWorldFrame, ...
%                                                                                                           varReferenceCentre, ...
%                                                                                                           enumTargetFrame, ...
%                                                                                                           kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to retrieve all the relevant data to build "mission design" datasets from SPK (.bsp) SPICE
% kernels. The output dataset object contains orbital state of the specified objects in the desired
% reference frame, the position of a target (e.g. an asteroid) with respect to the same frame, as well as
% Sun and Earth positions. Data are packaged in the container object "SReferenceImagesDataset" inheriting
% from "SReferenceMissionData". Spacecraft attitude is not retrieved or computed in the current version.
% Impulsive manoeuvres can be retrieved if the SPK is split in different intervals, retrieved using spkcov.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments (Input)
%     varTargetID             {mustBeA(varTargetID, ["string", "char", "double", "int32", "uint32", "single"])} 
%     enumTrajectKernelName   (1,:) {mustBeA(enumTrajectKernelName, ["string", "char", "EnumTrajectoryNames", "EnumTrajectKernelName"])}
%     dTimegridVect           (1,:) double {ismatrix, mustBeNumeric}
%     enumWorldFrame          (1,1) {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])}  % Enumeration class indicating the W frame in which the data are expressed
%     varReferenceCentre      (1,1) {mustBeA(varReferenceCentre, ["SEnumFrameName", "string", "char", "double", "int32", "uint32"])}
%     enumTargetFrame         (1,1) {mustBeA(enumTargetFrame, ["SEnumFrameName", "string", "char"])} = enumWorldFrame
% end
% arguments (Input)
%     kwargs.charTrajKernelFolderPath (1,:) char {ischar, isstring, mustBeFolder} = '.'
%     kwargs.dDeltaTimeStep           (1,1) double {isscalar, mustBeNumeric} = 60.0
%     kwargs.dEphTimes0toFinal        (2,1) double {isvector, mustBeNumeric} = [0;0]
%     kwargs.bLoadManoeuvres          {islogical, isscalar} = true;
%     kwargs.charKernelLengthUnits    (1,:) char {ischar, isstring, mustBeMember(kwargs.charKernelLengthUnits, ["km", "m"])} = "km";
%     kwargs.charOutputLengthUnits    (1,:) char {ischar, isstring, mustBeMember(kwargs.charOutputLengthUnits, ["km", "m"])} = "km";
%     kwargs.varTargetBodyID          {mustBeA(kwargs.varTargetBodyID, ["string", "char", "double", "int32", "uint32", "single"])} = varReferenceCentre % Defaults to reference centre in most cases
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% arguments (Output)
%     objReferenceMissionData     (1,1) SReferenceImagesDataset
%     dStateSC_TargetFixed        (6,:) double {ismatrix, mustBeNumeric} 
%     dSunPosition_TargetFixed    (3,:) double {ismatrix, mustBeNumeric} 
% end
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 25-06-2025        Pietro Califano     Derived from LoadReferenceRCS1 and generalized for any SPK kernel
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SReferenceImagesDataset; SReferenceMissionDesign
% GetManoeuvresFromSPK()
% -------------------------------------------------------------------------------------------------------------
%% Function code
fprintf("\nFetching data to build reference mission plan from kernels...\n")

% Validate target ID
if isnumeric(varTargetID)
    varTargetID = int32(varTargetID);
else
    try
        varTargetID = char(varTargetID);
    catch
        error('Invalid input datatype for varTargetID: must be an integer numeric value or a string/char. Got: %s.', class(varTargetID))
    end
end

% Validate/handle name of kernel path
% if iobject(enumTrajectKernelName)
%     % Input is an enumeration class
%     enumTrajectKernelName = char(enumTrajectKernelName);
% else
%     % Input is file name
%     enumTrajectKernelName = char(enumTrajectKernelName);
% end
enumTrajectKernelName = char(enumTrajectKernelName);
charKernelFilePath = char( fullfile(charTrajKernelFolderPath, sprintf("%s.bsp", enumTrajectKernelName)) );

if not(isfile(charKernelFilePath))
    error('SPK Kernel file not found at %s.', charKernelFilePath);
end

% Determine ETO, ETf for data fetching
if all(kwargs.dEphTimes0toFinal == [0;0], 'all')

    fprintf("No initial and final ephemeris time specified. Using default values retrieve from cspice_spkcov.\n")
    [dET0f] = cspice_spkcov(charKernelFilePath, varTargetID, 1e5) ;

    dET0 = dET0f(1);
    dETf = dET0f(end);

else
    % Get ephemeris times bounds from input
    dET0 = kwargs.dEphTimes0toFinal(1);
    dETf = kwargs.dEphTimes0toFinal(2);
end

fprintf("Initial and final ephemeris time bounds: [%10.1f, %10.1f].\n", dET0, dETf);

% Load manoeuvres
bHasManoeuvres = false;
if kwargs.bLoadManoeuvres == true

    [dTargetCover, dManVectors, dManMagnitudes, dStatePreMan, ...
        dPrePostManTime, dStatePostMan, ui32NumIntervals] = GetManoeuvresFromSPK(varTargetID, ...
                                                                        "charKernelFolderPath", charTrajKernelFolderPath, ...
                                                                        "charKernelNameWithoutExt", enumTrajectKernelName, ...
                                                                        "charKernelLengthUnits", kwargs.charKernelLengthUnits, ...
                                                                        "charOutputLengthUnits", kwargs.charOutputLengthUnits, ...
                                                                        "varRefCentre", varReferenceCentre, ...
                                                                        "charEnumWorldFrame", char(enumWorldFrame)); %#ok<ASGLU>

    if not(isempty(dManVectors)) && not(isempty(dManMagnitudes)) && not(isempty(dPrePostManTime))
        bHasManoeuvres = true;
    end
end

% Determine if timegrid is relative or absolute
bIsInputTimegridRelative = true; %#ok<*NASGU> % Default assumption
charGridType = "relative";

if not(isempty(dTimegridVect))

    % Check initial time instant
    dFirstTime = dTimegridVect(1);
    assert(dFirstTime >= 0, sprintf("ERROR: input timegrid must only contain non-negative real numbers! Found initial timestamp: %6g\n", dFirstTime));

    if abs(dFirstTime - dET0) < 100

        bIsInputTimegridRelative = false;
        charGridType = "absolute";
        dAbsTimegrid = dTimegridVect;
        dTimegridVect = dTimegridVect - dET0; % Re-write timegrid as relative;
    else
        % Convert timegrid to Ephemeris Time
        dAbsTimegrid = dTimegridVect + dET0;
    end

else
    % Build timegrid if not provided
    fprintf("No timegrid was specified as input. Building it to cover entire Ephemeris Time interval with timestep: %4.0f [s]\n", kwargs.dDeltaTimeStep)
    dTimegridVect = 0:kwargs.dDeltaTimeStep:(dETf - dET0); % Define relative timegrid first
    dAbsTimegrid = dTimegridVect + dET0;
end

% Print monitoring information
fprintf('Using %s timegrid with relative bounds from ET0: [%8.0f, %8.0f].\nData spans a total of %s.\n', ...
    upper(charGridType), dTimegridVect(1), dTimegridVect(end), formatElapsedTime(dTimegridVect(end) - dTimegridVect(1)));

% Check that last entry of timegrid is within allowed bounds
assert(dAbsTimegrid(end) <= dETf, "ERROR: last time instant specified in absolute timegrid is outside allowed ET bounds! %6.2g < %6.2g\n", dETf, dTimegridVect(end));

%% Load kernels
try
    cspice_furnsh(char(charKernelFilePath))
catch ME
    error('cspice_furnsh routine failed in loading kernel file with error:\n %s', string(ME.message))
end

%% Fetch data
try
    % Sun position in World frame from reference centre
    dSunPosition_WorldFrame = cspice_spkpos('SUN', dAbsTimegrid, char(enumWorldFrame), 'NONE', varReferenceCentre);
catch ME
    warning('Attempt to retrieve Sun position failed with error: %s', string(ME.message) );
end

try
    % Earth position in World frame from reference centre
    dEarthPosition_WorldFrame = - cspice_spkpos('EARTH', dAbsTimegrid, char(enumWorldFrame), 'NONE', varReferenceCentre);
catch ME
    warning('Attempt to retrieve Earth position failed with error: %s', string(ME.message) );
end

% Spacecraft state in World Frame from reference centre
dStateSC_WorldFrame = cspice_spkezr(varTargetID, dAbsTimegrid, char(enumWorldFrame), 'NONE', varReferenceCentre);

try
    % Target state in World Frame
    % TODO: does spice define frames with a specific origin or only as attitude?
    dTargetPosition_WorldFrame = cspice_spkpos(varTargetBodyID, dAbsTimegrid, char(enumWorldFrame), 'NONE', varReferenceCentre); % TODO which observer?? TBC
catch ME
    warning('Attempt to retrieve target body position failed with error: %s', string(ME.message) );
end

try
    % Target attitude in World Frame
    dTargetDCM_TBfromWorld = cspice_pxform(char(enumWorldFrame), char(enumTargetFrame), dAbsTimegrid);
catch
    warning('Attempt to retrieve target body attitude failed with error: %s', string(ME.message) );
end

%% Process additional targets if available in loaded kernel sets
% kwargs.varTargetBodyID          {mustBeA(kwargs.varTargetBodyID, ["string", "char", "double", "int32", "uint32", "single"])} = varReferenceCentre % Defaults to reference centre in most cases
% kwargs.AdditionalTargetsID      {iscell} = {};
% kwargs.AdditionalTargetNames    {iscell} = {};
assert(isempty(kwargs.AdditionalTargetNames) || length(kwargs.AdditionalTargetsID) == length(kwargs.AdditionalTargetNames), ...
    'ERROR: AdditionalTargetNames and AdditionalTargetsID must have the same length if any name has to be specified. Else leave AdditionalTargetNames as empty.');

if ~isempty(kwargs.AdditionalTargetsID)

    ui32NumAdditionalBodies = uint32(length(kwargs.AdditionalTargetsID));
    cellTargetTags = cell(1, ui32NumAdditionalBodies);
    cellTargetPos_W = cell(1, ui32NumAdditionalBodies);

    ui32AllocIdx = uint32(1);
    ui32CounterAddBodies = 0;
    for idB = 1:ui32NumAdditionalBodies

        % Get target ID
        varTargetBodyID_ = kwargs.AdditionalTargetsID{idB};

        try
            dTmpTargetPosition_WorldFrame = cspice_spkpos(varTargetBodyID_, dAbsTimegrid, char(enumWorldFrame), 'NONE', varReferenceCentre); 

            if not(isempty(kwargs.AdditionalTargetNames))
                charTmpTargetName = string(kwargs.AdditionalTargetNames(idB));
            else
                if isnumeric(varTargetBodyID_)
                    charTmpTargetName = num2str(varTargetBodyID_);
                else
                    charTmpTargetName = string(varTargetBodyID_);
                end
            end

            % Store position data into cell
            cellTargetPos_W{idB} = dTmpTargetPosition_WorldFrame;

            % Add name tags to cell array
            cellTargetTags{idB} = charTmpTargetName;

        catch
            warning('Attempt to retrieve target body attitude failed with error: %s', string(ME.message) );
            continue;
        end
        ui32AllocIdx = ui32AllocIdx + 3;
    end

end


% Instantiate output datastruct
% self = SReferenceMissionDesign(enumWorldFrame, ...
%                                dTimestamps, ...
%                                dStateSC_W, ...
%                                dDCM_TBfromW, ...
%                                dTargetPosition_W, ...
%                                dEarthPosition_W, ...
%                                optional)

objReferenceMissionData = SReferenceImagesDataset(CCameraIntrinsics(), ...
                                                  enumWorldFrame, ...
                                                  dAbsTimegrid, ...
                                                  dStateSC_WorldFrame, ...
                                                  dTargetDCM_TBfromWorld, ...
                                                  dTargetPosition_WorldFrame, ...
                                                  dSunPosition_WorldFrame, ...
                                                  dEarthPosition_WorldFrame, ...
                                                  "dRelativeTimestamps", dTimegridVect, ...
                                                  "charLengthUnits", kwargs.charOutputLengthUnits);

if bHasManoeuvres
    % Assign manoeuvres data
    objReferenceMissionData.dManoeuvresStartTimestamps  = dPrePostManTime(:,1); % Pre-manoeuvre timestamp in ET
    objReferenceMissionData.dManoeuvresDeltaV_SC        = dManVectors; % Actually in World Frame
end

if ui32CounterAddBodies > 0

    % Add additional bodies cells
    objReferenceMissionData.cellAdditionalBodiesPos_W   = cellTargetPos_W(1:ui32CounterAddBodies);
    objReferenceMissionData.cellAdditionalBodiesTags    = cellTargetTags(1:ui32CounterAddBodies);
   
end


fprintf("\nFetching and packaging completed.\n")
if nargout == 1
    return;
end
%% Additional data (if required)
% State of spacecraft in Target Frame
dStateSC_TargetFixed = cspice_spkezr(varTargetID, dAbsTimegrid, char(enumTargetFrame), 'NONE', varReferenceCentre);

if nargout == 2
    return;
end

% Apophis position to Sun in target frame
dSunPosition_TargetFixed = cspice_spkpos('SUN', dAbsTimegrid, char(enumTargetFrame), 'NONE', varReferenceCentre);

if nargout == 3
    return;
end

end

% Local function to get elapsed time as string
function charElapsedTime = formatElapsedTime(dSeconds)

dDays = floor(dSeconds / (24 * 3600)); % Convert to days
dHours = floor(dSeconds / 3600);       % Convert to hours
dMinutes = floor(dSeconds / 60);       % Convert to minutes

if dDays >= 1
    charElapsedTime = sprintf('%d d, %02d h, %02d min, %02d sec', ...
        dDays, mod(dHours, 24), mod(dMinutes, 60), mod(dSeconds, 60));

elseif dHours >= 1
    charElapsedTime = sprintf('%02d h, %02d min, %02d sec', ...
        dHours, mod(dMinutes, 60), mod(dSeconds, 60));

elseif dMinutes >= 1
    charElapsedTime = sprintf('%02d min, %02d sec', ...
        dMinutes, mod(dSeconds, 60));
else
    charElapsedTime = sprintf('%02d sec', floor(dSeconds));
end
end
