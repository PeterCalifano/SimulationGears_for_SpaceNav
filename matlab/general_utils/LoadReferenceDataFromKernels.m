function [objReferenceMissionData, dStateSC_TargetFixed, dSunPosition_TargetFixed] = LoadReferenceDataFromKernels(varTargetID, ...
                                                                                                                   enumTrajectKernelName, ...
                                                                                                                   dTimegridVect_kernelTimeScale, ...
                                                                                                                   enumWorldFrame, ...
                                                                                                                   varReferenceCentre, ...
                                                                                                                   enumTargetFrame, ...
                                                                                                                   kwargs)
arguments (Input)
    varTargetID             {mustBeA(varTargetID, ["string", "char", "double", "int32", "uint32", "single"])} 
    enumTrajectKernelName   (1,:) {mustBeA(enumTrajectKernelName, ["string", "char", "EnumTrajectoryNames", "EnumTrajectKernelName"])}
    dTimegridVect_kernelTimeScale           (1,:) double {ismatrix, mustBeNumeric}
    enumWorldFrame          (1,1) {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])}  % Enumeration class indicating the W frame in which the data are expressed
    varReferenceCentre      (1,1) {mustBeA(varReferenceCentre, ["SEnumFrameName", "string", "char", "double", "int32", "uint32"])}
    enumTargetFrame         (1,1) {mustBeA(enumTargetFrame, ["SEnumFrameName", "string", "char"])} = enumWorldFrame
end
arguments (Input)
    kwargs.bIsTimeGridRelative      (1,1) logical {mustBeScalarOrEmpty} = true
    kwargs.charTrajKernelFolderPath (1,:) char {ischar, isstring, mustBeFolder} = '.'
    kwargs.dDeltaTimeStep           (1,1) double {isscalar, mustBeNumeric} = 60.0
    kwargs.dEphTimes0toFinal        (2,1) double {isvector, mustBeNumeric} = [0;0]
    kwargs.bLoadManoeuvres          {islogical, isscalar} = true;
    kwargs.charKernelLengthUnits    (1,:) char {ischar, isstring, mustBeMember(kwargs.charKernelLengthUnits, ["km", "m"])} = "km";
    kwargs.charOutputLengthUnits    (1,:) char {ischar, isstring, mustBeMember(kwargs.charOutputLengthUnits, ["km", "m"])} = "km";
    kwargs.varTargetBodyID          {mustBeA(kwargs.varTargetBodyID, ["string", "char", "double", "int32", "uint32", "single"])} = varReferenceCentre % Defaults to reference centre in most cases
    kwargs.cellAdditionalTargetsID         {iscell} = {};
    kwargs.cellAdditionalTargetFrames      {iscell} = {};
    kwargs.cellAdditionalTargetNames       {iscell} = {};
    kwargs.bAdditionalBodiesRequireAttitude (1,:) logical {islogical} = false(0,0);
    kwargs.charKernelTimescale       (1,:) char {ischar, isstring, mustBeMember(kwargs.charKernelTimescale, ...
                                                        ["TAI", "TDB", "TDT", "TT", "ET", "JDTDB", "JDTDT", "JED", "GPS"])} = "ET";
    kwargs.charUserDefTimescale     (1,:) char {ischar, isstring, mustBeMember(kwargs.charUserDefTimescale, ...
                                                        ["TAI", "TDB", "TDT", "TT", "ET", "JDTDB", "JDTDT", "JED", "GPS"])} = "ET";
    kwargs.bUseKernelInitialTimestamp (1,1) logical {mustBeScalarOrEmpty} = false
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

% Convert timegrid if user defined time scale differ from kernel timescale and not relative timegrid
if not(strcmpi('ET', kwargs.charKernelTimescale)) && not(kwargs.bIsTimeGridRelative)
    dETtimegridVect = cspice_unitim(dTimegridVect_kernelTimeScale, kwargs.charKernelTimescale, 'ET');
else
    dETtimegridVect = dTimegridVect_kernelTimeScale;
end

% Validate target ID
[varTargetID, charTargetID] = ValidateID_(varTargetID);
[~, varReferenceCentre] = ValidateID_(varReferenceCentre);
ui32CounterAddBodies = length(kwargs.cellAdditionalTargetsID);

for idB = 1:ui32CounterAddBodies
    [~, kwargs.cellAdditionalTargetsID{idB}] = ValidateID_(kwargs.cellAdditionalTargetsID{idB});
end

kwargs.varTargetBodyID = ValidateID_(kwargs.varTargetBodyID);
% Validate/handle name of kernel path
% if iobject(enumTrajectKernelName)
%     % Input is an enumeration class
%     enumTrajectKernelName = char(enumTrajectKernelName);
% else
%     % Input is file name
%     enumTrajectKernelName = char(enumTrajectKernelName);
% end
enumTrajectKernelName = char(enumTrajectKernelName);
charKernelFilePath = char( fullfile(kwargs.charTrajKernelFolderPath, sprintf("%s.bsp", enumTrajectKernelName)) );

if not(isfile(charKernelFilePath))
    error('SPK Kernel file not found at %s.', charKernelFilePath);
end

% Determine ETO, ETf for data fetching
if all(kwargs.dEphTimes0toFinal == [0;0], 'all') && kwargs.bUseKernelInitialTimestamp

    fprintf("No initial and final ephemeris time specified. Using default values retrieved from cspice_spkcov.\n")
    [dTime0f_kernelTimeScale] = cspice_spkcov(charKernelFilePath, varTargetID, 1e5) ;
    
    % Get initial and final times of the SPK
    dET0f = cspice_unitim(dTime0f_kernelTimeScale', kwargs.charKernelTimescale, 'ET');

    dET0 = dET0f(1);
    dETf = dET0f(end);

elseif not(kwargs.bUseKernelInitialTimestamp)
    % Fix relative timegrid wrt initial time of the grid
    dET0 = dETtimegridVect(1);
    dETf = dETtimegridVect(end);
    
    dTime0f_kernelTimeScale = [dTimegridVect_kernelTimeScale(1), dTimegridVect_kernelTimeScale(end)];

else
    % Get ephemeris times bounds from input
    dET0 = kwargs.dEphTimes0toFinal(1);
    dETf = kwargs.dEphTimes0toFinal(2);
    
    dTime0f_kernelTimeScale = cspice_unitim(kwargs.dEphTimes0toFinal', 'ET', kwargs.charKernelTimescale);
end

% Assign initial and final times in kernel timescale
dTime0_kernelTimeScale = dTime0f_kernelTimeScale(1);
dTimef_kernelTimeScale = dTime0f_kernelTimeScale(end);

fprintf("Initial and final ephemeris time bounds: [%10.1f, %10.1f].\n", dET0, dETf);

% Load manoeuvres
bHasManoeuvres = false;
if kwargs.bLoadManoeuvres == true

    [dTargetCover, dManVectors, dManMagnitudes, dStatePreMan, ...
        dPrePostManTime, dStatePostMan, ui32NumIntervals] = GetManoeuvresFromSPK(varTargetID, ...
                                                                        "charKernelFolderPath", kwargs.charTrajKernelFolderPath, ...
                                                                        "charKernelNameWithoutExt", enumTrajectKernelName, ...
                                                                        "charKernelLengthUnits", kwargs.charKernelLengthUnits, ...
                                                                        "charOutputLengthUnits", kwargs.charOutputLengthUnits, ...
                                                                        "varRefCentre", varReferenceCentre, ...
                                                                        "charEnumWorldFrame", char(enumWorldFrame)); %#ok<ASGLU>

    if not(isempty(dManVectors)) && not(isempty(dManMagnitudes)) && not(isempty(dPrePostManTime))
        bHasManoeuvres = true;
    end
end

% Build relative and absolute timegrids
bIsInputTimegridRelative = true; %#ok<*NASGU> % Default assumption
charGridType = "relative";

if not(isempty(dETtimegridVect))

    % Check initial time instant
    dFirstTime = dETtimegridVect(1);
    assert(dFirstTime >= 0, sprintf("ERROR: input timegrid must only contain non-negative real numbers! Found initial timestamp: %6g\n", dFirstTime));

    if not(kwargs.bIsTimeGridRelative)

        bIsInputTimegridRelative = false;
        charGridType = "absolute";
        
        % ET timegrid
        dAbsTimegrid_ET = dETtimegridVect;
        dRelativeTimegridVect = dETtimegridVect - dET0; % Re-write timegrid as relative;
        
        % User defined time grid
        % dAbsTimegrid_UserDef = dTimegridVect_kernelTimeScale;
        dTimegridVect_kernelTimeScale = dTimegridVect_kernelTimeScale - dTime0_kernelTimeScale;

    else
        % Compute timegrid to absolute time grid (ET, GPS, etc.)
        dAbsTimegrid_ET = dETtimegridVect + dET0;
        dRelativeTimegridVect = dETtimegridVect;
        % dAbsTimegrid_UserDef = dTimegridVect_kernelTimeScale + dTime0_kernelTimeScale;
    end

else
    % Build timegrid if not provided
    fprintf("No timegrid was specified as input. Building it to cover entire Ephemeris Time interval with timestep: %4.0f [s]\n", kwargs.dDeltaTimeStep)
    dRelativeTimegridVect = 0:kwargs.dDeltaTimeStep:(dETf - dET0); % Define relative timegrid first
    dAbsTimegrid_ET = dRelativeTimegridVect + dET0;
end

% Print monitoring information
fprintf('Using %s timegrid with relative bounds from ET0: [%8.0f, %8.0f].\nData spans a total of %s.\n', ...
    upper(charGridType), dRelativeTimegridVect(1), dRelativeTimegridVect(end), FormatElapsedTime_(dETtimegridVect(end) - dETtimegridVect(1)));

% Check that last entry of timegrid is within allowed bounds
assert(dAbsTimegrid_ET(end) <= dETf, "ERROR: last time instant specified in absolute timegrid is outside allowed ET bounds! %6.2g < %6.2g\n", dETf, dETtimegridVect(end));

% Convert ET timegrid to kernel timescale
dAbsTimegrid_kernelTimeScale = cspice_unitim(dAbsTimegrid_ET, 'ET', kwargs.charKernelTimescale);

%% Load kernels
try
    cspice_furnsh(char(charKernelFilePath))
catch ME
    error('cspice_furnsh routine failed in loading kernel file with error:\n %s', string(ME.message))
end

%% Fetch data
try
    % Sun position in World frame from reference centre
    dSunPosition_WorldFrame = cspice_spkpos('SUN', dAbsTimegrid_ET, char(enumWorldFrame), 'NONE', varReferenceCentre);
catch ME
    warning('Attempt to retrieve Sun position failed with error: %s', string(ME.message) );
end

try
    % Earth position in World frame from reference centre
    dEarthPosition_WorldFrame = - cspice_spkpos('EARTH', dAbsTimegrid_ET, char(enumWorldFrame), 'NONE', varReferenceCentre);
catch ME
    warning('Attempt to retrieve Earth position failed with error: %s', string(ME.message) );
end

% Spacecraft state in World Frame from reference centre
dStateSC_WorldFrame = cspice_spkezr(charTargetID, dAbsTimegrid_kernelTimeScale, char(enumWorldFrame), 'NONE', varReferenceCentre);

try
    % Target state in World Frame
    % TODO: does spice define frames with a specific origin or only as attitude?
    dTargetPosition_WorldFrame = cspice_spkpos(kwargs.varTargetBodyID, dAbsTimegrid_ET, char(enumWorldFrame), 'NONE', varReferenceCentre); % TODO which observer?? TBC
catch ME
    warning('Attempt to retrieve target body position failed with error: %s', string(ME.message) );
end

try
    % Target attitude in World Frame
    dTargetDCM_TBfromWorld = cspice_pxform(char(enumWorldFrame), char(enumTargetFrame), dAbsTimegrid_ET);
catch ME
    error('Attempt to retrieve target body attitude failed with error: %s', string(ME.message) );
end

%% Process additional targets if available in loaded kernel sets
% kwargs.varTargetBodyID          {mustBeA(kwargs.varTargetBodyID, ["string", "char", "double", "int32", "uint32", "single"])} = varReferenceCentre % Defaults to reference centre in most cases
% kwargs.cellAdditionalTargetsID      {iscell} = {};
% kwargs.cellAdditionalTargetNames    {iscell} = {};
assert(isempty(kwargs.cellAdditionalTargetNames) || length(kwargs.cellAdditionalTargetsID) == length(kwargs.cellAdditionalTargetNames), ...
    'ERROR: cellAdditionalTargetNames and cellAdditionalTargetsID must have the same length if any name has to be specified. Else leave cellAdditionalTargetNames as empty.');

assert( (isempty(kwargs.cellAdditionalTargetFrames) && not(any(kwargs.bAdditionalBodiesRequireAttitude)) ) ||...
    length(kwargs.cellAdditionalTargetsID) == length(kwargs.cellAdditionalTargetFrames), ...
    'ERROR: cellAdditionalTargetFrames and cellAdditionalTargetsID must have the same length if any name has to be specified. Else leave cellAdditionalTargetFrames as empty.');


if ~isempty(kwargs.cellAdditionalTargetsID)

    ui32NumAdditionalBodies = uint32(length(kwargs.cellAdditionalTargetsID));
    cellTargetTags          = cell(1, ui32NumAdditionalBodies);
    cellTargetPos_W         = cell(1, ui32NumAdditionalBodies);
    cellTargetDCM_TBfromW   = cell(1, ui32NumAdditionalBodies);

    if isempty(kwargs.bAdditionalBodiesRequireAttitude)
        kwargs.bAdditionalBodiesRequireAttitude = false(1, ui32NumAdditionalBodies);
    end

    ui32AllocIdx = uint32(1);
    ui32CounterAddBodies = 0;
    for idB = 1:ui32NumAdditionalBodies

        % Get target ID
        varTargetBodyID_ = kwargs.cellAdditionalTargetsID{idB};

        try
            dTmpTargetPosition_WorldFrame = cspice_spkpos(varTargetBodyID_, dAbsTimegrid_ET, ...
                                char(enumWorldFrame), 'NONE', varReferenceCentre); 

            if not(isempty(kwargs.cellAdditionalTargetNames))
                charTmpTargetName = string(kwargs.cellAdditionalTargetNames(idB));
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

            % Attempt to fetch attitude data wrt World frame
            if kwargs.bAdditionalBodiesRequireAttitude(idB)
                dTmpTargetAttitudeDCM_TBfromW = cspice_pxform(char(enumWorldFrame), ...
                                                    char(kwargs.cellAdditionalTargetFrames{idB}), dAbsTimegrid_ET);
                cellTargetDCM_TBfromW{idB} = dTmpTargetAttitudeDCM_TBfromW;
            end

        catch ME
            warning('Attempt to retrieve target body position or attitude failed with error: %s', string(ME.message) );
            continue;
        end
        ui32AllocIdx = ui32AllocIdx + 3;
        ui32CounterAddBodies = ui32CounterAddBodies + 1;
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

% Convert timegrid if user defined time scale differ from kernel timescale
if not(strcmpi('ET', kwargs.charUserDefTimescale))
    dAbsTimegrid_userDefScale = cspice_unitim(dAbsTimegrid_ET, 'ET', kwargs.charUserDefTimescale);
else
    dAbsTimegrid_userDefScale = dAbsTimegrid_ET;
end

% Convert quantities before assignment if needed
[dStateSC_WorldFrame]        = ScaleUnits(dStateSC_WorldFrame, kwargs);
[dTargetPosition_WorldFrame] = ScaleUnits(dTargetPosition_WorldFrame, kwargs);
[dSunPosition_WorldFrame]    = ScaleUnits(dSunPosition_WorldFrame, kwargs);
[dEarthPosition_WorldFrame]  = ScaleUnits(dEarthPosition_WorldFrame, kwargs);


objReferenceMissionData = SReferenceImagesDataset(CCameraIntrinsics(), ...
                                                  enumWorldFrame, ...
                                                  dAbsTimegrid_userDefScale, ...
                                                  dStateSC_WorldFrame, ...
                                                  dTargetDCM_TBfromWorld, ...
                                                  dTargetPosition_WorldFrame, ...
                                                  dSunPosition_WorldFrame, ...
                                                  dEarthPosition_WorldFrame, ...
                                                  "dRelativeTimestamps", dRelativeTimegridVect, ...
                                                  "charLengthUnits", kwargs.charOutputLengthUnits);
% Set time format
objReferenceMissionData.enumTimeScale = kwargs.charUserDefTimescale;


if bHasManoeuvres
    % Convert timegrid if user defined time scale differ from kernel timescale
    dPreManTime_userDefScale = zeros(1, size(dPrePostManTime, 1));

    if not(strcmpi(kwargs.charKernelTimescale, kwargs.charUserDefTimescale))
        dPreManTime_userDefScale(:) = cspice_unitim(dPrePostManTime(:,1)', kwargs.charKernelTimescale, kwargs.charUserDefTimescale);
    else
        dPreManTime_userDefScale(:) = dPrePostManTime(:,1);
    end

    % Assign manoeuvres data
    objReferenceMissionData.dManoeuvresStartTimestamps  = dPreManTime_userDefScale; % Pre-manoeuvre timestamp in ET
    objReferenceMissionData.dManoeuvresDeltaV_SC        = dManVectors; % Actually in World Frame
end

if ui32CounterAddBodies > 0

    for idB = 1:ui32CounterAddBodies
        [cellTargetPos_W{idB}]        = ScaleUnits(cellTargetPos_W{idB}, kwargs);
    end

    % Add additional bodies cells
    objReferenceMissionData.cellAdditionalBodiesPos_W       = cellTargetPos_W(1:ui32CounterAddBodies);
    objReferenceMissionData.cellAdditionalBodiesTags        = cellTargetTags(1:ui32CounterAddBodies);

    objReferenceMissionData.cellAdditionalBodiesDCM_TBfromW = cellTargetDCM_TBfromW(1:ui32CounterAddBodies);
    objReferenceMissionData.cellAdditionalTargetFrames      = kwargs.cellAdditionalTargetFrames(1:ui32CounterAddBodies);
end


fprintf("\nFetching and packaging completed.\n")
if nargout == 1
    return;
end
%% Additional data (if required)
% State of spacecraft in Target Frame
try
    dStateSC_TargetFixed = cspice_spkezr(charTargetID, dAbsTimegrid_ET, char(enumTargetFrame), 'NONE', varReferenceCentre);
catch ME
    warning('Attempt to retrieve spacecraft state in target body frame failed with error: %s', string(ME.message) );
    dStateSC_TargetFixed = [];
end
if nargout == 2
    return;
end

% Apophis position to Sun in target frame
try
    dSunPosition_TargetFixed = cspice_spkpos('SUN', dAbsTimegrid_ET, char(enumTargetFrame), 'NONE', varReferenceCentre);
catch
    warning('Attempt to retrieve Sun position in target body frame failed with error: %s', string(ME.message) );
    dStateSC_TargetFixed = [];
end


if nargout == 3
    return;
end

end

% Local function to get elapsed time as string
function charElapsedTime = FormatElapsedTime_(dSeconds)

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

function [varTargetID, charTargetID] = ValidateID_(varTargetID)

dTmpNumber = str2double(varTargetID);
bIsNumberTargetID = not(isnan(dTmpNumber));

if isnumeric(varTargetID) || bIsNumberTargetID
    varTargetID = int32(varTargetID);
    charTargetID = char(sprintf('%d', varTargetID));
else
    try
        varTargetID = char(varTargetID);
        charTargetID = char(varTargetID);
    catch
        error('Invalid input datatype for target ID: must be an integer numeric value or a string/char. Got: %s.', class(varTargetID))
    end
end

end

function [dArrayToScale] = ScaleUnits(dArrayToScale, kwargs)
if strcmpi(kwargs.charOutputLengthUnits, 'm') && strcmpi(kwargs.charKernelLengthUnits, 'km')

    try
        dArrayToScale         = 1000 * dArrayToScale;
    catch ME
        warning('Scaling failed due to error: %s', string(ME.message) )
    end
elseif strcmpi(kwargs.charOutputLengthUnits, 'km') && strcmpi(kwargs.charKernelLengthUnits, 'm')

    try
        dArrayToScale         = 1./1000 * dArrayToScale;
    catch ME
        warning('Scaling failed due to error: %s', string(ME.message) )
    end
end
end

