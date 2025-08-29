function [dTimeGrid, bInsideWindowMask] = BuildTimegridFromWindows(charJsonPath, dDeltaTime, kwargs)
% BuildTimeGridFromWindows Generates a time grid and mask from JSON window file.
%
%   [dTimeGrid, bInsideWindowMask] = BuildTimeGridFromWindows(charJsonPath, dDeltaTime)
%
%   Inputs:
%       charJsonPath    (1,:) char : Path to JSON file with fields t_start, t_end
%       dDeltaTime      (1,1) double : Time step [s]
%
%   Outputs:
%       dTimeGrid           (1,N) double : Time grid covering all windows and in-between
%       bInsideWindowMask   (1,N) logical : True if time is inside any window

arguments (Input)
    charJsonPath (1,:) char     {mustBeFile}
    dDeltaTime   (1,1) double   {mustBePositive}
end
arguments (Input)
    kwargs.ui32NumStepsBefore       (1,1) uint32 {mustBeNumeric, mustBeScalarOrEmpty} = 5
    kwargs.ui32NumStepsAfter        (1,1) uint32 {mustBeNumeric, mustBeScalarOrEmpty} = 5
    kwargs.ui32MaxNumWindows        (1,1) uint32 {mustBeScalarOrEmpty} = 1E4
    kwargs.dMinWindowDuration       (1,1) double {mustBeNumeric, mustBePositive, mustBeScalarOrEmpty} = 20
    kwargs.dMaxDuration             (1,1) double {mustBeNumeric, mustBeScalarOrEmpty} = -1
    kwargs.charKernelTimescale      (1,:) char {ischar, isstring, mustBeMember(kwargs.charKernelTimescale, ...
                                                    ["TAI", "TDB", "TDT", "TT", "ET", "JDTDB", "JDTDT", "JED", "GPS"])} = "ET";
    kwargs.charUserDefTimescale     (1,:) char {ischar, isstring, mustBeMember(kwargs.charUserDefTimescale, ...
                                                        ["TAI", "TDB", "TDT", "TT", "ET", "JDTDB", "JDTDT", "JED", "GPS"])} = "ET";
    kwargs.i32TargetLabelID         (1,1) int32  = -1
    kwargs.dInitialRelTime          (1,1) double = 0.0
    kwargs.dInputTimeGrid           (1,:) double  {mustBeVector, mustBeNonnegative} = zeros(1,2);
end
arguments (Output)
    dTimeGrid           (1,:) double  {mustBeVector, mustBeNonnegative}
    bInsideWindowMask   (1,:) logical {mustBeVector}
end

% Read the windows file
charWindowsFileString = jsondecode(fileread(charJsonPath));

dBeginTstamps = arrayfun(@(s) s.t_start, charWindowsFileString);
dEndTstamps   = arrayfun(@(s) s.t_end,   charWindowsFileString);

%%% Filter based on initial relative time
dRelBeginTimes = dBeginTstamps - dBeginTstamps(1);
bIdxValidRelBeginTimes = dRelBeginTimes >= kwargs.dInitialRelTime;

dBeginTstamps = dBeginTstamps(bIdxValidRelBeginTimes);
dEndTstamps   = dEndTstamps  (bIdxValidRelBeginTimes);

%%% Apply constraints on time windows
% Calculate window durations
dWindowDurations = dEndTstamps - dBeginTstamps;

% Apply minimum duration constraint
bIdxValidDuration = dWindowDurations >= kwargs.dMinWindowDuration;
dBeginTstamps     = dBeginTstamps(bIdxValidDuration);
dEndTstamps       = dEndTstamps(bIdxValidDuration);

% Limit number of windows to maximum allowed
ui32NumWindows = min(length(dBeginTstamps), kwargs.ui32MaxNumWindows);
dBeginTstamps = dBeginTstamps(1:ui32NumWindows);
dEndTstamps   = dEndTstamps(1:ui32NumWindows);

if kwargs.i32TargetLabelID >= 0
    % Filter windows based on target ID
    ui32TargetLabelIDs = arrayfun(@(s) s.lbl, charWindowsFileString);

    % Apply filtering masks
    ui32TargetLabelIDs = ui32TargetLabelIDs(bIdxValidRelBeginTimes);
    ui32TargetLabelIDs = ui32TargetLabelIDs(bIdxValidDuration);
    ui32TargetLabelIDs = ui32TargetLabelIDs(1:ui32NumWindows);
end

% Compute extended time limits for the timegrid
dMinT = min(dBeginTstamps);
dMaxT = max(dEndTstamps);

%%% Determine timegrid

% dTimeGrid = dTStart + (0 : dTotalNumSteps - 1) * dDeltaTime;
if size(kwargs.dInputTimeGrid,2) > 2 && all(kwargs.dInputTimeGrid >= 0.0)
    % Use input timegrid
    dTimeGrid = kwargs.dInputTimeGrid;
else
    % Build timegrid from limits and steps
    dTStart = dMinT - double(kwargs.ui32NumStepsBefore) * dDeltaTime;
    dTEnd   = dMaxT + double(kwargs.ui32NumStepsAfter) * dDeltaTime;

    % Constrain maximum duration if max provided
    if kwargs.dMaxDuration > 0
        dTEnd = dTStart + min(dTEnd-dTStart, kwargs.dMaxDuration);
    end

    dTotalNumSteps = floor( (dTEnd - dTStart) / dDeltaTime) + 1;
    dTimeGrid = dTStart : dDeltaTime : dTEnd;
end

% Mask: true if inside timestamp is within any window
bInsideWindowMask = false(1, length(dTimeGrid));

% Loop over each window to mark entries using "union"
for iW = 1:numel(dBeginTstamps)
    bInsideWindowMask = bInsideWindowMask | ...
        (dTimeGrid >= dBeginTstamps(iW) & dTimeGrid <= dEndTstamps(iW));
end

% Filter based on target ID if provided
if kwargs.i32TargetLabelID >= 0
    bInsideWindowTargetMask = false(1, length(bInsideWindowMask));
    ui32ImageAcquisitionWindCount = uint32(0);

    if kwargs.i32TargetLabelID >= 0
            % Build visibility mask of specific target ID
        for iW = 1:numel(dBeginTstamps)
            
            if ui32TargetLabelIDs(iW) == kwargs.i32TargetLabelID
                bCurrentWindMask = (dTimeGrid >= dBeginTstamps(iW) & dTimeGrid <= dEndTstamps(iW));
                bInsideWindowTargetMask(bCurrentWindMask) = true;

                ui32ImageAcquisitionWindCount = ui32ImageAcquisitionWindCount + uint32(1);
            end
        end

    end

    % Update output mask
    bInsideWindowMask = bInsideWindowTargetMask & bInsideWindowMask;
    assert(ui32ImageAcquisitionWindCount == sum(ui32TargetLabelIDs == kwargs.i32TargetLabelID), ...
        'ERROR: mismatch of number of image acquisition windows due to target visibility. Something may have gone wrong.')
end

% Convert timegrid if user defined time scale differs
if not(strcmpi(kwargs.charKernelTimescale, kwargs.charUserDefTimescale))
    dTimeGrid = cspice_unitim(dTimeGrid, kwargs.charKernelTimescale, kwargs.charUserDefTimescale);
end

end
