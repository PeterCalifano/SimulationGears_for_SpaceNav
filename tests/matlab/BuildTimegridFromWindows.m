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
    kwargs.ui32NumStepsBefore  (1,1) uint32 {mustBeNumeric, mustBeScalarOrEmpty} = 5
    kwargs.ui32NumStepsAfter   (1,1) uint32 {mustBeNumeric, mustBeScalarOrEmpty} = 5
    kwargs.ui32MaxNumWindows   (1,1) uint32 {mustBeScalarOrEmpty} = 1E4
    kwargs.dMinWindowDuration  (1,1) double {mustBeNumeric, mustBePositive, mustBeScalarOrEmpty} = 20
end
arguments (Output)
    dTimeGrid           (1,:) double  {mustBeVector, mustBeNonnegative}
    bInsideWindowMask   (1,:) logical {mustBeVector}
end

% Read the windows file
charWindowsFilestring = jsondecode(fileread(charJsonPath));
dBeginTstamps = arrayfun(@(s) s.t_start, charWindowsFilestring);
dEndTstamps   = arrayfun(@(s) s.t_end,   charWindowsFilestring);

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

% Compute extended time limits for the timegrid
dMinT = min(dBeginTstamps);
dMaxT = max(dEndTstamps);

dTStart = dMinT - double(kwargs.ui32NumStepsBefore) * dDeltaTime;
dTEnd   = dMaxT + double(kwargs.ui32NumStepsAfter) * dDeltaTime;

%%% Build timegrid
dTotalNumSteps = floor( (dTEnd - dTStart) / dDeltaTime) + 1;
% dTimeGrid = dTStart + (0 : dTotalNumSteps - 1) * dDeltaTime;
dTimeGrid = dTStart : dDeltaTime : dTEnd;

% Mask: true if inside timestamp is within any window
bInsideWindowMask = false(1, length(dTimeGrid));

% Loop over each window to mark entries using "union"
for iW = 1:numel(dBeginTstamps)
    bInsideWindowMask = bInsideWindowMask | ...
        (dTimeGrid >= dBeginTstamps(iW) & dTimeGrid <= dEndTstamps(iW));
end

end
