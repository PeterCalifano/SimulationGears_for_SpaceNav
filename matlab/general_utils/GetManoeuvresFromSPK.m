function [dTargetCover, dManVectors, dManMagnitudes, dStatePreMan, ...
            dPrePostManTime, dStatePostMan, ui32NumIntervals] = GetManoeuvresFromSPK(i32TargetID, kwargs)
arguments
    i32TargetID (1,1) int32 {mustBeNumeric}
end
arguments
    kwargs.charKernelFolderPath     (1,:) char {mustBeText}
    kwargs.charKernelNameWithoutExt (1,:) char {mustBeText}
    kwargs.ui32MaxNumOfIntervals    (1,1) int32 {mustBeNumeric} = 100;
    kwargs.varRefCentre             {mustBeA(kwargs.varRefCentre, ["string", "char", "int32", "double"])} = ""
    kwargs.charKernelLengthUnits    (1,:) char {mustBeText, mustBeMember(kwargs.charKernelLengthUnits, ["km", "m"])} = "km";
    kwargs.charOutputLengthUnits    (1,:) char {mustBeText, mustBeMember(kwargs.charOutputLengthUnits, ["km", "m"])} = "m";
    kwargs.charEnumWorldFrame       (1,:) char {mustBeText} = "J2000";
end
%% SIGNATURE
% [dTargetCover, dManVectors, dManMagnitudes, dStatePreMan, ...
%  dPrePostManTime, dStatePostMan, ui32NumIntervals] = GetManoeuvresFromSPK(i32TargetID, kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to fetch manoeuvres states and DeltaV from SPK files, using cspice_spkcov. Manoeuvres are assumed
% to be impulsive and instantaneous, thus computed as difference of velocities.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments
%     i32TargetID (1,1) int32 {mustBeNumeric}
% end
% arguments
%     kwargs.charKernelFolderPath     (1,:) char {mustBeText}
%     kwargs.charKernelNameWithoutExt (1,:) char {mustBeText}
%     kwargs.ui32MaxNumOfIntervals    (1,1) int32 {mustBeNumeric} = 100;
%     kwargs.varRefCentre             {mustBeA(kwargs.varRefCentre, ["string", "char", "int32", "double"])} = ""
%     kwargs.charKernelLengthUnits    (1,:) char {mustBeText, mustBeMember(kwargs.charKernelLengthUnits, ["km", "m"])} = "km";
%     kwargs.charOutputLengthUnits    (1,:) char {mustBeText, mustBeMember(kwargs.charOutputLengthUnits, ["km", "m"])} = "m";
%     kwargs.charEnumWorldFrame       (1,:) char {mustBeText} = "J2000";
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dTargetCover
% dManVectors
% dManMagnitudes
% dStatePreMan
% dPrePostManTime
% dStatePostMan
% ui32NumIntervals
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-06-2025        Pietro Califano         First version for RCS-1 mission (generic).
% 23-06-2025        Pietro Califano         Update with function to fetch manoeuvres from SPK.
% 10-10-2025        Pietro Califano         [HOTFIX] Fix incorrect loop to fetch manoeuvres and print
%                                           segments. Implementation validated with RCS1 kernels.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code

if not(strcmpi(kwargs.charKernelFolderPath, "")) && not(strcmpi(kwargs.charKernelNameWithoutExt, ""))
    % Load kernel if specified
    cspice_furnsh( char( fullfile(kwargs.charKernelFolderPath, ...
        sprintf("%s.bsp", char(kwargs.charKernelNameWithoutExt)))) );
else
    error('Not kernel path and name specified. Please provide them as (key, value) inputs.')
end

% Get coverage for target ID
dTargetCover = cspice_spkcov( char( fullfile(kwargs.charKernelFolderPath, ...
                sprintf("%s.bsp", char(kwargs.charKernelNameWithoutExt))) ), ...
                int32(i32TargetID), ...
                kwargs.ui32MaxNumOfIntervals );

if isempty(dTargetCover)
    fprintf('No coverage found for ID %d in %s\n', i32TargetID, kwargs.charKernelNameWithoutExt);
    return
end

ui32NumIntervals = numel(dTargetCover)/2;
dManVectors     = zeros(3, ui32NumIntervals);
dManMagnitudes  = zeros(1, ui32NumIntervals);
dStatePreMan    = zeros(6, ui32NumIntervals);
dStatePostMan   = zeros(6, ui32NumIntervals);
dPreManTime     = zeros(1, ui32NumIntervals);
dPostManTime    = zeros(1, ui32NumIntervals);

% Loop over segments, print start/stop, and detect maneuver at each new start
ui32ManCounter = uint32(1);
dSegmentDeltaTimeThr = 1e-3;
fprintf('\n\n--------------------------------------- Manoeuvres info -------------------------------------------------\n');
for idInt = 1:numel(dTargetCover)-1

    idx = idInt : idInt + 1;

    % Convert both endpoints to strings
    charEndPointTimes = cspice_timout( dTargetCover(idx)', 'YYYY MON DD HR:MN:SC.### (TDB) ::TDB' );

    % Compute delta time to next
    dDeltaTimeToNext = dTargetCover(idx(2)) - dTargetCover(idx(1));

    if dDeltaTimeToNext >= dSegmentDeltaTimeThr
            fprintf('Segment %2d: duration: %5.5g [days], %s --> %s\n', ui32ManCounter, ...
                                                    diff(dTargetCover(idx)')/86400, ...
                                                    charEndPointTimes(1,:), ...
                                                    charEndPointTimes(2,:));
    end
    
    if dDeltaTimeToNext < dSegmentDeltaTimeThr

        % Print start of segment i is the burn epoch
        fprintf('   --> Maneuver at: %s\n\n', charEndPointTimes(1,:) );

        % If any centre of frame is specified, get state and compute impulsive manoeuvres
        if not(strcmpi(kwargs.varRefCentre, ""))

            % Get timestamps of endpoints
            dPreManTime(ui32ManCounter)  = dTargetCover(idx(1));     % stop time of segment i-1
            dPostManTime(ui32ManCounter) = dTargetCover(idx(2));     % start time of segment i

            % Pull state‐vectors [x y z vx vy vz]
            [dStatePreMan(:,ui32ManCounter), ~]  = cspice_spkezr( num2str(i32TargetID), dPreManTime(ui32ManCounter),  ...
                char(kwargs.charEnumWorldFrame), 'NONE', char(kwargs.varRefCentre) );
            [dStatePostMan(:,ui32ManCounter), ~] = cspice_spkezr( num2str(i32TargetID), dPostManTime(ui32ManCounter), ...
                char(kwargs.charEnumWorldFrame), 'NONE', char(kwargs.varRefCentre) );

            % DeltaV vector and magnitude (km/s)
            dManVectors(:, ui32ManCounter) = dStatePostMan(4:6, ui32ManCounter) - dStatePreMan(4:6, ui32ManCounter);
            dManMagnitudes(ui32ManCounter) = norm(dManVectors(:, ui32ManCounter));

            fprintf('Burn %2d:\n', ui32ManCounter);
            fprintf('  Pre‐burn @ %s\n',  dPreManTime(ui32ManCounter));
            fprintf('  Post‐burn@ %s\n',  dPostManTime(ui32ManCounter));
            fprintf('  ΔV = [%8.6f %8.6f %8.6f] %s/s  -->  |ΔV| = %.6f %s/s\n\n', ...
                dManVectors(1, ui32ManCounter), dManVectors(2, ui32ManCounter), ...
                dManVectors(3, ui32ManCounter), kwargs.charKernelLengthUnits, ...
                dManMagnitudes(ui32ManCounter), kwargs.charKernelLengthUnits);

            ui32ManCounter = ui32ManCounter + 1;
        end
    end
end

fprintf('-----------------------------------------------------------------------------------------------------\n\n');
assert(all(vecnorm(abs(dStatePostMan(1:3,:) - dStatePreMan(1:3,:)), 2, 1) < 1E-5, 'all'), ...
    ['ERROR: difference in position at time instants of manouvres is larger than 1E-12. ' ...
    'This function only supports instantaneous manoeuvres.']) 

% Remove unallocated entries
% TODO after test

% If output units is "m", convert states and manoeuvres
if strcmpi(kwargs.charOutputLengthUnits, 'm') && strcmpi(kwargs.charKernelLengthUnits, 'km')

    dManVectors     = 1000 * dManVectors;
    dManMagnitudes  = 1000 * dManMagnitudes;
    dStatePreMan    = 1000 * dStatePreMan ;
    dStatePostMan   = 1000 * dStatePostMan;

elseif strcmpi(kwargs.charOutputLengthUnits, 'km') && strcmpi(kwargs.charKernelLengthUnits, 'm')

    dManVectors     = 1./1000 * dManVectors;
    dManMagnitudes  = 1./1000 * dManMagnitudes;
    dStatePreMan    = 1./1000 * dStatePreMan ;
    dStatePostMan   = 1./1000 * dStatePostMan;

end

% Stack pre and post manoeuvres timestamps
dPrePostManTime = [dPreManTime', dPostManTime'];

