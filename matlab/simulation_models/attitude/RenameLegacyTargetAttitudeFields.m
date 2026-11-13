function strAttitude = RenameLegacyTargetAttitudeFields(strAttitude)
%% SIGNATURE
% strAttitude = RenameLegacyTargetAttitudeFields(strAttitude)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Give the target rate and spin-axis fields of a saved attitude struct their current names. Data saved before
% 13-11-2026 uses dAngVel_IN, dAngVel_TB, dAngVelTimegrid, enumAngularVelocityRegime, dSpinAxis_TB and
% charSpinAxisSource; the current names carry the Target prefix. Loaders of saved packs call this on the
% strAttData, strMainBodyRefData or history struct they read. A struct that holds both the old and the new name
% of one field is rejected.
% TODO: remove this function and its calls once the saved JGCD 2026 packs are exported with the new names.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strAttitude    (1,1) struct   attitude struct read from saved data
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strAttitude    (1,1) struct   the same struct with the current field names
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-11-2026  Pietro Califano, Claude Opus 5.5   First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strAttitude (1,1) struct
end
arguments (Output)
    strAttitude (1,1) struct
end

% List each old name next to its current name
cellFieldNames = {'dAngVel_IN',                'dTargetAngVel_IN'; ...
                  'dAngVel_TB',                'dTargetAngVel_TB'; ...
                  'dAngVelTimegrid',           'dTargetAngVelTimegrid'; ...
                  'enumAngularVelocityRegime', 'enumTargetAngularVelocityRegime'; ...
                  'dSpinAxis_TB',              'dTargetSpinAxis_TB'; ...
                  'charSpinAxisSource',        'charTargetSpinAxisSource'};

% Move every old field to its current name, unless the struct already has both
for ui32FieldIdx = uint32(1):uint32(size(cellFieldNames, 1))
    charOldName = cellFieldNames{ui32FieldIdx, 1};
    charNewName = cellFieldNames{ui32FieldIdx, 2};

    if ~isfield(strAttitude, charOldName)
        continue
    end

    if isfield(strAttitude, charNewName)
        error('RenameLegacyTargetAttitudeFields:DuplicateField', ...
            'The attitude struct holds both %s and its current name %s.', charOldName, charNewName);
    end

    strAttitude.(charNewName) = strAttitude.(charOldName);
    strAttitude = rmfield(strAttitude, charOldName);
end

end
