function [dirStruct] = getDirStruct(dirPath)
%% PROTOTYPE
% [dirStruct] = getDirStruct(dirPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to get directory struct of input dirPath automatically removing "." and ".." fields.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dirPath: [string] Input directory path
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dirStruct: [struct] Directory path struct
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-05-2025        Pietro Califano       Function using dir to get directory structure removing "." and ".."
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Get output folder info
dirStruct = dir(dirPath);

% Remove useless directories
removeEntryMask = false(length(dirStruct) ,1);

for idF = 1:length(dirStruct)
    if strcmpi(dirStruct(idF).name, '.') || strcmpi(dirStruct(idF).name, '..')
        removeEntryMask(idF) = true;
    end
end

dirStruct(removeEntryMask) = [];

end
