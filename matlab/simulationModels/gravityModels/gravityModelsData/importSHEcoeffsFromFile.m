function [modelSHEcoefficients, savedFileName] = importSHEcoeffsFromFile(PathToFile, bENABLE_SAVING, lMax) 
%% PROTOTYPE
% [modelSHEcoefficients, savedFileName] = importSHEcoeffsFromFile(PathToFile, bENABLE_SAVING, lMax) 
% ----------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to read Spherical Harmonics coefficients from file. The function assumes the common data format
% provided by agencies and institutions: a comma separated [1x6] row: [l,m,Clm,Slm,sigmaClm,sigmaSlm].
% ACHTUNG: the output is formatted for use in "loadSHEcoeffModel" function. Specifically, the first row must
%          contain the coefficients for the pair (l=1,m=1).
% ----------------------------------------------------------------------------------------------------------
%% INPUT
% PathToFile:            [1]   Path to accessible file (function using fscanf to read)
% bENABLE_SAVING:        [1]   Bool flag to enable saving of the data array extracted from file
% lMax:                  [1]   Maximum requested degree to load to file
% ----------------------------------------------------------------------------------------------------------
%% OUTPUT
% modelSHEcoefficients:  [sum((1:lMax)+1)-1, 6]  Array of coefficients as read from file (first row removed)
% savedFileName:         [1]                     Name of the saved file containing modelSHEcoefficients
% ----------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-02-2023        Pietro Califano         Prototype coded to handle typical format of SHE coefficients
% ----------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% ----------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% ----------------------------------------------------------------------------------------------------------
%% Function code

% Initialize variables
% Maximum degree set based on limitation of function to convert from Normalized to  Non-normalized 
% coefficients of the SHE (ExtSHE_normFactors)
if nargin < 3
    lMax = 83;
    if nargin < 2
        bENABLE_SAVING = false;
    end
end
if lMax == Inf
    endRow = Inf;
else
    endRow = sum((1:lMax)+1);
end
%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
formatSpec = '%f,%f,%f,%f,%f,%f\n';

%% Open the text file.
fileID = fopen(PathToFile,'r');

%% Read data according to the format
dataArray = fscanf(fileID, formatSpec, [6, endRow])';

%% Close the text file.
fclose(fileID);

%% Create output variable
modelSHEcoefficients = dataArray(2:end, :);

%% Save file if requested

if bENABLE_SAVING
    % Get information about the file
    fileInfo = dir(fullfile(which(PathToFile)));
    % Access the file name from the fileInfo structure
    savedFileName = fileInfo.name;
    % Remove file extension
    [~, savedFileName, ~] = fileparts(savedFileName);
    % Save file .mat
    save(savedFileName, "modelSHEcoefficients");
else
    savedFileName = "dataNotSaved";
end


