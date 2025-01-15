function [o_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffDataPath, ui16lMax, bENABLE_UNSCALING)
%% PROTOTYPE
% [o_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffDataPath, ui16lMax, bENABLE_UNSCALING)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to load Spherical Harmonics Expansion coefficients from storage file (.mat or any other like
% .txt, .dat, csv). In the latter cases, comma separated rows are assumed: [l,m,Clm,Slm,sigmaClm,sigmaSlm].
% The input data are also assumed to contain a first row that does not contain useful coefficients: the
% function discards it in the current version (assumed to be the [1,0]).
% ACHTUNG: input model coefficient must start from degree/order = 1. Automatically handled if loaded from
%          file other than .mat.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% modelCoeffDataPath:  [1]   Path to file containing the model coefficients. Files .mat are directly loaded;
%                            other data files such as .dat are read by importSHEcoeffsFromFile() function
%                            according to the format: comma separated [1x6] row: [l,m,Clm,Slm,sigClm,sigSlm]
%                            First row gets discarded (assumed to be the [1,0])
% ui16lMax:            [1]   Maximum requested degree of the Expansion
% bENABLE_UNSCALING:   [1]   Bool flag enabling the conversion from normalized to unnormalized coefficients
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dCSlmCoeffCols:    [(2:ui16lMax)+1, 2]  [Clm, Slm] Coefficients of the SHE gravity model
% o_dlmPairs:          [(2:ui16lMax)+1, 2]  Pairs (l,m) of the loaded coefficients for verification
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-02-2024        Pietro Califano         Loading and un-scaling of SHE coefficient model.
% 08-02-2024        Pietro Califano         Added function to load data from file.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% importSHEcoeffsFromFile()
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% DEFINE
lMAX_ALLOWABLE = 83;

if isempty(fullfile(which(modelCoeffDataPath)))
    [~, ~, fileExtension] = fileparts(modelCoeffDataPath);
else
    [~, ~, fileExtension] = fileparts(fullfile(which(modelCoeffDataPath)));
end

if strcmp(fileExtension, '.mat')
    % Load file
    modelCoeffData = load(modelCoeffDataPath);
elseif not(strcmp(fileExtension, ""))
    % Call function for import from data file (not .mat)
    modelCoeffData = importSHEcoeffsFromFile(modelCoeffDataPath, false, ui16lMax);
else
    error('No file extension detected: please check input.')
end

if isstruct(modelCoeffData) % Handle struct() case
    % Get fieldnames
    structfieldNames = fieldnames(modelCoeffData);

    if length(structfieldNames) > 1
        warning('Multiple fields in loaded struct: first is taken! Check loaded file.')
    end

    % Extract LUTs from model oefficients array
    Clm_LUTbar = modelCoeffData.(structfieldNames{1})(1:end, 3); % Starting from degree/order = 1
    Slm_LUTbar = modelCoeffData.(structfieldNames{1})(1:end, 4);
    o_dlmPairs = modelCoeffData.(structfieldNames{1})(1:end, 1:2);

elseif isnumeric(modelCoeffData) % Handle array case

    % Extract LUTs from model oefficients array
    Clm_LUTbar = modelCoeffData(1:end, 3); % Starting from degree/order = 1
    Slm_LUTbar = modelCoeffData(1:end, 4);
    o_dlmPairs = modelCoeffData(1:end, 1:2);

else
    error('Loaded file is neither a struct nor a numeric array. Please check inputs.')
end

% Find maximum order and pre-allocate output arrays
% ui16lMax = 83;
nEntries = sum((2:ui16lMax)+1) + 1;

% Normalized to Un-normalized coefficients conversion module
if bENABLE_UNSCALING == true

    if ui16lMax > lMAX_ALLOWABLE
        warning(strcat("Override occurred: Scaling factors routine only supports lMax=", num2str(lMAX_ALLOWABLE)));
        ui16lMax = lMAX_ALLOWABLE;
        nEntries = sum((2:ui16lMax)+1);
    end

    Clm_LUT = zeros(nEntries, 1);
    Slm_LUT = zeros(nEntries, 1);

    % Generate scaling factors
    scaleFactors = ExtSHE_normFactors(ui16lMax);
    % 
    % Clm_LUT(1) = 1; % l=1
    % Slm_LUT(1) = 0; % m=1

    Clm_LUT(1:end) = Clm_LUTbar(1:nEntries)./scaleFactors;
    Slm_LUT(1:end) = Slm_LUTbar(1:nEntries)./scaleFactors;

    % Pack output
    o_dCSlmCoeffCols = [Clm_LUT, Slm_LUT];
else
    
    % Pack output
    o_dCSlmCoeffCols = [Clm_LUTbar(1:nEntries), Slm_LUTbar(1:nEntries)];
end

o_dlmPairs = o_dlmPairs(1:nEntries, :);

%% LOCAL FUNCTION
    function scaleFactors = ExtSHE_normFactors(lMax) %#codegen
        %% PROTOTYPE
        % [scaleFactors] = ExtSHE_normFactors(lMax)
        % -------------------------------------------------------------------------------------------------------------
        %% DESCRIPTION
        % Function computing the scaling factors of the SHE model coefficients to convert between Normalized 
        % and Non-normalized (scaled) coefficients, typically indicated as (Clm-bar, Slm-bar) and (Clm, Slm) 
        % respectively. A 1D array containing the scaling coefficients for each entry of the Clm and Slm as 
        % required by function ExtSHE_GradU(), is produced.
        % REFERENCE:
        % 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
        % -------------------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 18-08-2023    Pietro Califano     Function coded. Run-time verified. Output validated with Vallado
        %                                   Appendix D4 tables.
        % -------------------------------------------------------------------------------------------------------------
        %% INPUT
        % lMax:         [1]                 Maximum required degree of the expansion
        % -------------------------------------------------------------------------------------------------------------
        %% OUTPUT
        % scaleFactors: [sum((2:lMax)+1)]   Number of scaleFactors, function of lMax
        % -------------------------------------------------------------------------------------------------------------

        %% Function code
        % Compute number of coefficients
        MAX_ALLOWABLE_DEGREE = 83;
        arraySize = sum((2:lMax)+1) + 1;

        if lMax <= MAX_ALLOWABLE_DEGREE

            % Initialize storage arrays
            scaleFactors = zeros(arraySize, 1);
            scaleFactors(1) = 1;

            % Compute Scaling factors for each (l, m) pairs: (1, 1) to (lMax, lMax)
            idPair = 2;
            for l = 2:lMax
                for m = 0:l
                    if m > 0
                        scaleFactors(idPair) = sqrt(factorial(l+m) /...
                            ( (factorial(l-m)) * 2 * (2*l+1) ));

                    else
                        scaleFactors(idPair) = sqrt(factorial(l+m) /...
                            ( (factorial(l-m)) * 1 * (2*l+1) ));
                    end
                    idPair = idPair + 1;
                end
            end
            if (idPair-1 ~= arraySize)
                error('Mismatch between precomputed arraySize and allocator index idPair. Result may be incorrect.')
            end
        else
            error('Current implementation does not support order > 84 due to factorial() limitation.');
        end

    end
end
