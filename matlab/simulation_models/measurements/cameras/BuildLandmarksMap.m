function [o_dLMpos_FILTER_TB, o_bValidLMinMap] = BuildLandmarksMap(i_dLMposTable_TB, initializedLMsMask, i_iSAM2estimator, i_ui16MaxMapSize)
arguments
    i_dLMposTable_TB   (4, :) {isnumeric}
    initializedLMsMask (1, :) {isnumeric}
    i_iSAM2estimator   (1, 1) {isobject}
    i_ui16MaxMapSize   (1, 1) uint16
end

%% PROTOTYPE
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-06-2024         Pietro Califano        Functon version adapted from script code
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code
import gtsam.*

extractedCounter = 1;
% Initialize current map from ISAM2
o_dLMpos_FILTER_TB = zeros(4, i_ui16MaxMapSize);
o_bValidLMinMap = false(i_ui16MaxMapSize, 1);

NUM_SIMULATED_LM = size(i_dLMposTable_TB, 2);
idLMarray = 1:NUM_SIMULATED_LM;

% Get estimates of landmarks in map until maximum number of landmarks used by filter is reached
for idL = NUM_SIMULATED_LM:-1:1
    % for idL = 1:1:NUM_SIMULATED_LM

    if initializedLMsMask(idLMarray(idL)) == true && extractedCounter <= i_ui16MaxMapSize

        landmarkKey = symbol('L', i_dLMposTable_TB(1, idL));
        % If value exist in graph for the given key, then extract it
        % if iSAM2estimator.valueExists(landmarkKey)
        o_dLMpos_FILTER_TB(1, extractedCounter)   = i_dLMposTable_TB(1, idL);
        o_dLMpos_FILTER_TB(2:4, extractedCounter) = i_iSAM2estimator.calculateEstimatePoint3(landmarkKey);
        o_bValidLMinMap(extractedCounter) = true;

        extractedCounter = extractedCounter + 1;
        % end

    end
end

end
