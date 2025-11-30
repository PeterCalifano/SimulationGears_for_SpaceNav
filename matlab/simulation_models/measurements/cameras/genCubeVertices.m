function [dLandmarkPositions_TB] = genCubeVertices(dHalfSideSize)%#codegen
arguments (Input) 
    dHalfSideSize (1,1) double {isscalar, isnumeric}
end
arguments (Output)
    dLandmarkPositions_TB (4, 9) double {ismatrix}
end
%% PROTOTYPE
% [dLandmarkPositions_TB] = genCubeVertices(dHalfSideSize)%#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dHalfSideSize (1,1) double {isscalar, isnumeric}
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dLandmarkPositions_TB (4, 9) double {ismatrix}
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-05-2024        Pietro Califano         Function created.
% 14-01-2025        Pietro Califano         Update for re-use in unit tests.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

dLandmarkPositions_TB = [1, dHalfSideSize, dHalfSideSize, - dHalfSideSize;
                         2, dHalfSideSize, dHalfSideSize, dHalfSideSize;
                         3, - dHalfSideSize, dHalfSideSize, dHalfSideSize;
                         4, - dHalfSideSize, dHalfSideSize, - dHalfSideSize;
                         5, - dHalfSideSize, - dHalfSideSize, dHalfSideSize;
                         6, - dHalfSideSize, - dHalfSideSize, - dHalfSideSize;
                         7, dHalfSideSize, - dHalfSideSize, - dHalfSideSize;
                         8, dHalfSideSize, -dHalfSideSize, dHalfSideSize;
                         9, 0, 0, 0]';

end
