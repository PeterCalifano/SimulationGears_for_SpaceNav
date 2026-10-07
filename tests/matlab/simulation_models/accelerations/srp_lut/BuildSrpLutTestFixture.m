function [strLut, strHost, strPanel] = BuildSrpLutTestFixture(bConstant, bIncludeTransverse, dAngularGridStep)
%% SIGNATURE
% [strLut, strHost, strPanel] = BuildSrpLutTestFixture(bConstant, bIncludeTransverse, dAngularGridStep)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build a compact fixture from one optical plate at the selected spacing, or replace its
% response with an exact constant scalar coefficient for independent oracles.
% Example: strLut = BuildSrpLutTestFixture(true);
% Output: A validated 73-by-37 numeric payload with coefficient two.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% bConstant           Select the constant radial fixture; default false.
% bIncludeTransverse   Include vector storage in the fixture; default true.
% dAngularGridStep     Grid spacing in degrees; default five.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strLut      Compact fixed-schema numeric response table.
% strHost     Host response and construction metadata.
% strPanel    Synthetic one-plate geometry for independent direct evaluations.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Add independent SRP contract fixtures.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpResponseLut, PackSrpResponseLut.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    bConstant (1, 1) logical = false
    bIncludeTransverse (1, 1) logical = true
    dAngularGridStep (1, 1) double = 5
end

arguments (Output)
    strLut (1, 1) struct
    strHost (1, 1) struct
    strPanel (1, 1) struct
end

% Build one optical triangle without self-shadowing or external assets.
strPanel = struct('dVerticesPos', [0, 0, 0;0, 1, 0;0, 0, 1], ...
    'ui32FaceVertexIds', uint32([1, 2, 3]), 'dSCquadsArea', 0.5, ...
    'dQuadsNormals_SCB', [1;0;0], 'dQuadsPressCentre_SCB', [0;1/3;1/3], ...
    'dDiffSpecQuadsCoeffs', [0.15, 0.75], 'charSourceObjFilePath', '');
strHost = BuildSrpResponseLut(strPanel, 0.5, dAngularGridStep, ...
    bSelfShadowing=false, ui32ShadowLevel=uint32(0),bUseCodegen=false);

% Replace the panel response with an exact radial law for independent derivatives.
if bConstant
    strHost.dEffectiveCr(:) = 2;
    [dAzimuth, dElevation] = meshgrid(strHost.dAzimuth, strHost.dElevation);
    strHost.dForcePerPressure(1, :, :) = -reshape(cosd(dElevation).*cosd(dAzimuth), ...
        1, size(dElevation, 1), size(dElevation, 2));
    strHost.dForcePerPressure(2, :, :) = -reshape(cosd(dElevation).*sind(dAzimuth), ...
        1, size(dElevation, 1), size(dElevation, 2));
    strHost.dForcePerPressure(3, :, :) = -reshape(sind(dElevation), ...
        1, size(dElevation, 1), size(dElevation, 2));
    strHost.dTransverseForcePerPressure(:) = 0;
end

% Preserve the fixed runtime schema for both fixture variants.
strLut = PackSrpResponseLut(strHost, ...
    ui32Capacity=uint32([numel(strHost.dAzimuth), numel(strHost.dElevation)]), ...
    bIncludeTransverse=bIncludeTransverse);
strHost.strResponseLut = strLut;
end
