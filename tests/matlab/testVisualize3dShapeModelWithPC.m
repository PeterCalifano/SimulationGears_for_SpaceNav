function tests = testVisualize3dShapeModelWithPC
%% SIGNATURE
% tests = testVisualize3dShapeModelWithPC
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify that detailed wireframe shape models are reduced before they are passed to MATLAB graphics.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% [-]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Visualize3dShapeModelWithPC
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);

end

function setupOnce(testCase)
charTestPath = fileparts(mfilename("fullpath"));
charRepositoryPath = fullfile(charTestPath, "..");
testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
    charRepositoryPath, IncludingSubfolders=true));
end

function testDetailedWireframeUsesReferencedVerticesOnly(testCase)
ui32NumFaces = uint32(12000);
ui32NumVertices = 3 * ui32NumFaces;

strShapeModel.dVerticesPos = reshape(double(1:(3 * double(ui32NumVertices))), ...
    3, double(ui32NumVertices));
strShapeModel.ui32triangVertexPtr = reshape(uint32(1:ui32NumVertices), ...
    3, double(ui32NumFaces));

charPreviousFigureVisibility = get(groot, "DefaultFigureVisible");
set(groot, "DefaultFigureVisible", "off");
objVisibilityCleanup = onCleanup(@() set(groot, ...
    "DefaultFigureVisible", charPreviousFigureVisibility)); %#ok<NASGU>

[objFigure, cellPlotObjects] = Visualize3dShapeModelWithPC(strShapeModel, ...
    [1; 1; 1], [0; 0; 0], eye(3), ...
    "bEnableLegend", false, ...
    "bUseBlackBackground", false, ...
    "bShowAsWireframe", true);
objFigureCleanup = onCleanup(@() close(objFigure)); %#ok<NASGU>

objPatch = cellPlotObjects{1};
testCase.verifyEqual(size(objPatch.Faces, 1), 10000);
testCase.verifyLessThanOrEqual(size(objPatch.Vertices, 1), 30000);
testCase.verifyEqual(double(max(objPatch.Faces, [], "all")), ...
    size(objPatch.Vertices, 1));
end
