function [strSRPpanelData, objShapeModel] = BuildQuadsModelSRPDataFromObj(charObjFilePath, options)
arguments
    charObjFilePath (1,1) string {mustBeA(charObjFilePath, ["string", "char"])}
    options.charInputUnit (1,:) string {mustBeA(options.charInputUnit, ["string", "char"]), mustBeMember(options.charInputUnit, ["m", "km"])} = "m"
    options.charTargetUnitOutput (1,:) string {mustBeA(options.charTargetUnitOutput, ["string", "char"]), mustBeMember(options.charTargetUnitOutput, ["m", "km"])} = "m"
    options.dDiffuseCoeff (1,1) double {mustBeFinite, mustBeNonnegative, mustBeLessThanOrEqual(options.dDiffuseCoeff, 1)} = 0.0
    options.dSpecularCoeff (1,1) double {mustBeFinite, mustBeNonnegative, mustBeLessThanOrEqual(options.dSpecularCoeff, 1)} = 0.0
    options.dDiffSpecQuadsCoeffs (:,2) double {mustBeFinite, mustBeNonnegative, mustBeLessThanOrEqual(options.dDiffSpecQuadsCoeffs, 1)} = zeros(0, 2)
    options.bOrientNormalsOutward (1,1) logical = true
    options.bFlipNormals (1,1) logical = false
    options.dMeshSimplifyFactor (1,1) double {mustBeFinite} = 1.0
end
%% PROTOTYPE
% [strSRPpanelData, objShapeModel] = BuildQuadsModelSRPDataFromObj(charObjFilePath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Loads a Wavefront OBJ mesh through CShapeModel and converts its triangular faces into the flat-panel
% data required by ComputeQuadsModelSRP. Each triangle is represented by its area, unit outward normal,
% pressure center, and optical coefficients.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charObjFilePath                  (1,1) string   Path to Wavefront OBJ file.
% options.charInputUnit            (1,:) string   Length unit used by the OBJ vertices.
% options.charTargetUnitOutput     (1,:) string   Length unit for output pressure centers and areas.
% options.dDiffuseCoeff            (1,1) double   Default diffuse coefficient for every panel.
% options.dSpecularCoeff           (1,1) double   Default specular coefficient for every panel.
% options.dDiffSpecQuadsCoeffs     (:,2) double   Optional per-panel [diffuse, specular] coefficients.
% options.bOrientNormalsOutward    (1,1) logical  Orient normals away from the mesh vertex centroid.
% options.bFlipNormals             (1,1) logical  Flip all normals after orientation.
% options.dMeshSimplifyFactor      (1,1) double   CShapeModel load-time mesh keep fraction.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strSRPpanelData.dSCquadsArea           (N,1) double   Panel areas.
% strSRPpanelData.dDiffSpecQuadsCoeffs   (N,2) double   [diffuse, specular] optical coefficients.
% strSRPpanelData.dQuadsNormals_SCB      (3,N) double   Panel normals in spacecraft body frame.
% strSRPpanelData.dQuadsPressCentre_SCB  (3,N) double   Panel pressure centers in spacecraft body frame.
% objShapeModel                                      Loaded CShapeModel object.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5      Add OBJ-to-panel SRP preprocessing utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CShapeModel
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Load OBJ file and extract vertices and triangular face vertex indices, applying length unit conversion if needed
objShapeModel = CShapeModel("file_obj", charObjFilePath, ...
    options.charInputUnit, options.charTargetUnitOutput, true, "", true, ...
    dMeshSimplifyFactor=options.dMeshSimplifyFactor);

ui32FaceVertexIds = uint32(objShapeModel.ui32triangVertexPtr');
dVerticesPos = objShapeModel.dVerticesPos';

assert(~isempty(ui32FaceVertexIds) && ~isempty(dVerticesPos), ...
    'BuildQuadsModelSRPDataFromObj:EmptyMesh', ...
    'The OBJ file must contain vertices and triangular faces.');

% Make panels from triangular faces and compute their areas, normals, and pressure centers
dFaceVerts1 = dVerticesPos(ui32FaceVertexIds(:, 1), :);
dFaceVerts2 = dVerticesPos(ui32FaceVertexIds(:, 2), :);
dFaceVerts3 = dVerticesPos(ui32FaceVertexIds(:, 3), :);

dFaceCross = cross(dFaceVerts2 - dFaceVerts1, dFaceVerts3 - dFaceVerts1, 2);
dDoubleArea = sqrt(sum(dFaceCross.^2, 2));

assert(all(dDoubleArea > 0), ...
    'BuildQuadsModelSRPDataFromObj:DegenerateFace', ...
    'OBJ faces used for SRP panel preprocessing must have non-zero area.');

dPanelArea = 0.5 * dDoubleArea;
dPanelNormals = dFaceCross ./ dDoubleArea;
dPanelCenters = (dFaceVerts1 + dFaceVerts2 + dFaceVerts3) / 3.0;

if options.bOrientNormalsOutward

    % Ensure normals are oriented outward from the mesh
    dMeshCentroid = mean(dVerticesPos, 1);
    bInwardNormal = sum(dPanelNormals .* (dPanelCenters - dMeshCentroid), 2) < 0;
    dPanelNormals(bInwardNormal, :) = -dPanelNormals(bInwardNormal, :);
end

if options.bFlipNormals
    dPanelNormals = -dPanelNormals;
end

% Assign optical coefficients, using per-panel values if provided, otherwise default values
nPanels = size(ui32FaceVertexIds, 1);
if isempty(options.dDiffSpecQuadsCoeffs)
    dDiffSpecQuadsCoeffs = repmat([options.dDiffuseCoeff, options.dSpecularCoeff], nPanels, 1);
else
    assert(size(options.dDiffSpecQuadsCoeffs, 1) == nPanels, ...
        'BuildQuadsModelSRPDataFromObj:CoeffSizeMismatch', ...
        'Per-panel optical coefficients must have one row per OBJ face.');
    dDiffSpecQuadsCoeffs = options.dDiffSpecQuadsCoeffs;
end

strSRPpanelData = struct();
strSRPpanelData.dSCquadsArea = dPanelArea;
strSRPpanelData.dDiffSpecQuadsCoeffs = dDiffSpecQuadsCoeffs;
strSRPpanelData.dQuadsNormals_SCB = dPanelNormals';
strSRPpanelData.dQuadsPressCentre_SCB = dPanelCenters';
strSRPpanelData.ui32FaceVertexIds = ui32FaceVertexIds;
strSRPpanelData.dVerticesPos = dVerticesPos;
strSRPpanelData.ui32NumPanels = uint32(nPanels);
strSRPpanelData.charSourceObjFilePath = char(charObjFilePath);
strSRPpanelData.charLengthUnit = char(options.charTargetUnitOutput);

end
