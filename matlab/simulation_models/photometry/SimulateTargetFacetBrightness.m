function [dBrightness, strBrightnessData] = SimulateTargetFacetBrightness(ui32FaceVertexIds, ...
                                                                          dVerticesPos_TB, ...
                                                                          dDCM_INfromTB, ...
                                                                          dSunPosition_IN, ...
                                                                          dObserverPosition_IN, ...
                                                                          kwargs)
arguments (Input)
    ui32FaceVertexIds       (:,:) uint32
    dVerticesPos_TB         (:,:) double {mustBeFinite, mustBeNumeric}
    dDCM_INfromTB           (3,3) double {mustBeFinite, mustBeNumeric}
    dSunPosition_IN         (3,1) double {mustBeFinite, mustBeNumeric}
    dObserverPosition_IN    (3,1) double {mustBeFinite, mustBeNumeric}
    kwargs.charScatteringLaw (1,:) char {mustBeMember(kwargs.charScatteringLaw, ...
                                      {'lommel_seeliger', 'lambert', 'mixed'})} = 'lommel_seeliger'
    kwargs.dLambertWeight   (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(kwargs.dLambertWeight, 0.0), ...
                                      mustBeLessThanOrEqual(kwargs.dLambertWeight, 1.0)} = 0.5
    kwargs.bApplyRangeScaling (1,1) logical = false
end
arguments (Output)
    dBrightness             (1,1) double
    strBrightnessData       (1,1) struct
end
%% SIGNATURE
% [dBrightness, strBrightnessData] = SimulateTargetFacetBrightness(ui32FaceVertexIds, dVerticesPos_TB, dDCM_INfromTB, dSunPosition_IN, dObserverPosition_IN, kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute relative disk-integrated target brightness from a triangular shape model, target attitude, Sun vector, and observer vector. Facets contribute only when both illuminated and visible. Supported scattering laws are Lambert, Lommel-Seeliger, and a convex mix of both.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32FaceVertexIds       (3,N) or (N,3) uint32  Triangle vertex ids, one-based.
% dVerticesPos_TB         (3,N) or (N,3) double  Vertex positions in target-body frame.
% dDCM_INfromTB           (3,3) double           DCM mapping target-body vectors to inertial/world frame.
% dSunPosition_IN         (3,1) double           Sun position relative to target centre in inertial/world frame.
% dObserverPosition_IN    (3,1) double           Observer position relative to target centre in inertial/world frame.
% kwargs.charScatteringLaw (1,:) char            'lommel_seeliger', 'lambert', or 'mixed'.
% kwargs.dLambertWeight   (1,1) double           Lambert contribution for mixed law.
% kwargs.bApplyRangeScaling (1,1) logical        Apply inverse-square Sun and observer range scaling.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dBrightness             (1,1) double           Relative brightness.
% strBrightnessData       (1,1) struct           Diagnostics for phase angle and active facet contributions.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      First prototype for approach-phase spin lightcurve estimation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Accept both row-major and column-major mesh storage conventions before
% vectorized facet geometry is evaluated.
[ui32FaceRows, dVertexRows_TB] = NormalizeShapeModelInputs_(ui32FaceVertexIds, dVerticesPos_TB);

ui32NumVertices = uint32(size(dVertexRows_TB, 1));
if any(ui32FaceRows(:) < uint32(1)) || any(ui32FaceRows(:) > ui32NumVertices)
    error('SimulateTargetFacetBrightness:InvalidFaceVertexIds', ...
          'Face vertex ids must be one-based and within the vertex array.');
end

% Build per-facet centroids, outward unit normals, and physical areas from
% the triangular mesh. Degenerate triangles are masked and rejected if they
% are the only available facets.
dFaceVertex1_TB = dVertexRows_TB(double(ui32FaceRows(:, 1)), :);
dFaceVertex2_TB = dVertexRows_TB(double(ui32FaceRows(:, 2)), :);
dFaceVertex3_TB = dVertexRows_TB(double(ui32FaceRows(:, 3)), :);

dFaceCentroid_TB = (dFaceVertex1_TB + dFaceVertex2_TB + dFaceVertex3_TB) / 3.0;
dEdge12_TB = dFaceVertex2_TB - dFaceVertex1_TB;
dEdge13_TB = dFaceVertex3_TB - dFaceVertex1_TB;
dRawNormalRows_TB = cross(dEdge12_TB, dEdge13_TB, 2);
dDoubleArea = vecnorm(dRawNormalRows_TB, 2, 2);
bValidAreaMask = dDoubleArea > 100.0 * eps('double');

if ~any(bValidAreaMask)
    error('SimulateTargetFacetBrightness:DegenerateShapeModel', ...
          'Shape model has no non-degenerate triangular facets.');
end

dUnitNormalRows_TB = zeros(size(dRawNormalRows_TB));
dUnitNormalRows_TB(bValidAreaMask, :) = dRawNormalRows_TB(bValidAreaMask, :) ./ dDoubleArea(bValidAreaMask);

dMeshCentre_TB = mean(dVertexRows_TB, 1);
dOutwardCheckRows_TB = dFaceCentroid_TB - dMeshCentre_TB;
bInwardNormalMask = sum(dUnitNormalRows_TB .* dOutwardCheckRows_TB, 2) < 0.0;
dUnitNormalRows_TB(bInwardNormalMask, :) = -dUnitNormalRows_TB(bInwardNormalMask, :);

dFacetArea = 0.5 * dDoubleArea;

% Work in the target-body frame so facet normals, Sun direction, and
% observer direction are all expressed in the same coordinates.
dDCM_TBfromIN = transpose(dDCM_INfromTB);
dSunPosition_TB = dDCM_TBfromIN * dSunPosition_IN;
dObserverPosition_TB = dDCM_TBfromIN * dObserverPosition_IN;

dFacetToSunRows_TB = transpose(dSunPosition_TB) - dFaceCentroid_TB;
dFacetToObserverRows_TB = transpose(dObserverPosition_TB) - dFaceCentroid_TB;
dSunDistance = vecnorm(dFacetToSunRows_TB, 2, 2);
dObserverDistance = vecnorm(dFacetToObserverRows_TB, 2, 2);

bValidGeometryMask = bValidAreaMask & dSunDistance > eps('double') & dObserverDistance > eps('double');

dSunDirectionRows_TB = zeros(size(dFacetToSunRows_TB));
dObserverDirectionRows_TB = zeros(size(dFacetToObserverRows_TB));
dSunDirectionRows_TB(bValidGeometryMask, :) = dFacetToSunRows_TB(bValidGeometryMask, :) ./ dSunDistance(bValidGeometryMask);
dObserverDirectionRows_TB(bValidGeometryMask, :) = dFacetToObserverRows_TB(bValidGeometryMask, :) ./ dObserverDistance(bValidGeometryMask);

dCosIncidence = max(sum(dUnitNormalRows_TB .* dSunDirectionRows_TB, 2), 0.0);
dCosEmission = max(sum(dUnitNormalRows_TB .* dObserverDirectionRows_TB, 2), 0.0);
bActiveFacetMask = bValidGeometryMask & dCosIncidence > 0.0 & dCosEmission > 0.0;

% Evaluate the selected single-scattering law. The mixed law is a convex
% combination useful for simulation sweeps between Lambert and LS behavior.
dLambertContribution = dFacetArea .* dCosIncidence .* dCosEmission;
dLommelSeeligerContribution = dFacetArea .* dCosIncidence .* dCosEmission ./ ...
                              max(dCosIncidence + dCosEmission, eps('double'));

switch kwargs.charScatteringLaw
    case 'lambert'
        dFacetBrightness = dLambertContribution;
    case 'lommel_seeliger'
        dFacetBrightness = dLommelSeeligerContribution;
    case 'mixed'
        dFacetBrightness = kwargs.dLambertWeight * dLambertContribution + ...
                           (1.0 - kwargs.dLambertWeight) * dLommelSeeligerContribution;
end

dFacetBrightness(~bActiveFacetMask) = 0.0;

% Optional inverse-square scaling keeps the default output as a relative
% shape/phase brightness while allowing range-aware synthetic photometry.
if kwargs.bApplyRangeScaling
    dFacetBrightness(bActiveFacetMask) = dFacetBrightness(bActiveFacetMask) ./ ...
                                         (dSunDistance(bActiveFacetMask).^2 .* dObserverDistance(bActiveFacetMask).^2);
end

dBrightness = sum(dFacetBrightness, 'all');

dTargetToSunUnit_IN = dSunPosition_IN / max(norm(dSunPosition_IN), eps('double'));
dTargetToObserverUnit_IN = dObserverPosition_IN / max(norm(dObserverPosition_IN), eps('double'));
dPhaseAngleRad = acos(max(-1.0, min(1.0, dot(dTargetToSunUnit_IN, dTargetToObserverUnit_IN))));

strBrightnessData = struct();
strBrightnessData.dPhaseAngleRad = dPhaseAngleRad;
strBrightnessData.dPhaseAngleDeg = rad2deg(dPhaseAngleRad);
strBrightnessData.ui32NumActiveFacets = uint32(nnz(bActiveFacetMask));
strBrightnessData.dFacetBrightness = dFacetBrightness;
strBrightnessData.dCosIncidence = dCosIncidence;
strBrightnessData.dCosEmission = dCosEmission;

end

function [ui32FaceRows, dVertexRows_TB] = NormalizeShapeModelInputs_(ui32FaceVertexIds, dVerticesPos_TB)
% Normalize shape arrays to row-major faces and vertices.

if size(ui32FaceVertexIds, 2) == 3
    ui32FaceRows = ui32FaceVertexIds;
elseif size(ui32FaceVertexIds, 1) == 3
    ui32FaceRows = transpose(ui32FaceVertexIds);
else
    error('SimulateTargetFacetBrightness:InvalidFaceArray', ...
          'Face vertex ids must have shape 3xN or Nx3.');
end

if size(dVerticesPos_TB, 2) == 3
    dVertexRows_TB = dVerticesPos_TB;
elseif size(dVerticesPos_TB, 1) == 3
    dVertexRows_TB = transpose(dVerticesPos_TB);
else
    error('SimulateTargetFacetBrightness:InvalidVertexArray', ...
          'Vertex positions must have shape 3xN or Nx3.');
end

if size(ui32FaceRows, 1) == 0 || size(dVertexRows_TB, 1) == 0
    error('SimulateTargetFacetBrightness:EmptyShapeModel', ...
          'Shape model faces and vertices must be non-empty.');
end

end
