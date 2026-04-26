function strDiagnostics = ComputePolyhedronGravitySHfitDiagnostics( ...
    ui32FaceVertexIds, dVerticesPos, strSHgravityData, options)
arguments
    ui32FaceVertexIds               (:,3) uint32
    dVerticesPos                    (:,3) double {mustBeFinite, mustBeReal}
    strSHgravityData                (1,1) struct
    options.dHoldoutShellRadii      (1,:) double {mustBeFinite, mustBeReal, mustBePositive} = []
    options.ui32HoldoutPtsPerShell  (:,:) uint32 = uint32([])
    options.ui32NumHoldoutShells    (1,1) uint32 = uint32(4)
    options.dHoldoutMinRadiusScale  (1,1) double {mustBeFinite, mustBeReal, mustBePositive} = 1.15
    options.dHoldoutMaxRadiusScale  (1,1) double {mustBeFinite, mustBeReal, mustBePositive} = 3.0
    options.dPhaseBase              (1,1) double {mustBeFinite, mustBeReal} = 17.0
    options.ui32EdgeVertexIds = uint32([])
    options.dEdgeDyadics = []
    options.dFaceDyadics = []
end
%% PROTOTYPE
% strDiagnostics = ComputePolyhedronGravitySHfitDiagnostics( ...
%     ui32FaceVertexIds, dVerticesPos, strSHgravityData, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes holdout diagnostics between the exact polyhedron gravity model
% and a fitted exterior spherical harmonics perturbation model.
%
% The holdout set is built on exterior spherical shells and the returned
% diagnostics include pointwise errors, shell-wise RMS errors, and the
% sampled polyhedron / SH perturbation fields.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32FaceVertexIds:              [nFaces x 3]      Triangle vertex indices.
% dVerticesPos:                   [nVertices x 3]   Vertex coordinates.
% strSHgravityData:               struct            Output of
%                                                  FitSpherHarmCoeffToPolyhedrGrav().
% options.dHoldoutShellRadii:     [1 x Ns]          Optional explicit holdout shell radii.
% options.ui32HoldoutPtsPerShell: [1 x Ns]          Optional per-shell holdout counts.
% options.ui32NumHoldoutShells:   [1]               Number of holdout shells for auto mode.
% options.dHoldoutMinRadiusScale: [1]               Minimum auto shell radius scale.
% options.dHoldoutMaxRadiusScale: [1]               Maximum auto shell radius scale.
% options.dPhaseBase:             [1]               Shell point-set phase offset.
% options.ui32EdgeVertexIds:      [nEdges x 2]      Optional precomputed edge vertex pairs.
% options.dEdgeDyadics:           [3 x 3 x Ne]      Optional precomputed edge dyadics.
% options.dFaceDyadics:           [3 x 3 x Nf]      Optional precomputed face dyadics.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strDiagnostics: struct containing holdout sample positions, polyhedron
%                 and SH perturbation fields, shell settings, and error
%                 metrics.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Add reusable polyhedron-vs-SH holdout diagnostics.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePolyhedronFaceEdgeData()
% ComputeLogSpacedValues()
% GenerateShellPointSet()
% EvalPolyhedronGravPerturbationSamples()
% EvalExtSphHarmInTargetFrame()
% ComputeGravityFieldFitMetrics()
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Validate required fields in strSHgravityData
cellRequiredFields = {'dCSlmCoeffCols', 'ui32MaxDegree', 'dGravParam', ...
    'dBodyRadiusRef', 'dDensity', 'dGravConst'};
for idField = 1:numel(cellRequiredFields)
    assert(isfield(strSHgravityData, cellRequiredFields{idField}), ...
        'ComputePolyhedronGravitySHfitDiagnostics:InvalidSHgravityData', ...
        'Missing required field "%s" in strSHgravityData.', cellRequiredFields{idField});
end

% Validate face vertex indices
if isempty(options.dHoldoutShellRadii)
    dHoldoutShellRadii = ComputeLogSpacedValues( ...
        options.dHoldoutMinRadiusScale * strSHgravityData.dBodyRadiusRef, ...
        options.dHoldoutMaxRadiusScale * strSHgravityData.dBodyRadiusRef, ...
        options.ui32NumHoldoutShells);
else
    dHoldoutShellRadii = options.dHoldoutShellRadii;
end

if any(dHoldoutShellRadii <= strSHgravityData.dBodyRadiusRef)
    error('ComputePolyhedronGravitySHfitDiagnostics:InvalidHoldoutRadii', ...
        'All holdout shell radii must lie strictly outside the reference radius.');
end

% Compute the number of unknown SH coefficients for reference when setting holdout counts
ui32NumUnknowns = strSHgravityData.ui32MaxDegree * strSHgravityData.ui32MaxDegree + ...
    2 * strSHgravityData.ui32MaxDegree - uint32(3);

ui32HoldoutPtsPerShell = reshape(options.ui32HoldoutPtsPerShell, 1, []);
if isempty(ui32HoldoutPtsPerShell)
    ui32HoldoutPtsPerShell = repmat(uint32(max(256, 4 * double(ui32NumUnknowns))), size(dHoldoutShellRadii));

elseif isscalar(ui32HoldoutPtsPerShell)
    ui32HoldoutPtsPerShell = repmat(ui32HoldoutPtsPerShell, size(dHoldoutShellRadii));

elseif numel(ui32HoldoutPtsPerShell) ~= numel(dHoldoutShellRadii)
    error('ComputePolyhedronGravitySHfitDiagnostics:InvalidHoldoutCounts', ...
        'ui32HoldoutPtsPerShell must be empty, scalar, or match the number of holdout shells.');
end

if any(ui32HoldoutPtsPerShell < uint32(1), 'all')
    error('ComputePolyhedronGravitySHfitDiagnostics:InvalidHoldoutCounts', ...
        'Each holdout shell must contain at least one sample point.');
end

% Compute edge and face dyadics if not provided (required for polyhedron gravity evaluation)
if isempty(options.ui32EdgeVertexIds) || isempty(options.dEdgeDyadics) || isempty(options.dFaceDyadics)

    [ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData( ...
        ui32FaceVertexIds, dVerticesPos);

else
    ui32EdgeVertexIds = options.ui32EdgeVertexIds;
    dEdgeDyadics = options.dEdgeDyadics;
    dFaceDyadics = options.dFaceDyadics;
end

% Generate holdout sample set on spherical shells
[dHoldoutPos_TB, ui32HoldoutShellIds] = GenerateShellPointSet( ...
    dHoldoutShellRadii, ui32HoldoutPtsPerShell, options.dPhaseBase);

% Evaluate perturbation fields at holdout points
[dPotentialPolyPert, dAccPolyPertTB] = EvalPolyhedronGravPerturbationSamples( ...
    dHoldoutPos_TB, ui32FaceVertexIds, dVerticesPos, strSHgravityData.dDensity, ...
    ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, ...
    strSHgravityData.dGravConst, strSHgravityData.dGravParam);

% Evaluate SH perturbation at holdout points
[dPotentialSHEpert, dAccSHEpertTB] = EvalExtSphHarmInTargetFrame( ...
    dHoldoutPos_TB, strSHgravityData.ui32MaxDegree, strSHgravityData.dCSlmCoeffCols, ...
    strSHgravityData.dGravParam, strSHgravityData.dBodyRadiusRef);

% Compute error metrics between the two perturbation fields at holdout points
strMetrics = ComputeGravityFieldFitMetrics( ...
    dPotentialSHEpert, dPotentialPolyPert, dAccSHEpertTB, dAccPolyPertTB, ...
    ui32HoldoutShellIds, uint32(numel(dHoldoutShellRadii)));

% Compile diagnostics into output struct
strDiagnostics = struct();
strDiagnostics.dHoldoutShellRadii = dHoldoutShellRadii;
strDiagnostics.ui32HoldoutPtsPerShell = ui32HoldoutPtsPerShell;
strDiagnostics.dHoldoutPos_TB = dHoldoutPos_TB;
strDiagnostics.ui32HoldoutShellIds = ui32HoldoutShellIds;
strDiagnostics.dPotentialPolyPert = dPotentialPolyPert;
strDiagnostics.dAccPolyPertTB = dAccPolyPertTB;
strDiagnostics.dPotentialSHEpert = dPotentialSHEpert;
strDiagnostics.dAccSHEpertTB = dAccSHEpertTB;
strDiagnostics.strMetrics = strMetrics;

end
