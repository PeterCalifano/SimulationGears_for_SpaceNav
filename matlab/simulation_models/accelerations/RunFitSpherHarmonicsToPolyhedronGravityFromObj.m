function strRunOutputs = RunFitSpherHarmonicsToPolyhedronGravityFromObj(charObjFilePath, ui32MaxDegree, options)
arguments
    charObjFilePath                 (1,:) string {mustBeA(charObjFilePath, ["string", "char"])}
    ui32MaxDegree                   (1,1) uint32
    options.charInputUnit           (1,:) string {mustBeA(options.charInputUnit, ["string", "char"]), ...
        mustBeMember(options.charInputUnit, ["m", "km"])} = "m"
    options.charTargetUnitOutput    (1,:) string {mustBeA(options.charTargetUnitOutput, ["string", "char"]), ...
        mustBeMember(options.charTargetUnitOutput, ["m", "km"])} = "m"
    options.bVertFacesOnly          (1,1) logical = true
    options.charModelName           (1,:) string {mustBeA(options.charModelName, ["string", "char"])} = ""
    options.dGravParam              (1,1) double = NaN
    options.dDensity                (1,1) double = NaN
    options.dGravConst              (1,1) double = 6.67430e-11
    options.dBodyRadiusRef          (1,1) double = NaN
    options.ui32MaxFitIterations    (1,1) uint32 = uint32(5)
    options.bCacheOnShapeModel      (1,1) logical = true
    options.dHoldoutShellRadii      (1,:) double {mustBeFinite, mustBeReal, mustBePositive} = []
    options.ui32HoldoutPtsPerShell  (:,:) uint32 = uint32([])
    options.ui32NumHoldoutShells    (1,1) uint32 = uint32(4)
    options.dHoldoutMinRadiusScale  (1,1) double {mustBeFinite, mustBeReal, mustBePositive} = 1.15
    options.dHoldoutMaxRadiusScale  (1,1) double {mustBeFinite, mustBeReal, mustBePositive} = 3.0
    options.dHoldoutPhaseBase       (1,1) double {mustBeFinite, mustBeReal} = 17.0
    options.bShowMeshFigure         (1,1) logical = true
    options.bShowConvergenceFigure  (1,1) logical = true
    options.bShowHoldoutFigure      (1,1) logical = true
    options.charFigureRenderer      (1,:) string {mustBeA(options.charFigureRenderer, ["string", "char"])} = "opengl"
    options.bUseBlackBackground     (1,1) logical = false
    options.bVerbose                (1,1) logical = true
end
%% PROTOTYPE
% strRunOutputs = RunFitSpherHarmonicsToPolyhedronGravityFromObj(charObjFilePath, ui32MaxDegree, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% User-facing entry point to:
% 1) load a triangulated mesh from a Wavefront .obj file,
% 2) fit exterior spherical harmonics coefficients from the exact
%    polyhedron gravity field,
% 3) optionally cache the fitted SH data on the CShapeModel object,
% 4) compute holdout diagnostics against the exact polyhedron model,
% 5) generate visualization figures for the mesh, fit convergence, and
%    holdout errors.
%
% All inputs are explicit so the user can specify them manually at the
% call site without editing internal code.
%
% Example:
% strRunOutputs = RunFitSpherHarmonicsToPolyhedronGravityFromObj( ...
%     "/path/to/body.obj", uint32(8), ...
%     dDensity=2100.0, ...
%     ui32MaxFitIterations=uint32(4), ...
%     bShowMeshFigure=true, ...
%     bShowConvergenceFigure=true, ...
%     bShowHoldoutFigure=true);
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charObjFilePath:                [1]         Path to the input .obj file.
% ui32MaxDegree:                  [1]         Maximum SH degree.
% options.charInputUnit:          [1]         Input mesh unit ('m' or 'km').
% options.charTargetUnitOutput:   [1]         Internal/output mesh unit ('m' or 'km').
% options.bVertFacesOnly:         [1]         Load only vertices and faces from the .obj file.
% options.charModelName:          [1]         Optional model name.
% options.dGravParam:             [1]         Optional gravitational parameter.
% options.dDensity:               [1]         Optional density.
% options.dGravConst:             [1]         Gravitational constant.
% options.dBodyRadiusRef:         [1]         Optional SH reference radius override.
% options.ui32MaxFitIterations:   [1]         Maximum adaptive fit iterations.
% options.bCacheOnShapeModel:     [1]         Store the SH fit on the returned CShapeModel object.
% options.dHoldoutShellRadii:     [1 x Ns]    Optional explicit holdout radii.
% options.ui32HoldoutPtsPerShell: [1 x Ns]    Optional holdout point counts.
% options.ui32NumHoldoutShells:   [1]         Number of auto-generated holdout shells.
% options.dHoldoutMinRadiusScale: [1]         Minimum auto holdout shell scale.
% options.dHoldoutMaxRadiusScale: [1]         Maximum auto holdout shell scale.
% options.dHoldoutPhaseBase:      [1]         Holdout shell phase offset.
% options.bShowMeshFigure:        [1]         Show mesh figure.
% options.bShowConvergenceFigure: [1]         Show convergence figure.
% options.bShowHoldoutFigure:     [1]         Show holdout diagnostics figure.
% options.charFigureRenderer:     [1]         MATLAB renderer.
% options.bUseBlackBackground:    [1]         Use repo black-background plot style.
% options.bVerbose:               [1]         Print fit and holdout summary.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strRunOutputs: struct containing:
%   objShapeModel     Loaded shape model object.
%   strSHgravityData  Fitted spherical harmonics data.
%   strDiagnostics    Holdout diagnostics against the polyhedron model.
%   strFigureHandles  Generated figure handles.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Add user-facing run entry point for OBJ-to-SH workflow.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% FitSpherHarmonicsToPolyhedronGravityFromObj()
% ComputePolyhedronGravitySHfitDiagnostics()
% PlotPolyhedronSHfitDiagnostics()
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Call the compute-only builder function to get the shape model and SH fit data
[objShapeModel, strSHgravityData] = FitSpherHarmonicsToPolyhedronGravityFromObj(charObjFilePath, ui32MaxDegree, ...
                                                                    charInputUnit=options.charInputUnit, ...
                                                                    charTargetUnitOutput=options.charTargetUnitOutput, ...
                                                                    bVertFacesOnly=options.bVertFacesOnly, ...
                                                                    charModelName=options.charModelName, ...
                                                                    dGravParam=options.dGravParam, ...
                                                                    dDensity=options.dDensity, ...
                                                                    dGravConst=options.dGravConst, ...
                                                                    dBodyRadiusRef=options.dBodyRadiusRef, ...
                                                                    ui32MaxFitIterations=options.ui32MaxFitIterations, ...
                                                                    bCacheOnShapeModel=options.bCacheOnShapeModel);

% Build polyhedron gravity data on the shape model if not already present (required for diagnostics)
objShapeModel = objShapeModel.BuildPolyhedronGravityData();
[ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, ui32FacesRows, dVerticesRows] = objShapeModel.getPolyhedronGravityData();

% Compute holdout diagnostics against the exact polyhedron gravity model and the fitted SH perturbation
strDiagnostics = ComputePolyhedronGravitySHfitDiagnostics(ui32FacesRows, dVerticesRows, strSHgravityData, ...
                                                    dHoldoutShellRadii=options.dHoldoutShellRadii, ...
                                                    ui32HoldoutPtsPerShell=options.ui32HoldoutPtsPerShell, ...
                                                    ui32NumHoldoutShells=options.ui32NumHoldoutShells, ...
                                                    dHoldoutMinRadiusScale=options.dHoldoutMinRadiusScale, ...
                                                    dHoldoutMaxRadiusScale=options.dHoldoutMaxRadiusScale, ...
                                                    dPhaseBase=options.dHoldoutPhaseBase, ...
                                                    ui32EdgeVertexIds=ui32EdgeVertexIds, ...
                                                    dEdgeDyadics=dEdgeDyadics, ...
                                                    dFaceDyadics=dFaceDyadics);

% Make diagnostic plots for the mesh, fit convergence, and holdout errors
strShapeModel = objShapeModel.getShapeStruct();
strFigureHandles = PlotPolyhedronSHfitDiagnostics(strShapeModel, strSHgravityData, strDiagnostics, ...
                                                    bShowMeshFigure=options.bShowMeshFigure, ...
                                                    bShowConvergenceFigure=options.bShowConvergenceFigure, ...
                                                    bShowHoldoutFigure=options.bShowHoldoutFigure, ...
                                                    charFigureRenderer=options.charFigureRenderer, ...
                                                    charDistanceUnit=options.charTargetUnitOutput, ...
                                                    bUseBlackBackground=options.bUseBlackBackground);

% Print summary of fit and holdout diagnostics if verbose
if options.bVerbose
    if options.charModelName == ""
        [~, charModelStem, ~] = fileparts(char(charObjFilePath));
        charModelName = string(charModelStem);
    else
        charModelName = options.charModelName;
    end

    fprintf('\nPolyhedron-to-SH fit summary\n');
    fprintf('  Model:                %s\n', char(charModelName));
    fprintf('  File:                 %s\n', char(charObjFilePath));
    fprintf('  Max degree:           %u\n', strSHgravityData.ui32MaxDegree);
    fprintf('  Gravitational param:  %.12g\n', strSHgravityData.dGravParam);
    fprintf('  Density:              %.12g\n', strSHgravityData.dDensity);
    fprintf('  Reference radius:     %.12g\n', strSHgravityData.dBodyRadiusRef);
    fprintf('  Fit iterations:       %u\n', strSHgravityData.strFitStats.ui32NumIterations);
    fprintf('  Best iteration:       %u\n', strSHgravityData.strFitStats.ui32BestIteration);
    fprintf('  Converged:            %d\n', strSHgravityData.strFitStats.bConverged);
    fprintf('  Holdout acc RMS rel:  %.6e\n', strDiagnostics.strMetrics.dAccRMSrel);
    fprintf('  Holdout pot RMS rel:  %.6e\n', strDiagnostics.strMetrics.dPotentialRMSrel);
    fprintf('  Holdout acc max rel:  %.6e\n', strDiagnostics.strMetrics.dAccMaxRel);
    fprintf('  Holdout pot max rel:  %.6e\n', strDiagnostics.strMetrics.dPotentialMaxRel);
end

strRunOutputs = struct();
strRunOutputs.objShapeModel = objShapeModel;
strRunOutputs.strSHgravityData = strSHgravityData;
strRunOutputs.strDiagnostics = strDiagnostics;
strRunOutputs.strFigureHandles = strFigureHandles;

end
