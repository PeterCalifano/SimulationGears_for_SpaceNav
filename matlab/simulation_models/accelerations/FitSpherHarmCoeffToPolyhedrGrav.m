function strSHgravityData = FitSpherHarmCoeffToPolyhedrGrav( ...
                                            ui32FaceVertexIds, ...
                                            dVerticesPos, ...
                                            ui32MaxDegree, ...
                                            dGravParam, ...
                                            dDensity, ...
                                            dGravConst, ...
                                            dBodyRadiusRef, ...
                                            ui32MaxFitIterations)%#codegen
%% PROTOTYPE
% strSHgravityData = FitSpherHarmCoeffToPolyhedrGrav( ...
%     ui32FaceVertexIds, dVerticesPos, ui32MaxDegree, ...
%     dGravParam, dDensity, dGravConst, dBodyRadiusRef, ui32MaxFitIterations)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Fits exterior spherical harmonics coefficients from the exact
% constant-density polyhedron gravity field.
%
% The routine samples the exterior field on concentric shells, subtracts
% the central point-mass contribution, and fits the perturbative field in
% the repo-native ExtSHE coefficient format. Training samples are refined
% adaptively until the holdout error stops changing materially or the
% maximum number of fit iterations is reached.
%
% This is the polyhedron-specific sampling/adaptation layer. The low-level
% coefficient solve from already-sampled perturbative field values is owned
% by FitGravityFieldExtSHEcoefficients().
%
% ACHTUNG:
% 1) The input mesh must be triangulated, closed, and outward-wound.
% 2) The mesh is assumed to be centered at the expansion origin. The fit is
%    rejected if the uniform-density COM is not sufficiently close to the
%    origin because the active ExtSHE path does not model degree-1 terms.
% 3) The fitted coefficients approximate the EXTERIOR field only.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32FaceVertexIds:        [nFaces x 3]    Triangle vertex indices.
% dVerticesPos:             [nVertices x 3] Vertex coordinates [LU].
% ui32MaxDegree:            [1]             Maximum harmonic degree.
% dGravParam:               [1]             Gravitational parameter [LU^3/TU^2], or NaN to derive from density.
% dDensity:                 [1]             Density [MU/LU^3], or NaN to derive from dGravParam.
% dGravConst:               [1]             Gravitational constant [LU^3/(MU*TU^2)].
% dBodyRadiusRef:           [1]             Reference radius override [LU], or NaN to use enclosing radius.
% ui32MaxFitIterations:     [1]             Maximum adaptive fit iterations.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strSHgravityData: struct with fields
%   dCSlmCoeffCols   [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
%   ui32MaxDegree    [1]       Fitted maximum degree.
%   dGravParam       [1]       Gravitational parameter used by the fit.
%   dBodyRadiusRef   [1]       Reference radius used by the fit.
%   dDensity         [1]       Density used by the fit.
%   dGravConst       [1]       Gravitational constant used by the fit.
%   strFitStats      struct    Convergence history and mesh diagnostics.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Replace name-value options with positional inputs for codegen compatibility.
% 23-04-2026    Pietro Califano     Add adaptive polyhedron-to-SHE fitter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePolyhedronFaceEdgeData()
% EvalPolyhedronGrav()
% ComputeMeshModelVolumeAndCoM()
% FitGravityFieldExtSHEcoefficients()
% ComputeLogSpacedValues()
% GenerateShellPointSet()
% EvalPolyhedronGravPerturbationSamples()
% EvalExtSphHarmSamplesInTargetFrame()
% ComputeGravityFieldFitMetrics()
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Input validation
if ~isa(ui32FaceVertexIds, 'uint32') || size(ui32FaceVertexIds, 2) ~= 3
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidFaces', ...
        'ui32FaceVertexIds must be an nFaces-by-3 uint32 array.');
end

if ~isa(dVerticesPos, 'double') || size(dVerticesPos, 2) ~= 3
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidVertices', ...
        'dVerticesPos must be an nVertices-by-3 double array.');
end

if ~isa(ui32MaxDegree, 'uint32') || ~isscalar(ui32MaxDegree)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidMaxDegree', ...
        'ui32MaxDegree must be a uint32 scalar.');
end

if ui32MaxDegree < uint32(2)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidMaxDegree', ...
        'The maximum degree must be at least 2.');
end

if ~isscalar(dGravParam) || ~isa(dGravParam, 'double') || ~isreal(dGravParam)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidGravParam', ...
        'dGravParam must be a real double scalar or NaN.');
end

if ~isscalar(dDensity) || ~isa(dDensity, 'double') || ~isreal(dDensity)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidDensity', ...
        'dDensity must be a real double scalar or NaN.');
end

if ~isscalar(dGravConst) || ~isa(dGravConst, 'double') || ~isreal(dGravConst) || ...
        ~isfinite(dGravConst) || ~(dGravConst > 0.0)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidGravConst', ...
        'The gravitational constant must be a positive finite scalar.');
end

if ~isscalar(dBodyRadiusRef) || ~isa(dBodyRadiusRef, 'double') || ~isreal(dBodyRadiusRef)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidRefRadius', ...
        'dBodyRadiusRef must be a real double scalar or NaN.');
end

if ~isscalar(ui32MaxFitIterations) || ~isa(ui32MaxFitIterations, 'uint32') || ui32MaxFitIterations < uint32(1)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidMaxIterations', ...
        'The maximum number of fit iterations must be at least 1.');
end

bHasMu = isfinite(dGravParam);
bHasDensity = isfinite(dDensity);

if ~(bHasMu || bHasDensity)
    error('FitSpherHarmCoeffToPolyhedrGrav:MissingPhysicalInputs', ...
        'Specify at least one of dGravParam or dDensity.');
end

if bHasMu && ~(dGravParam > 0.0)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidGravParam', ...
        'dGravParam must be a positive finite scalar.');
end

if bHasDensity && ~(dDensity > 0.0)
    error('FitSpherHarmCoeffToPolyhedrGrav:InvalidDensity', ...
        'dDensity must be a positive finite scalar.');
end

% Compute polyhedron face and edge dyadics (required for gravity evaluation)
[ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData(ui32FaceVertexIds, dVerticesPos);

% Compute mesh volume, COM, and enclosing radius for consistency checks and fit configuration
[dVolume, dCOM] = ComputeMeshModelVolumeAndCoM(ui32FaceVertexIds, dVerticesPos);
dEnclosingRadius = max(vecnorm(dVerticesPos, 2, 2));

% Validate physical consistency of inputs and assign defaults
if isfinite(dBodyRadiusRef)
    if ~(dBodyRadiusRef > 0.0)
        error('FitSpherHarmCoeffToPolyhedrGrav:InvalidRefRadius', ...
            'dBodyRadiusRef must be a positive finite scalar when specified.');
    end
    if dBodyRadiusRef < dEnclosingRadius
        error('FitSpherHarmCoeffToPolyhedrGrav:InvalidRefRadius', ...
            'The reference radius override cannot be smaller than the enclosing radius.');
    end
else
    dBodyRadiusRef = dEnclosingRadius;
end

if norm(dCOM) > 1.0e-10 * dBodyRadiusRef
    error('FitSpherHarmCoeffToPolyhedrGrav:NonCenteredMesh', ...
        'The mesh COM must be centered at the expansion origin before fitting.');
end

% Derive any missing physical parameter and check consistency if both are provided
if bHasMu && bHasDensity

    dGravParamFromDensity = dGravConst * dDensity * dVolume;
    dRelMismatch = abs(dGravParamFromDensity - dGravParam) / max(abs(dGravParam), eps(dGravParam));
    
    if dRelMismatch > 1.0e-10
        error('FitSpherHarmCoeffToPolyhedrGrav:InconsistentMassInputs', ...
            'Provided dGravParam and dDensity are inconsistent with the mesh volume.');
    end

elseif bHasMu
    dDensity = dGravParam / (dGravConst * dVolume);
else
    dGravParam = dGravConst * dDensity * dVolume;
end

% Construct the SH gravity data struct with metadata and placeholders for the fit results
ui32NumUnknowns = ui32MaxDegree * ui32MaxDegree + 2 * ui32MaxDegree - uint32(3);
dTrainShellRadii = ComputeLogSpacedValues(1.05 * dBodyRadiusRef, 4.0 * dBodyRadiusRef, uint32(6));
dValidationShellRadii = ComputeLogSpacedValues(1.10 * dBodyRadiusRef, 3.5 * dBodyRadiusRef, uint32(5));

ui32TrainPtsPerShell = repmat(uint32(max(256, 3 * double(ui32NumUnknowns))), size(dTrainShellRadii));
ui32ValidationPtsPerShell = repmat(uint32(max(512, 6 * double(ui32NumUnknowns))), size(dValidationShellRadii));

% Build fixed validation samples once; each adaptive iteration reuses this
% holdout set to compare the fitted SHE field against the polyhedron field.
[dValidationPos_TB, ui32ValidationShellIds] = GenerateShellPointSet( ...
    dValidationShellRadii, ui32ValidationPtsPerShell, 0.25);

[dValidationPotentialPert, dValidationAccPertTB] = EvalPolyhedronGravPerturbationSamples( ...
    dValidationPos_TB, ui32FaceVertexIds, dVerticesPos, dDensity, ...
    ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst, dGravParam);

ui32MaxIter = ui32MaxFitIterations;

% Initialize fit history arrays and tracking variables
dCoeffRelChangeHistory = NaN(double(ui32MaxIter), 1);
dTrainAccRMSrelHistory = NaN(double(ui32MaxIter), 1);
dTrainPotRMSrelHistory = NaN(double(ui32MaxIter), 1);
dValidationAccRMSrelHistory = NaN(double(ui32MaxIter), 1);
dValidationPotRMSrelHistory = NaN(double(ui32MaxIter), 1);
dValidationAccMaxRelHistory = NaN(double(ui32MaxIter), 1);
dValidationPotMaxRelHistory = NaN(double(ui32MaxIter), 1);
dConditionNumberHistory = NaN(double(ui32MaxIter), 1);
ui32RankHistory = zeros(double(ui32MaxIter), 1, 'uint32');
bLeastNormHistory = false(double(ui32MaxIter), 1);
dTrainCountsHistory = zeros(double(ui32MaxIter), numel(ui32TrainPtsPerShell), 'uint32');
dValidationShellAccRMSrelHistory = NaN(double(ui32MaxIter), numel(dValidationShellRadii));
dValidationShellPotRMSrelHistory = NaN(double(ui32MaxIter), numel(dValidationShellRadii));

dBestMetric = inf;
dBestCoeffCols = [];
ui32BestIteration = uint32(0);
dPrevCoeffVector = [];
dPrevValAccRMSrel = NaN;
dPrevValPotRMSrel = NaN;
bConverged = false;

% Main adaptive fitting loop
for idIter = 1:double(ui32MaxIter)
    
    % Store the training counts for this iteration before any potential refinement
    dTrainCountsHistory(idIter, :) = ui32TrainPtsPerShell;

    % Generate training samples
    [dTrainPos_TB, ui32TrainShellIds] = GenerateShellPointSet( ...
        dTrainShellRadii, ui32TrainPtsPerShell, double(idIter));

    % Evaluate the perturbative polyhedron gravity field at the training points for this iteration
    [dTrainPotentialPert, dTrainAccPertTB] = EvalPolyhedronGravPerturbationSamples( ...
        dTrainPos_TB, ui32FaceVertexIds, dVerticesPos, dDensity, ...
        ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst, dGravParam);

    % Run the coefficient fit for this iteration
    [dCoeffColsIter, strSolveInfo] = FitGravityFieldExtSHEcoefficients( ...
        dTrainPos_TB, dTrainPotentialPert, dTrainAccPertTB, ...
        ui32MaxDegree, dGravParam, dBodyRadiusRef);

    % Evaluate the fitted field at the training and validation points for diagnostics
    [dTrainPotentialPred, dTrainAccPredTB] = EvalExtSphHarmSamplesInTargetFrame( ...
        dTrainPos_TB, ui32MaxDegree, dCoeffColsIter, dGravParam, dBodyRadiusRef);

    [dValidationPotentialPred, dValidationAccPredTB] = EvalExtSphHarmSamplesInTargetFrame( ...
        dValidationPos_TB, ui32MaxDegree, dCoeffColsIter, dGravParam, dBodyRadiusRef);

    % Compute error metrics for this iteration
    strTrainMetrics = ComputeGravityFieldFitMetrics( ...
        dTrainPotentialPred, dTrainPotentialPert, dTrainAccPredTB, dTrainAccPertTB, ...
        ui32TrainShellIds, uint32(numel(dTrainShellRadii)));
        
    strValidationMetrics = ComputeGravityFieldFitMetrics( ...
        dValidationPotentialPred, dValidationPotentialPert, dValidationAccPredTB, dValidationAccPertTB, ...
        ui32ValidationShellIds, uint32(numel(dValidationShellRadii)));

    % Compute coefficient change relative to the previous iteration for convergence monitoring
    dCoeffVectorIter = PackActiveCoeffVector(dCoeffColsIter, ui32MaxDegree);
    if isempty(dPrevCoeffVector)
        dCoeffRelChange = inf;
    else
        dCoeffRelChange = norm(dCoeffVectorIter - dPrevCoeffVector) / max(norm(dCoeffVectorIter), eps(norm(dCoeffVectorIter)));
    end

    % Store iteration diagnostics
    dCoeffRelChangeHistory(idIter) = dCoeffRelChange;
    dTrainAccRMSrelHistory(idIter) = strTrainMetrics.dAccRMSrel;
    dTrainPotRMSrelHistory(idIter) = strTrainMetrics.dPotentialRMSrel;
    dValidationAccRMSrelHistory(idIter) = strValidationMetrics.dAccRMSrel;
    dValidationPotRMSrelHistory(idIter) = strValidationMetrics.dPotentialRMSrel;
    dValidationAccMaxRelHistory(idIter) = strValidationMetrics.dAccMaxRel;
    dValidationPotMaxRelHistory(idIter) = strValidationMetrics.dPotentialMaxRel;
    dConditionNumberHistory(idIter) = strSolveInfo.dConditionNumber;
    ui32RankHistory(idIter) = strSolveInfo.ui32Rank;
    bLeastNormHistory(idIter) = strSolveInfo.bUsedLeastNorm;
    dValidationShellAccRMSrelHistory(idIter, :) = strValidationMetrics.dAccShellRMSrel;
    dValidationShellPotRMSrelHistory(idIter, :) = strValidationMetrics.dPotentialShellRMSrel;

    % Compute combined relative error metric (normalized) for this iteration and check for improvement
    dCurrentMetric = strValidationMetrics.dAccRMSrel + strValidationMetrics.dPotentialRMSrel;
    if dCurrentMetric < dBestMetric
        dBestMetric = dCurrentMetric;
        dBestCoeffCols = dCoeffColsIter;
        ui32BestIteration = uint32(idIter);
    end

    % Check for convergence based on coefficient change and validation error improvement
    if idIter >= 2
        dAccImprovement = abs(strValidationMetrics.dAccRMSrel - dPrevValAccRMSrel) / max(dPrevValAccRMSrel, eps(dPrevValAccRMSrel));
        dPotImprovement = abs(strValidationMetrics.dPotentialRMSrel - dPrevValPotRMSrel) / max(dPrevValPotRMSrel, eps(dPrevValPotRMSrel));

        if dCoeffRelChange < 1.0e-8 && dAccImprovement < 5.0e-3 && dPotImprovement < 5.0e-3
            bConverged = true;
            break;
        end
    end

    dPrevCoeffVector = dCoeffVectorIter;
    dPrevValAccRMSrel = strValidationMetrics.dAccRMSrel;
    dPrevValPotRMSrel = strValidationMetrics.dPotentialRMSrel;

    if idIter < double(ui32MaxIter)
        ui32TrainPtsPerShell = RefineTrainingShellCounts( ...
            ui32TrainPtsPerShell, dTrainShellRadii, dValidationShellRadii, ...
            strValidationMetrics.dAccShellRMSrel, ...
            strValidationMetrics.dPotentialShellRMSrel);
    end
end

if isempty(dBestCoeffCols)
    error('FitSpherHarmCoeffToPolyhedrGrav:FitFailed', ...
        'Unable to compute a valid spherical harmonics fit.');
end

ui32NumCompletedIterations = uint32(find(~isnan(dValidationAccRMSrelHistory), 1, 'last'));

% Compile fit statistics and results into the output struct
strFitStats = struct();
strFitStats.dVolume = dVolume;
strFitStats.dCOM = dCOM;
strFitStats.dEnclosingRadius = dEnclosingRadius;
strFitStats.dTrainShellRadii = dTrainShellRadii;
strFitStats.dValidationShellRadii = dValidationShellRadii;
strFitStats.ui32ValidationPtsPerShell = ui32ValidationPtsPerShell;
strFitStats.ui32NumUnknowns = ui32NumUnknowns;
strFitStats.ui32NumIterations = ui32NumCompletedIterations;
strFitStats.ui32BestIteration = ui32BestIteration;
strFitStats.bConverged = bConverged;
strFitStats.dCoeffRelChangeHistory = dCoeffRelChangeHistory(1:double(ui32NumCompletedIterations));
strFitStats.dTrainAccRMSrelHistory = dTrainAccRMSrelHistory(1:double(ui32NumCompletedIterations));
strFitStats.dTrainPotRMSrelHistory = dTrainPotRMSrelHistory(1:double(ui32NumCompletedIterations));
strFitStats.dValidationAccRMSrelHistory = dValidationAccRMSrelHistory(1:double(ui32NumCompletedIterations));
strFitStats.dValidationPotRMSrelHistory = dValidationPotRMSrelHistory(1:double(ui32NumCompletedIterations));
strFitStats.dValidationAccMaxRelHistory = dValidationAccMaxRelHistory(1:double(ui32NumCompletedIterations));
strFitStats.dValidationPotMaxRelHistory = dValidationPotMaxRelHistory(1:double(ui32NumCompletedIterations));
strFitStats.dConditionNumberHistory = dConditionNumberHistory(1:double(ui32NumCompletedIterations));
strFitStats.ui32RankHistory = ui32RankHistory(1:double(ui32NumCompletedIterations));
strFitStats.bLeastNormHistory = bLeastNormHistory(1:double(ui32NumCompletedIterations));
strFitStats.ui32TrainPtsPerShellHistory = dTrainCountsHistory(1:double(ui32NumCompletedIterations), :);
strFitStats.dValidationShellAccRMSrelHistory = dValidationShellAccRMSrelHistory(1:double(ui32NumCompletedIterations), :);
strFitStats.dValidationShellPotRMSrelHistory = dValidationShellPotRMSrelHistory(1:double(ui32NumCompletedIterations), :);

strSHgravityData = struct();
strSHgravityData.dCSlmCoeffCols = dBestCoeffCols;
strSHgravityData.ui32MaxDegree = ui32MaxDegree;
strSHgravityData.dGravParam = dGravParam;
strSHgravityData.dBodyRadiusRef = dBodyRadiusRef;
strSHgravityData.dDensity = dDensity;
strSHgravityData.dGravConst = dGravConst;
strSHgravityData.strFitStats = strFitStats;

end

%% Internal helper functions
function ui32TrainPtsPerShell = RefineTrainingShellCounts(ui32TrainPtsPerShell, dTrainShellRadii, dValidationShellRadii, ...
                dAccShellRMSrel, dPotShellRMSrel)

dMetricMin = min([dAccShellRMSrel(:); dPotShellRMSrel(:)]);
dMetricMax = max([dAccShellRMSrel(:); dPotShellRMSrel(:)]);

if ~isfinite(dMetricMin) || ~isfinite(dMetricMax)
    ui32TrainPtsPerShell = 2 * ui32TrainPtsPerShell;
    return;
end

if dMetricMax <= 1.25 * max(dMetricMin, eps)
    ui32TrainPtsPerShell = 2 * ui32TrainPtsPerShell;
    return;
end

% Identify shells with the worst relative errors in acceleration or potential and refine those
dSortedAccMetric = sort(dAccShellRMSrel);
dSortedPotMetric = sort(dPotShellRMSrel);
idThreshold = max(1, ceil(0.7 * numel(dSortedAccMetric)));
dAccThreshold = dSortedAccMetric(idThreshold);
dPotThreshold = dSortedPotMetric(idThreshold);
idWorstValShells = find(dAccShellRMSrel >= dAccThreshold | dPotShellRMSrel >= dPotThreshold);

% Refine the training point counts for shells with the worst errors, ensuring we stay within reasonable limits
for idWorst = reshape(idWorstValShells, 1, [])
    
    [~, idNearestTrainShell] = min(abs(dTrainShellRadii - dValidationShellRadii(idWorst)));
    idTrainShellsToRefine = unique(max(1, idNearestTrainShell - 1):min(numel(ui32TrainPtsPerShell), idNearestTrainShell + 1));
    
    ui32TrainPtsPerShell(idTrainShellsToRefine) = 2 * ui32TrainPtsPerShell(idTrainShellsToRefine);
end

end

function dCoeffVector = PackActiveCoeffVector(dCSlmCoeffCols, ui32MaxDegree)
% Pack the active coefficients from the [Clm, Slm] column format into a single vector for convergence monitoring.
ui32NumUnknowns = ui32MaxDegree * ui32MaxDegree + 2 * ui32MaxDegree - uint32(3);
dCoeffVector = zeros(double(ui32NumUnknowns), 1);

idPair = uint32(1);
idUnknown = uint32(1);

for idxDeg = uint32(1):ui32MaxDegree
    for idxOrd = uint32(0):idxDeg
        if idxDeg == uint32(1) && idxOrd == uint32(0)
            continue;
        end

        if idxDeg >= uint32(2)
            dCoeffVector(idUnknown) = dCSlmCoeffCols(idPair, 1);
            idUnknown = idUnknown + uint32(1);

            if idxOrd > uint32(0)
                dCoeffVector(idUnknown) = dCSlmCoeffCols(idPair, 2);
                idUnknown = idUnknown + uint32(1);
            end
        end

        idPair = idPair + uint32(1);
    end
end
end
