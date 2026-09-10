function strComparison = DemoItokawaDegree16GravityComparison( ...
    charAlbanCoeffFilePath, charDataRootPath, options)
%% SIGNATURE
% strComparison = DemoItokawaDegree16GravityComparison( ...
%     charAlbanCoeffFilePath, charDataRootPath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate a degree-16 exterior spherical-harmonics gravity model from the
% registry-backed Itokawa DSK and compare it with two existing coefficient
% families:
%   1. A demo-local Scheeres et al. degree-4 reference.
%   2. The legacy Alban degree-16 normalized text file used by COSMICA.
%
% The mesh is translated to its uniform-density volume centroid before the
% fit. CShapeModel fits at the mesh enclosing radius, after which the
% generated coefficients are rescaled degree by degree to the registered
% fitted-family reference radius. Coefficient scores use physical degrees
% 2-4 against the historical Scheeres reference. A common held-out point set
% compares all models with the full polyhedron field. The registry itself
% remains authoritative for the selected hardcoded degree-16 runtime model.
%
% Example:
%   strComparison = DemoItokawaDegree16GravityComparison();
%   disp(strComparison.charSelectedModel)
%
% Expected output:
%   The default full-mesh run prints coefficient and held-out field metrics,
%   followed by the registered decision "generated_degree16". The returned
%   struct contains the models, mesh metadata, timings, and comparison
%   evidence.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charAlbanCoeffFilePath       Legacy normalized degree-16 coefficient file.
%                               Empty resolves the sibling COSMICA checkout.
% charDataRootPath             SimulationGears data root. Empty uses the
%                               repository/default SIMGEARS data resolution.
% options.ui32MaxFitIterations Maximum adaptive CShapeModel fit iterations.
% options.ui32HoldoutPtsPerShell
%                              Number of full-polyhedron validation points on
%                               each of five independent exterior shells.
% options.dMeshSimplifyFactor  Mesh keep fraction. The default 1.0 uses all
%                               registry DSK faces and is required for the
%                               model-selection result.
% options.charOutputMatFilePath
%                              Optional MAT path for the complete comparison.
% options.bVerbose             Print progress and comparison metrics.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strComparison               Generated and source models, coefficient and
%                              field metrics, selection, mesh data, and timing.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-07-2026  Pietro Califano, Codex     First full-mesh Itokawa degree-16 comparison example.
% 24-07-2026  Pietro Califano, Codex     Keep Scheeres as a demo-local comparison reference.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry
% CShapeModel
% ComputeMeshModelVolumeAndCoM
% EvalExtSphHarmSamplesInTargetFrame
% EvalPolyhedronGravPerturbationSamples
% GenerateShellPointSet
% GetSphHarmNormalizationFactors
% RescaleSphericalHarmonicsReferenceRadius
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charAlbanCoeffFilePath (1, :) char = ''
    charDataRootPath (1, :) char = ''
    options.ui32MaxFitIterations (1, 1) uint32 = uint32(1)
    options.ui32HoldoutPtsPerShell (1, 1) uint32 = uint32(128)
    options.dMeshSimplifyFactor (1, 1) double {mustBeGreaterThan(options.dMeshSimplifyFactor, 0.0), ...
        mustBeLessThanOrEqual(options.dMeshSimplifyFactor, 1.0)} = 1.0
    options.charOutputMatFilePath (1, :) char = ''
    options.bVerbose (1, 1) logical = true
end

arguments (Output)
    strComparison (1, 1) struct
end

charExampleDirectory = fileparts(mfilename('fullpath'));
charRepositoryRoot = fileparts(fileparts(fileparts(charExampleDirectory)));
run(fullfile(charRepositoryRoot, 'matlab', 'SetupSimGears.m'));

% Resolve the historical comparison source only for this cross-repository
% manual experiment; production SimulationGears does not depend on COSMICA.
if isempty(charAlbanCoeffFilePath)
    charDevRoot = fileparts(charRepositoryRoot);
    charAlbanCoeffFilePath = fullfile(charDevRoot, 'projects-DART', ...
        'cosmica-simulator', 'data', 'kernels', 'Itokawa', ...
        'shcoeff_itokawa_16x16.txt');
end
mustBeFile(charAlbanCoeffFilePath);

ui32MaxDegree = uint32(16);
[strRegisteredModel, strRegisteredMetadata] = ...
    CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
        "Itokawa", ui32MaxDegree, "km");
if isempty(strRegisteredModel.dCSlmCoeffCols)
    error('DemoItokawaGravity:MissingRegisteredData', ...
        'The SimulationGears registry did not return Itokawa degree-16 data.');
end
strScheeresModel = BuildScheeresReference_( ...
    strRegisteredModel.dGravParam, ...
    strRegisteredModel.dBodyRadiusRef, ...
    strRegisteredModel.dGravConst);
strScheeresMetadata = strScheeresModel.strFitStats;

% Load the selected registry mesh and compute the centroid translation using
% the same full-resolution geometry that will drive the gravity fit.
[objShapeModel, ~, strShapeMetadata] = DefineShapeModel( ...
    "Itokawa", charDataRootPath, ...
    charOutputLengthUnits="km", ...
    dMeshSimplifyFactor=options.dMeshSimplifyFactor, ...
    bInitSphericalHarmonicsGravityData=false);
strShapeData = objShapeModel.getShapeStruct();
ui32FacesRows = uint32(strShapeData.ui32triangVertexPtr');
dVerticesRows = strShapeData.dVerticesPos';
[dVolumeBeforeCentering, dMeshCOM] = ComputeMeshModelVolumeAndCoM( ...
    ui32FacesRows, dVerticesRows);

strCenteredShapeData = strShapeData;
strCenteredShapeData.dVerticesPos = ...
    strShapeData.dVerticesPos - dMeshCOM;
objCenteredShape = CShapeModel( ...
    "struct", strCenteredShapeData, "km", "km", true, ...
    "Itokawa centered for gravity fit", true);
strCenteredShapeData = objCenteredShape.getShapeStruct();
ui32CenteredFaces = uint32(strCenteredShapeData.ui32triangVertexPtr');
dCenteredVertices = strCenteredShapeData.dVerticesPos';
[dCenteredVolume, dCenteredCOM] = ComputeMeshModelVolumeAndCoM( ...
    ui32CenteredFaces, dCenteredVertices);
dEnclosingRadius = max(vecnorm(dCenteredVertices, 2, 2));

ui32NumFaces = uint32(size(ui32CenteredFaces, 1));
ui32NumVertices = uint32(size(dCenteredVertices, 1));
if options.bVerbose
    fprintf('\nItokawa degree-16 gravity comparison\n');
    fprintf('  Mesh faces:             %u\n', ui32NumFaces);
    fprintf('  Mesh vertices:          %u\n', ui32NumVertices);
    fprintf('  Mesh keep fraction:     %.6f\n', options.dMeshSimplifyFactor);
    fprintf('  Original volume COM:    [%+.12e %+.12e %+.12e] km\n', dMeshCOM);
    fprintf('  Centered COM norm:      %.6e km\n', norm(dCenteredCOM));
    fprintf('  Enclosing radius:       %.12g km\n', dEnclosingRadius);
    fprintf('  Fit iterations allowed: %u\n', options.ui32MaxFitIterations);
end

% Fit the exterior field through the CShapeModel-owned utility. NaN selects
% the enclosing radius, which is required because the registered reference radius
% lies inside the irregular body.
ui64FitTimer = tic;
strGeneratedAtFitRadius = ...
    CShapeModel.BuildSphericalHarmonicsGravityData( ...
        objCenteredShape, ui32MaxDegree, ...
        dGravParam=strRegisteredModel.dGravParam, ...
        dDensity=NaN, ...
        dGravConst=strRegisteredModel.dGravConst, ...
        dBodyRadiusRef=NaN, ...
        ui32MaxFitIterations=options.ui32MaxFitIterations);
dFitElapsedSec = toc(ui64FitTimer);

% Put the generated and legacy candidates on the same unnormalized
% coefficient convention, radius, GM, and row ordering as the registered
% fitted family.
strGeneratedModel = RescaleSphericalHarmonicsReferenceRadius( ...
    strGeneratedAtFitRadius, strRegisteredModel.dBodyRadiusRef);
strAlbanModel = LoadAlbanModel_(charAlbanCoeffFilePath, ui32MaxDegree);
AssertMatchedScalar_(strAlbanModel.dGravParam, ...
    strRegisteredModel.dGravParam, 'gravitational parameter');
AssertMatchedScalar_(strAlbanModel.dBodyRadiusRef, ...
    strRegisteredModel.dBodyRadiusRef, 'reference radius');

% Score only physical coefficients in the overlap with the published
% degree-4 source; degree-1 compatibility entries are reported separately.
strGeneratedToScheeres = CompareCoefficientRange_( ...
    strGeneratedModel, strScheeresModel, uint32(2), uint32(4));
strAlbanToScheeres = CompareCoefficientRange_( ...
    strAlbanModel, strScheeresModel, uint32(2), uint32(4));
strGeneratedToAlban = CompareCoefficientRange_( ...
    strGeneratedModel, strAlbanModel, uint32(2), uint32(4));

% Evaluate the exact full-polyhedron field once on an independent set of
% shells, then reuse those samples for all three SH candidates.
dHoldoutRadiusScale = [1.12, 1.31, 1.67, 2.19, 3.17];
dHoldoutShellRadii = dEnclosingRadius .* dHoldoutRadiusScale;
ui32HoldoutCounts = repmat(options.ui32HoldoutPtsPerShell, ...
    size(dHoldoutShellRadii));
[dHoldoutPos_TB, ui32HoldoutShellIds] = GenerateShellPointSet( ...
    dHoldoutShellRadii, ui32HoldoutCounts, 7.25);

objCenteredShape = objCenteredShape.BuildPolyhedronGravityData();
[ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, ...
    ui32FacesRows, dVerticesRows] = ...
    objCenteredShape.getPolyhedronGravityData();

ui64HoldoutTimer = tic;
[dPolyPotentialPert, dPolyAccPertTB] = ...
    EvalPolyhedronGravPerturbationSamples( ...
        dHoldoutPos_TB, ui32FacesRows, dVerticesRows, ...
        strGeneratedAtFitRadius.dDensity, ui32EdgeVertexIds, ...
        dEdgeDyadics, dFaceDyadics, ...
        strGeneratedAtFitRadius.dGravConst, ...
        strGeneratedAtFitRadius.dGravParam);
dPolyhedronElapsedSec = toc(ui64HoldoutTimer);

strGeneratedFieldMetrics = EvaluateModelAgainstPolyhedron_( ...
    strGeneratedModel, dHoldoutPos_TB, dPolyPotentialPert, ...
    dPolyAccPertTB, ui32HoldoutShellIds, ...
    uint32(numel(dHoldoutShellRadii)));
strAlbanFieldMetrics = EvaluateModelAgainstPolyhedron_( ...
    strAlbanModel, dHoldoutPos_TB, dPolyPotentialPert, ...
    dPolyAccPertTB, ui32HoldoutShellIds, ...
    uint32(numel(dHoldoutShellRadii)));
strScheeresFieldMetrics = EvaluateModelAgainstPolyhedron_( ...
    strScheeresModel, dHoldoutPos_TB, dPolyPotentialPert, ...
    dPolyAccPertTB, ui32HoldoutShellIds, ...
    uint32(numel(dHoldoutShellRadii)));

% Record the explicit project decision separately from the metrics so this
% reproducibility demo cannot silently change the runtime registry policy.
charSelectedModel = 'generated_degree16';
strSelectionEvidence = struct( ...
    'bCloserToScheeresDegree4', ...
        strGeneratedToScheeres.dRelativeL2 < ...
        strAlbanToScheeres.dRelativeL2, ...
    'bLowerHoldoutAccelerationRMS', ...
        strGeneratedFieldMetrics.dAccRMSrel < ...
        strAlbanFieldMetrics.dAccRMSrel);

% Preserve enough evidence to reproduce the selection and inspect the fit
% without requiring another full-polyhedron execution.
strComparison = struct();
strComparison.charSelectedModel = charSelectedModel;
strComparison.strSelectionEvidence = strSelectionEvidence;
strComparison.strMesh = struct( ...
    'ui32NumFaces', ui32NumFaces, ...
    'ui32NumVertices', ui32NumVertices, ...
    'dMeshSimplifyFactor', options.dMeshSimplifyFactor, ...
    'dVolumeBeforeCentering', dVolumeBeforeCentering, ...
    'dCenteredVolume', dCenteredVolume, ...
    'dOriginalCOM', dMeshCOM, ...
    'dCenteredCOM', dCenteredCOM, ...
    'dEnclosingRadius', dEnclosingRadius, ...
    'strRegistryMetadata', strShapeMetadata);
strComparison.strGeneratedAtFitRadius = strGeneratedAtFitRadius;
strComparison.strGeneratedModel = strGeneratedModel;
strComparison.strRegisteredModel = strRegisteredModel;
strComparison.strRegisteredMetadata = strRegisteredMetadata;
strComparison.strAlbanModel = strAlbanModel;
strComparison.strScheeresModel = strScheeresModel;
strComparison.strScheeresMetadata = strScheeresMetadata;
strComparison.strCoefficientMetrics = struct( ...
    'strGeneratedToScheeres', strGeneratedToScheeres, ...
    'strAlbanToScheeres', strAlbanToScheeres, ...
    'strGeneratedToAlban', strGeneratedToAlban, ...
    'dAlbanDegree1Coeff', strAlbanModel.dCSlmCoeffCols(1, :), ...
    'dGeneratedDegree1Coeff', strGeneratedModel.dCSlmCoeffCols(1, :));
strComparison.strFieldMetrics = struct( ...
    'strGenerated', strGeneratedFieldMetrics, ...
    'strAlban', strAlbanFieldMetrics, ...
    'strScheeresDegree4', strScheeresFieldMetrics, ...
    'dHoldoutShellRadii', dHoldoutShellRadii, ...
    'ui32PointsPerShell', options.ui32HoldoutPtsPerShell);
strComparison.strTiming = struct( ...
    'dFitElapsedSec', dFitElapsedSec, ...
    'dPolyhedronHoldoutElapsedSec', dPolyhedronElapsedSec);
strComparison.charAlbanCoeffFilePath = charAlbanCoeffFilePath;

PrintComparison_(strComparison, options.bVerbose);

if ~isempty(options.charOutputMatFilePath)
    save(options.charOutputMatFilePath, 'strComparison', '-v7.3');
end

end

function strScheeresModel = BuildScheeresReference_( ...
        dGravParam, dBodyRadiusRef, dGravConst)
% Build the historical degree-4 reference used only by this comparison.
strScheeresModel = struct();
strScheeresModel.dCSlmCoeffCols = [ ...
    0.00000000000000000e+00, 0.00000000000000000e+00; ...
    -3.24712847420609518e-01, 0.00000000000000000e+00; ...
    0.00000000000000000e+00, 0.00000000000000000e+00; ...
    1.41635000970805225e-01, 0.00000000000000000e+00; ...
    9.55513085990977035e-02, 0.00000000000000000e+00; ...
    -3.03935937520831319e-02, -6.62871761102150694e-03; ...
    -1.60173503072969792e-02, -1.60173503072969792e-02; ...
    9.62465805857249355e-03, 4.73772684358695836e-03; ...
    2.63556000000000012e-01, 0.00000000000000000e+00; ...
    3.23206912812829539e-02, 4.62008766150600204e-03; ...
    -2.75624447110556572e-02, 2.19134661794979365e-05; ...
    -1.83306235670569265e-03, -8.97975254192929910e-04; ...
    3.17528679272777300e-03, 2.45665212993211556e-04];
strScheeresModel.ui32MaxDegree = uint32(4);
strScheeresModel.dGravParam = dGravParam;
strScheeresModel.dBodyRadiusRef = dBodyRadiusRef;
strScheeresModel.dDensity = NaN;
strScheeresModel.dGravConst = dGravConst;
strScheeresModel.strFitStats = struct( ...
    'charSource', 'Scheeres et al. 2006 Itokawa degree-4 gravity coefficients', ...
    'charSourceUrl', 'https://doi.org/10.2514/1.19464', ...
    'charNormalization', ...
        'Scheeres normalized coefficients converted with GetSphHarmNormalizationFactors', ...
    'charFieldKind', 'published');
end

function strAlbanModel = LoadAlbanModel_(charFilePath, ui32MaxDegree)
% Load the legacy normalized COSMICA file into SimulationGears coefficient rows.
dFileId = fopen(charFilePath, 'r');
if dFileId < 0
    error('DemoItokawaGravity:AlbanOpenFailed', ...
        'Unable to open legacy coefficient file: %s', charFilePath);
end
objFileCleanup = onCleanup(@() fclose(dFileId)); %#ok<NASGU>

charHeaderLine = fgetl(dFileId);
dHeaderValues = sscanf(strrep(charHeaderLine, ',', ' '), '%f').';
if numel(dHeaderValues) < 6
    error('DemoItokawaGravity:InvalidAlbanHeader', ...
        'Legacy coefficient header must contain six numeric values.');
end

cellColumns = textscan(dFileId, '%f%f%f%f', ...
    'Delimiter', ',', 'CollectOutput', true);
dFileRows = cellColumns{1};
if isempty(dFileRows) || size(dFileRows, 2) ~= 4
    error('DemoItokawaGravity:InvalidAlbanRows', ...
        'Legacy coefficient file contains no degree/order rows.');
end

ui32AvailableDegree = uint32(dHeaderValues(4));
if ui32MaxDegree > ui32AvailableDegree
    error('DemoItokawaGravity:AlbanDegreeUnavailable', ...
        'Requested degree %u exceeds legacy file degree %u.', ...
        ui32MaxDegree, ui32AvailableDegree);
end

[ui32DegreeIds, ui32OrderIds] = BuildCoeffIds_(ui32MaxDegree);
dCoeffRows = zeros(numel(ui32DegreeIds), 2);
bAssignedRows = false(numel(ui32DegreeIds), 1);
for dFileRowIndex = 1:size(dFileRows, 1)
    ui32Degree = uint32(dFileRows(dFileRowIndex, 1));
    ui32Order = uint32(dFileRows(dFileRowIndex, 2));
    if ui32Degree > ui32MaxDegree || ...
            (ui32Degree == uint32(1) && ui32Order == uint32(0))
        continue
    end

    dCoeffRowIndex = find(ui32DegreeIds == ui32Degree & ...
        ui32OrderIds == ui32Order, 1);
    if ~isempty(dCoeffRowIndex)
        dCoeffRows(dCoeffRowIndex, :) = ...
            dFileRows(dFileRowIndex, 3:4);
        bAssignedRows(dCoeffRowIndex) = true;
    end
end

if ~all(bAssignedRows)
    error('DemoItokawaGravity:IncompleteAlbanRows', ...
        'Legacy coefficient file is missing one or more required rows.');
end

% The legacy header flag 1 denotes normalized coefficients; divide by the
% canonical scale factors to obtain SimulationGears' unnormalized storage.
dNormalizationFlag = dHeaderValues(6);
if dNormalizationFlag == 1.0
    dScaleFactors = GetSphHarmNormalizationFactors( ...
        double(ui32MaxDegree));
    dCoeffRows = dCoeffRows ./ dScaleFactors;
elseif dNormalizationFlag ~= 0.0
    error('DemoItokawaGravity:UnsupportedNormalization', ...
        'Unsupported legacy normalization flag %.12g.', ...
        dNormalizationFlag);
end

strAlbanModel = struct();
strAlbanModel.dCSlmCoeffCols = dCoeffRows;
strAlbanModel.ui32MaxDegree = ui32MaxDegree;
strAlbanModel.dGravParam = dHeaderValues(2);
strAlbanModel.dBodyRadiusRef = dHeaderValues(1);
strAlbanModel.dDensity = NaN;
strAlbanModel.dGravConst = 6.67430e-20;
strAlbanModel.strFitStats = struct( ...
    'charSource', 'Legacy COSMICA Alban degree-16 text file', ...
    'charSourceFile', charFilePath, ...
    'dInputNormalizationFlag', dNormalizationFlag);
end

function strMetrics = CompareCoefficientRange_( ...
    strCandidate, strReference, ui32MinDegree, ui32MaxDegree)
% Compare active C_lm and S_lm terms over one common degree interval.
dCandidateVector = PackCoeffRange_( ...
    strCandidate.dCSlmCoeffCols, strCandidate.ui32MaxDegree, ...
    ui32MinDegree, ui32MaxDegree);
dReferenceVector = PackCoeffRange_( ...
    strReference.dCSlmCoeffCols, strReference.ui32MaxDegree, ...
    ui32MinDegree, ui32MaxDegree);
dDifference = dCandidateVector - dReferenceVector;

strMetrics = struct();
strMetrics.ui32MinDegree = ui32MinDegree;
strMetrics.ui32MaxDegree = ui32MaxDegree;
strMetrics.ui32NumTerms = uint32(numel(dDifference));
strMetrics.dRelativeL2 = norm(dDifference) / ...
    max(norm(dReferenceVector), eps(1.0));
strMetrics.dRMSabsolute = sqrt(mean(dDifference .^ 2));
strMetrics.dMaxAbsolute = max(abs(dDifference));
end

function dCoeffVector = PackCoeffRange_( ...
    dCoeffRows, ui32ModelMaxDegree, ui32MinDegree, ui32MaxDegree)
% Pack C_lm and only physically active S_lm terms for selected degrees.
if ui32MaxDegree > ui32ModelMaxDegree
    error('DemoItokawaGravity:CoefficientRangeUnavailable', ...
        'Requested degree %u exceeds model degree %u.', ...
        ui32MaxDegree, ui32ModelMaxDegree);
end

[ui32DegreeIds, ui32OrderIds] = BuildCoeffIds_(ui32ModelMaxDegree);
bSelectedRows = ui32DegreeIds >= ui32MinDegree & ...
    ui32DegreeIds <= ui32MaxDegree;
dSelectedRows = find(bSelectedRows);
dNumSineTerms = sum(ui32OrderIds(dSelectedRows) > uint32(0));
dCoeffVector = zeros(numel(dSelectedRows) + dNumSineTerms, 1);

dOutputIndex = 1;
for dSelectedRowIndex = reshape(dSelectedRows, 1, [])
    dCoeffVector(dOutputIndex) = dCoeffRows(dSelectedRowIndex, 1);
    dOutputIndex = dOutputIndex + 1;
    if ui32OrderIds(dSelectedRowIndex) > uint32(0)
        dCoeffVector(dOutputIndex) = dCoeffRows(dSelectedRowIndex, 2);
        dOutputIndex = dOutputIndex + 1;
    end
end
end

function [ui32DegreeIds, ui32OrderIds] = BuildCoeffIds_(ui32MaxDegree)
% Build degree/order identifiers for the canonical SimulationGears row order.
ui32NumRows = (ui32MaxDegree + uint32(1)) * ...
    (ui32MaxDegree + uint32(2)) / uint32(2) - uint32(2);
ui32DegreeIds = zeros(double(ui32NumRows), 1, 'uint32');
ui32OrderIds = zeros(double(ui32NumRows), 1, 'uint32');
ui32DegreeIds(1) = uint32(1);
ui32OrderIds(1) = uint32(1);

ui32Row = uint32(2);
for ui32Degree = uint32(2):ui32MaxDegree
    for ui32Order = uint32(0):ui32Degree
        ui32DegreeIds(ui32Row) = ui32Degree;
        ui32OrderIds(ui32Row) = ui32Order;
        ui32Row = ui32Row + uint32(1);
    end
end
end

function strMetrics = EvaluateModelAgainstPolyhedron_( ...
    strModel, dSamplePos_TB, dTruePotential, dTrueAccTB, ...
    ui32ShellIds, ui32NumShells)
% Evaluate one SH model on the shared holdout set and return common metrics.
[dPredictedPotential, dPredictedAccTB] = ...
    EvalExtSphHarmSamplesInTargetFrame( ...
        dSamplePos_TB, strModel.ui32MaxDegree, ...
        strModel.dCSlmCoeffCols, strModel.dGravParam, ...
        strModel.dBodyRadiusRef);
strMetrics = ComputeGravityFieldFitMetrics( ...
    dPredictedPotential, dTruePotential, ...
    dPredictedAccTB, dTrueAccTB, ...
    ui32ShellIds, ui32NumShells);
end

function AssertMatchedScalar_(dActual, dExpected, charQuantity)
% Reject source comparisons whose dimensional metadata do not match.
dRelativeDifference = abs(dActual - dExpected) / ...
    max(abs(dExpected), eps(1.0));
if dRelativeDifference > 1.0e-12
    error('DemoItokawaGravity:MetadataMismatch', ...
        'Legacy %s %.17g does not match the registered model %.17g.', ...
        charQuantity, dActual, dExpected);
end
end

function PrintComparison_(strComparison, bVerbose)
% Print the concise evidence table used for the model-family decision.
if ~bVerbose
    return
end

strCoeff = strComparison.strCoefficientMetrics;
strField = strComparison.strFieldMetrics;
fprintf('\nCoefficient agreement with Scheeres, physical degrees 2-4\n');
fprintf('  Model                  Relative L2       RMS absolute      Max absolute\n');
fprintf('  Generated              %14.6e   %14.6e   %14.6e\n', ...
    strCoeff.strGeneratedToScheeres.dRelativeL2, ...
    strCoeff.strGeneratedToScheeres.dRMSabsolute, ...
    strCoeff.strGeneratedToScheeres.dMaxAbsolute);
fprintf('  Alban                  %14.6e   %14.6e   %14.6e\n', ...
    strCoeff.strAlbanToScheeres.dRelativeL2, ...
    strCoeff.strAlbanToScheeres.dRMSabsolute, ...
    strCoeff.strAlbanToScheeres.dMaxAbsolute);
fprintf('  Alban degree-1 row:    [%+.6e %+.6e]\n', ...
    strCoeff.dAlbanDegree1Coeff);

fprintf('\nHeld-out perturbative acceleration agreement with full polyhedron\n');
fprintf('  Model                  Relative RMS      Maximum relative\n');
fprintf('  Generated              %14.6e   %14.6e\n', ...
    strField.strGenerated.dAccRMSrel, ...
    strField.strGenerated.dAccMaxRel);
fprintf('  Alban                  %14.6e   %14.6e\n', ...
    strField.strAlban.dAccRMSrel, ...
    strField.strAlban.dAccMaxRel);
fprintf('  Scheeres degree 4      %14.6e   %14.6e\n', ...
    strField.strScheeresDegree4.dAccRMSrel, ...
    strField.strScheeresDegree4.dAccMaxRel);

fprintf('\nRuntime\n');
fprintf('  Degree-16 fit:         %.3f s\n', ...
    strComparison.strTiming.dFitElapsedSec);
fprintf('  Polyhedron holdout:    %.3f s\n', ...
    strComparison.strTiming.dPolyhedronHoldoutElapsedSec);
fprintf('\nSelected family: %s\n\n', ...
    strComparison.charSelectedModel);
end
