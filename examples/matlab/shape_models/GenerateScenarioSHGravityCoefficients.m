function strGeneration = ...
        GenerateScenarioSHGravityCoefficients( ...
            enumScenarioName, ui32MaxDegree, charDataRootPath, options)
%% SIGNATURE
% strGeneration = GenerateScenarioSHGravityCoefficients( ...
%     enumScenarioName, ui32MaxDegree, charDataRootPath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Manually generate one exterior spherical-harmonics gravity coefficient
% family from a registry-backed target shape.
%
% DefineShapeModel owns target-specific OBJ/DSK resolution. This generator
% translates the loaded mesh to its uniform-density volume centroid, fits the
% requested degree outside the mesh enclosing sphere, and rescales the fitted
% coefficients to the target's registered mean/reference normalization radius.
%
% Example:
% strGeneration = GenerateScenarioSHGravityCoefficients( ...
%     "Apophis", uint32(16));
% disp(strGeneration.strGeneratedModel.dCSlmCoeffCols)
%
% Expected output:
% The function prints mesh/normalization evidence and MATLAB-ready coefficient
% rows. The returned struct contains both the direct fit and the rescaled model
% intended for review before registry embedding.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumScenarioName             Registered scenario name, alias, or EnumScenarioName.
% ui32MaxDegree                Requested maximum spherical-harmonics degree.
% charDataRootPath             SimulationGears data root. Empty uses default data resolution.
% options.ui32MaxFitIterations Maximum adaptive fit iterations.
% options.dMeshSimplifyFactor  Mesh keep fraction; 1.0 uses the complete registered mesh.
% options.charOutputMatFilePath
%                              Optional MAT output path for the generation evidence.
% options.bPrintCoefficientRows
%                              Print MATLAB-ready unnormalized [Clm, Slm] rows.
% options.bVerbose             Print mesh and fit metadata.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strGeneration               Generated coefficient family, mesh evidence, and scenario metadata.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry, DefineShapeModel, CShapeModel, ComputeMeshModelVolumeAndCoM,
% RescaleSphericalHarmonicsReferenceRadius
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    enumScenarioName (1, :) {mustBeA(enumScenarioName, ...
        ["string", "char", "EnumScenarioName"])} = "Apophis"
    ui32MaxDegree (1, 1) uint32 = uint32(16)
    charDataRootPath (1, :) char = ''
    options.ui32MaxFitIterations (1, 1) uint32 = uint32(1)
    options.dMeshSimplifyFactor (1, 1) double ...
        {mustBeGreaterThan(options.dMeshSimplifyFactor, 0.0), ...
         mustBeLessThanOrEqual(options.dMeshSimplifyFactor, 1.0)} = 1.0
    options.charOutputMatFilePath (1, :) char = ''
    options.bPrintCoefficientRows (1, 1) logical = true
    options.bVerbose (1, 1) logical = true
end

arguments (Output)
    strGeneration (1, 1) struct
end

charExampleDirectory = fileparts(mfilename('fullpath'));
charRepositoryRoot = fileparts(fileparts(fileparts(charExampleDirectory)));
run(fullfile(charRepositoryRoot, 'matlab', 'SetupSimGears.m'));

[enumResolvedScenario, charCanonicalName] = ...
    CScenarioRegistry.ResolveScenario(enumScenarioName);
strScenarioSpec = CScenarioRegistry.GetScenarioSpec( ...
    enumResolvedScenario, charLengthUnits="km");
ValidateScenarioGravityInputs_(strScenarioSpec, ui32MaxDegree);

% Reuse the registry-backed shape builder so target-specific OBJ/DSK loading,
% unit conversion, and installed-data validation stay in one implementation.
[objShapeModel, ~, strShapeMetadata] = DefineShapeModel( ...
    enumResolvedScenario, charDataRootPath, ...
    charOutputLengthUnits="km", ...
    dMeshSimplifyFactor=options.dMeshSimplifyFactor, ...
    bInitSphericalHarmonicsGravityData=false);
strShapeData = objShapeModel.getShapeStruct();
ui32FaceVertexIds = uint32(strShapeData.ui32triangVertexPtr.');
dVerticesPos = strShapeData.dVerticesPos.';
[dVolumeBeforeCentering, dMeshCOM] = ComputeMeshModelVolumeAndCoM( ...
    ui32FaceVertexIds, dVerticesPos);

% Center the expansion origin at the uniform-density volume centroid. The
% original offset remains in the returned evidence rather than changing the
% installed scenario asset.
strCenteredShapeData = strShapeData;
strCenteredShapeData.dVerticesPos = ...
    strShapeData.dVerticesPos - dMeshCOM;
objCenteredShape = CShapeModel( ...
    "struct", strCenteredShapeData, "km", "km", true, ...
    sprintf('%s centered for gravity fit', charCanonicalName), true);
strCenteredShapeData = objCenteredShape.getShapeStruct();
ui32CenteredFaces = uint32( ...
    strCenteredShapeData.ui32triangVertexPtr.');
dCenteredVertices = strCenteredShapeData.dVerticesPos.';
[dCenteredVolume, dCenteredCOM] = ComputeMeshModelVolumeAndCoM( ...
    ui32CenteredFaces, dCenteredVertices);
dEnclosingRadius = max(vecnorm(dCenteredVertices, 2, 2));

% Fit at an enclosing radius for numerical sampling, then reuse the shared
% transformation to express the same physical field at the registered
% normalization radius.
strGeneratedAtFitRadius = ...
    CShapeModel.BuildSphericalHarmonicsGravityData( ...
        objCenteredShape, ui32MaxDegree, ...
        dGravParam=strScenarioSpec.dGravParam, ...
        dDensity=NaN, ...
        dGravConst=NaN, ...
        dBodyRadiusRef=NaN, ...
        ui32MaxFitIterations=options.ui32MaxFitIterations);
strGeneratedModel = RescaleSphericalHarmonicsReferenceRadius( ...
    strGeneratedAtFitRadius, strScenarioSpec.dReferenceRadius);

strGeneration = struct();
strGeneration.strGeneratedAtFitRadius = strGeneratedAtFitRadius;
strGeneration.strGeneratedModel = strGeneratedModel;
strGeneration.strScenarioSpec = strScenarioSpec;
strGeneration.strMesh = struct( ...
    'charCanonicalName', charCanonicalName, ...
    'ui32NumFaces', uint32(size(ui32CenteredFaces, 1)), ...
    'ui32NumVertices', uint32(size(dCenteredVertices, 1)), ...
    'dMeshSimplifyFactor', options.dMeshSimplifyFactor, ...
    'dVolumeBeforeCentering', dVolumeBeforeCentering, ...
    'dCenteredVolume', dCenteredVolume, ...
    'dOriginalCOM', dMeshCOM, ...
    'dCenteredCOM', dCenteredCOM, ...
    'dEnclosingRadius', dEnclosingRadius, ...
    'dNormalizationRadius', strScenarioSpec.dReferenceRadius, ...
    'strShapeMetadata', strShapeMetadata);

PrintGeneration_(strGeneration, options.bVerbose, ...
    options.bPrintCoefficientRows);

if ~isempty(options.charOutputMatFilePath)
    save(options.charOutputMatFilePath, 'strGeneration', '-v7.3');
end

end


function ValidateScenarioGravityInputs_(strScenarioSpec, ui32MaxDegree)
%% DESCRIPTION
% Reject targets that cannot define a registry-normalized gravity fit.
% -------------------------------------------------------------------------------------------------------------

if ui32MaxDegree < uint32(2)
    error('GenerateScenarioSHGravity:InvalidDegree', ...
        'The requested maximum degree must be at least 2.');
end
if ~strScenarioSpec.bHasGravityDefaults || ...
        ~isfinite(strScenarioSpec.dGravParam) || ...
        ~(strScenarioSpec.dGravParam > 0.0) || ...
        ~isfinite(strScenarioSpec.dReferenceRadius) || ...
        ~(strScenarioSpec.dReferenceRadius > 0.0)
    error('GenerateScenarioSHGravity:MissingGravityMetadata', ...
        ['Scenario "%s" requires a finite positive registered GM and ', ...
         'reference radius before coefficient generation.'], ...
        strScenarioSpec.charCanonicalName);
end

end


function PrintGeneration_(strGeneration, bVerbose, bPrintCoefficientRows)
%% DESCRIPTION
% Print concise generation evidence and copy-ready coefficient rows.
% -------------------------------------------------------------------------------------------------------------

if bVerbose
    strMesh = strGeneration.strMesh;
    strModel = strGeneration.strGeneratedModel;
    fprintf('\n%s degree-%u gravity coefficient generation\n', ...
        strMesh.charCanonicalName, strModel.ui32MaxDegree);
    fprintf('  Mesh faces:              %u\n', strMesh.ui32NumFaces);
    fprintf('  Mesh vertices:           %u\n', strMesh.ui32NumVertices);
    fprintf('  Mesh keep fraction:      %.6f\n', ...
        strMesh.dMeshSimplifyFactor);
    fprintf('  Original volume COM:     [%+.12e %+.12e %+.12e] km\n', ...
        strMesh.dOriginalCOM);
    fprintf('  Centered COM norm:       %.6e km\n', ...
        norm(strMesh.dCenteredCOM));
    fprintf('  Enclosing fit radius:    %.12g km\n', ...
        strMesh.dEnclosingRadius);
    fprintf('  Normalization radius:    %.12g km\n', ...
        strMesh.dNormalizationRadius);
    fprintf('  Fit iterations / best:   %u / %u\n', ...
        strModel.strFitStats.ui32NumIterations, ...
        strModel.strFitStats.ui32BestIteration);
end

if bPrintCoefficientRows
    fprintf('\ndCSlmCoeffCols = [ ...\n');
    dCoefficientRows = ...
        strGeneration.strGeneratedModel.dCSlmCoeffCols;
    for ui32Row = uint32(1):uint32(size(dCoefficientRows, 1))
        fprintf('    %.17e, %.17e; ...\n', ...
            dCoefficientRows(ui32Row, 1), ...
            dCoefficientRows(ui32Row, 2));
    end
    fprintf('];\n\n');
end

end
