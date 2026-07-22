function strMexInfo = BuildMexTargets_InertialDynMaxFidelity(charBuildDir)
%% PROTOTYPE
% strMexInfo = BuildMexTargets_InertialDynMaxFidelity(charBuildDir)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Builds spherical-harmonic and polyhedron MEX targets for the max-fidelity inertial dynamics RHS and matching
% Jacobian. The generic target names retain spherical harmonics for backward compatibility; polyhedron targets use
% the explicit _polyhedron_mex suffix.
%
% The build is intentionally fail-fast: if RHS code generation fails, the Jacobian target is not attempted; if the
% Jacobian target fails, the first failure is reported and rethrown.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charBuildDir:        [1]       Optional output directory for generated MEX files.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strMexInfo:          struct    Build directory, generated target names, and representative interface metadata.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-05-2026    Pietro Califano                 Add fail-fast MEX build utility for max-fidelity RHS and Jacobian.
% 28-05-2026    Pietro Califano, Codex 5.5      Move to codegen builders and standardize builder name.
% 22-07-2026    Pietro Califano, Codex           Build separate compile-time SH and polyhedron gravity variants.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% evalRHS_InertialDynMaxFidelity()
% evalJac_InertialDynMaxFidelity()
% ComputePolyhedronFaceEdgeData()
% ComputeMeshModelVolumeAndCoM()
% GenerateGaussMarkovAccelSeq()
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Ensure repo-local source and MathCore dependency paths are available when the builder is invoked directly.
BootstrapRepositoryPaths_();

% Set default build directory if not provided, and validate inputs.
if nargin < 1
    charBuildDir = "";
end
charBuildDir = ResolveMexBuildDirectory(charBuildDir, 'simulation_models', 'dynamics');

addpath(charBuildDir);
charCallDir = pwd;
objCleanup = onCleanup(@() cd(charCallDir));
cd(charBuildDir);

cfg = coder.config('mex');
cfg.GenerateReport = true;

[dStateTimetag, dxState_IN, strDynParams, strModelConfigFlags, strAccelInfo] = BuildRepresentativeInputs_();

strSHmodelConfigFlags = strModelConfigFlags;
strSHmodelConfigFlags.bIncludeSphericalHarmonics = true;
strSHmodelConfigFlags.bIncludePolyhedronGravity = false;

strPolyhedronModelConfigFlags = strModelConfigFlags;
strPolyhedronModelConfigFlags.bIncludeSphericalHarmonics = false;
strPolyhedronModelConfigFlags.bIncludePolyhedronGravity = true;

cellGravityModelNames = {'spherical_harmonics', 'polyhedron'};
cellModelConfigFlags = {strSHmodelConfigFlags, strPolyhedronModelConfigFlags};
cellRHSTargets = {'evalRHS_InertialDynMaxFidelity_mex', ...
                  'evalRHS_InertialDynMaxFidelity_polyhedron_mex'};
cellJacTargets = {'evalJac_InertialDynMaxFidelity_mex', ...
                  'evalJac_InertialDynMaxFidelity_polyhedron_mex'};
cellMexTargets = cell(1, 2 * numel(cellGravityModelNames));

for ui32ModelIdx = uint32(1):uint32(numel(cellGravityModelNames))
    dModelIdx = double(ui32ModelIdx);
    strSelectedModelConfigFlags = cellModelConfigFlags{dModelIdx};
    charRHSTarget = cellRHSTargets{dModelIdx};
    charJacTarget = cellJacTargets{dModelIdx};

    try
        codegen('-config', cfg, '-d', charBuildDir, '-o', charRHSTarget, ...
            'evalRHS_InertialDynMaxFidelity', ...
            '-args', {dStateTimetag, dxState_IN, strDynParams, coder.Constant(strSelectedModelConfigFlags)});
    catch objException
        ReportCodegenFailure_(charRHSTarget, objException);
        rethrow(objException);
    end

    try
        codegen('-config', cfg, '-d', charBuildDir, '-o', charJacTarget, ...
            'evalJac_InertialDynMaxFidelity', ...
            '-args', {dStateTimetag, dxState_IN, strDynParams, coder.Constant(strSelectedModelConfigFlags), strAccelInfo});
    catch objException
        ReportCodegenFailure_(charJacTarget, objException);
        rethrow(objException);
    end

    ui32TargetOffset = uint32(2) * (ui32ModelIdx - uint32(1));
    cellMexTargets{double(ui32TargetOffset + 1)} = charRHSTarget;
    cellMexTargets{double(ui32TargetOffset + 2)} = charJacTarget;
end

strMexInfo = struct();
strMexInfo.charBuildDir = charBuildDir;
strMexInfo.ui32StateSize = uint32(numel(dxState_IN));
strMexInfo.cellGravityModelNames = cellGravityModelNames;
strMexInfo.cellMexTargets = cellMexTargets;

end

function [dStateTimetag, dxState_IN, strDynParams, strModelConfigFlags, strAccelInfo] = BuildRepresentativeInputs_()
% Build a compact representative max-fidelity payload for codegen signatures.
dStateTimetag = 0.0;
dxState_IN = [4.0; 0.3; -0.2; 0.0; 0.01; 0.0];

ui32FaceVertexIds = uint32([1 2 3; 1 4 2; 1 3 4; 2 4 3]);
dVerticesPos = [ 1.0  1.0  1.0;
                 1.0 -1.0 -1.0;
                -1.0  1.0 -1.0;
                -1.0 -1.0  1.0 ];
dDensity = 2500.0;
dGravConst = 6.67430e-11;

[ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData(ui32FaceVertexIds, dVerticesPos);
[dVolume, ~] = ComputeMeshModelVolumeAndCoM(ui32FaceVertexIds, dVerticesPos);
dGravParam = dGravConst * dDensity * dVolume;

strPolyhedronGravityData = struct();
strPolyhedronGravityData.ui32FaceVertexIds = ui32FaceVertexIds;
strPolyhedronGravityData.dVerticesPos = dVerticesPos;
strPolyhedronGravityData.dDensity = dDensity;
strPolyhedronGravityData.ui32EdgeVertexIds = ui32EdgeVertexIds;
strPolyhedronGravityData.dEdgeDyadics = dEdgeDyadics;
strPolyhedronGravityData.dFaceDyadics = dFaceDyadics;
strPolyhedronGravityData.dGravConst = dGravConst;

dSHcoeff = zeros(4, 2);
dSHcoeff(2, 1) = -1.0e-3;

strDynParams = struct();
strDynParams.strMainData.dGM = dGravParam;
strDynParams.strMainData.dRefRadius = 1.0;
strDynParams.strMainData.dSHcoeff = dSHcoeff;
strDynParams.strMainData.ui16MaxSHdegree = uint16(2);
strDynParams.strMainData.strPolyhedronGravityData = strPolyhedronGravityData;

strDynParams.strBody3rdData(1).dGM = 3.0;
strDynParams.strBody3rdData(1).dRefRadius = 0.1;
strDynParams.strBody3rdData(1).strOrbitData = BuildConstantOrbitData_([20.0; 0.0; 0.0]);
strDynParams.strBody3rdData(2).dGM = 5.0;
strDynParams.strBody3rdData(2).dRefRadius = 0.2;
strDynParams.strBody3rdData(2).strOrbitData = BuildConstantOrbitData_([0.0; 30.0; 0.0]);

strDynParams.strSRPdata.dP_SRP0 = 4.0;
strDynParams.strSRPdata.dP_SRP = 4.0;
strDynParams.strSRPdata.dReferenceDistance = 10.0;
strDynParams.strSRPdata.bRecomputePressureFromDistance = true;

strDynParams.strSCdata.dReflCoeff = 1.0;
strDynParams.strSCdata.dSCmass = 1.0;
strDynParams.strSCdata.dA_SRP = 2.0;
strDynParams.strSCdata.strSRPpanelData = struct( ...
    'dSCquadsArea', 1.0, ...
    'dDiffSpecQuadsCoeffs', [0.0 0.0], ...
    'dQuadsNormals_SCB', [1.0; 0.0; 0.0], ...
    'dQuadsPressCentre_SCB', [0.0; 0.0; 0.0], ...
    'charLengthUnit', 'm');
strDynParams.strStochasticAccelData = GenerateGaussMarkovAccelSeq( ...
    [0.0, 1.0], zeros(3, 1), ones(3, 1), 1.0, uint32(1), ...
    'bSampleInitialAccel', false);

strModelConfigFlags = struct();
strModelConfigFlags.bIncludeMainGravity = true;
strModelConfigFlags.bIncludeSphericalHarmonics = false;
strModelConfigFlags.bIncludeThirdBodies = true;
strModelConfigFlags.bIncludeSunThirdBody = true;
strModelConfigFlags.bIncludeEarthThirdBody = true;
strModelConfigFlags.bIncludeSRP = true;
strModelConfigFlags.bIncludeEclipse = true;
strModelConfigFlags.bUsePanelSRP = false;
strModelConfigFlags.bIncludeStochasticAcceleration = false;
strModelConfigFlags.bIncludePolyhedronGravity = false;
strModelConfigFlags.bRecomputeSRPpressureFromDistance = true;

strAccelInfo = struct();
strAccelInfo.dSRPdistToSun = norm(dxState_IN(1:3) - [20.0; 0.0; 0.0]);
strAccelInfo.bIsSRPActive = true;

end

function BootstrapRepositoryPaths_()
% Reuse the repository path bootstrap so codegen sees SimulationGears and MathCore sources.
charBuilderDir = fileparts(mfilename('fullpath'));
charMatlabRoot = fileparts(fileparts(charBuilderDir));
addpath(charMatlabRoot, '-begin');

SetupSimGears();

end

function strOrbitData = BuildConstantOrbitData_(dPosition_IN)
% Build a constant Chebyshev ephemeris compatible with evalChbvPolyWithCoeffs.
strOrbitData = struct();
strOrbitData.ui32PolyDeg = uint32(2);
strOrbitData.dChbvPolycoeffs = [dPosition_IN(1); 0.0; 0.0; ...
                                dPosition_IN(2); 0.0; 0.0; ...
                                dPosition_IN(3); 0.0; 0.0];
strOrbitData.dTimeLowBound = -100.0;
strOrbitData.dTimeUpBound = 100.0;

end

function ReportCodegenFailure_(charTarget, objException)
% Print the first failed target explicitly before rethrowing MATLAB's diagnostic.
fprintf(2, 'Code generation failed while building %s. Stopping without attempting remaining targets.\n', charTarget);
fprintf(2, '%s\n', objException.message);

end
