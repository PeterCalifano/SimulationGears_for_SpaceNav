function strMexInfo = BuildGravityModelMexTargets(charBuildDir, ui32MaxSHdegree)
%% PROTOTYPE
% strMexInfo = BuildGravityModelMexTargets(charBuildDir, ui32MaxSHdegree)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Builds MEX targets for the codegen-oriented gravity evaluators:
%   - exterior spherical harmonics target-frame, world-frame, multi-sample, core, and Jacobian evaluators;
%   - polyhedron gravity single-point and perturbative multi-sample evaluators.
%
% This utility is intentionally scoped to RHS/Jacobian evaluation kernels.
% It does not build the adaptive polyhedron-to-SHE fitter, which performs
% sampling, least-squares fitting, diagnostics, and struct assembly.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charBuildDir:        [1]       Optional output directory for generated MEX files.
% ui32MaxSHdegree:     [1]       Optional fixed maximum SH degree for generated MEX signatures.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strMexInfo:          struct    Build directory, target names, and representative dimensions.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Add focused MEX build utility for gravity evaluators.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalExtSphHarmExpInTargetFrame()
% EvalExtSphHarmExpInWorldFrame()
% EvalExtSphHarmSamplesInTargetFrame()
% EvalJac_ExtSphHarmExpInTargetFrame()
% EvalExtSphericalHarmExpCore()
% EvalPolyhedronGrav()
% EvalPolyhedronGravPerturbationSamples()
% ComputePolyhedronFaceEdgeData()
% ComputeMeshModelVolumeAndCoM()
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Set default build directory if not provided, and validate inputs
if nargin < 1 || strlength(string(charBuildDir)) == 0
    charBuildDir = fullfile(tempdir, 'simgears_gravity_codegen');
end
charBuildDir = char(string(charBuildDir));

if nargin < 2
    ui32MaxSHdegree = uint32(8);
end

if ~isa(ui32MaxSHdegree, 'uint32') || ~isscalar(ui32MaxSHdegree) || ui32MaxSHdegree < uint32(2)
    error('BuildGravityModelMexTargets:InvalidMaxDegree', ...
        'ui32MaxSHdegree must be a uint32 scalar greater than or equal to 2.');
end

if ~exist(charBuildDir, 'dir')
    mkdir(charBuildDir);
end

addpath(charBuildDir);

% Sample data for codegen target signatures
cfg = coder.config('mex');

dSampleRadius = 7000.0;
dSampleMu = 3.986004418e5;
ui32CoeffRows = (ui32MaxSHdegree + uint32(1)) * (ui32MaxSHdegree + uint32(2)) / uint32(2) - uint32(2);
dCoeffCols = zeros(double(ui32CoeffRows), 2);

dPosSC_TB = [dSampleRadius; 100.0; 50.0];
dSamplePos_TB = [dSampleRadius 0.5 * dSampleRadius -0.25 * dSampleRadius; ...
                 100.0         dSampleRadius  0.5 * dSampleRadius; ...
                 50.0         -0.5 * dSampleRadius dSampleRadius];
dDCM_WfromTB = eye(3);

% Build MEX targets for spherical harmonics and polyhedron gravity evaluators
codegen('-config', cfg, '-d', charBuildDir, 'EvalExtSphHarmExpInTargetFrame', ...
    '-args', {dPosSC_TB, ui32MaxSHdegree, dCoeffCols, dSampleMu, dSampleRadius});
codegen('-config', cfg, '-d', charBuildDir, 'EvalExtSphHarmExpInWorldFrame', ...
    '-args', {dPosSC_TB, dDCM_WfromTB, ui32MaxSHdegree, dCoeffCols, dSampleMu, dSampleRadius});
codegen('-config', cfg, '-d', charBuildDir, 'EvalExtSphHarmSamplesInTargetFrame', ...
    '-args', {dSamplePos_TB, ui32MaxSHdegree, dCoeffCols, dSampleMu, dSampleRadius});
codegen('-config', cfg, '-d', charBuildDir, 'EvalJac_ExtSphHarmExpInTargetFrame', ...
    '-args', {dPosSC_TB, ui32MaxSHdegree, dCoeffCols, dSampleMu, dSampleRadius});
codegen('-config', cfg, '-d', charBuildDir, 'EvalExtSphericalHarmExpCore', ...
    '-args', {norm(dPosSC_TB), asin(dPosSC_TB(3) / norm(dPosSC_TB)), ...
              atan2(dPosSC_TB(2), dPosSC_TB(1)), ui32MaxSHdegree, ...
              dCoeffCols, dSampleMu, dSampleRadius});

ui32FaceVertexIds = uint32([1 2 3; 1 4 2; 1 3 4; 2 4 3]);
dVerticesPos = [ 1.0  1.0  1.0;
                 1.0 -1.0 -1.0;
                -1.0  1.0 -1.0;
                -1.0 -1.0  1.0 ];
dDensity = 2500.0;
dGravConst = 6.67430e-11;

% Get intermediate samples for polyhedron gravity targets
[ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData(ui32FaceVertexIds, dVerticesPos);
[dVolume, ~] = ComputeMeshModelVolumeAndCoM(ui32FaceVertexIds, dVerticesPos);

dPolyGravParam = dGravConst * dDensity * dVolume;
dFieldPoint_BF = [3.0; -0.25; 0.5];
dPolySamplePos_TB = [3.0 0.5 -0.25; 0.0 3.0 0.5; 0.25 -0.5 3.0];

% Build MEX targets for polyhedron gravity evaluators
codegen('-config', cfg, '-d', charBuildDir, 'EvalPolyhedronGrav', ...
    '-args', {dFieldPoint_BF, ui32FaceVertexIds, dVerticesPos, dDensity, ...
              ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst});

codegen('-config', cfg, '-d', charBuildDir, 'EvalPolyhedronGravPerturbationSamples', ...
    '-args', {dPolySamplePos_TB, ui32FaceVertexIds, dVerticesPos, dDensity, ...
              ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst, dPolyGravParam});

strMexInfo = struct();
strMexInfo.charBuildDir = charBuildDir;
strMexInfo.ui32MaxSHdegree = ui32MaxSHdegree;
strMexInfo.ui32CoeffRows = ui32CoeffRows;
strMexInfo.cellMexTargets = { ...
    'EvalExtSphHarmExpInTargetFrame_mex', ...
    'EvalExtSphHarmExpInWorldFrame_mex', ...
    'EvalExtSphHarmSamplesInTargetFrame_mex', ...
    'EvalJac_ExtSphHarmExpInTargetFrame_mex', ...
    'EvalExtSphericalHarmExpCore_mex', ...
    'EvalPolyhedronGrav_mex', ...
    'EvalPolyhedronGravPerturbationSamples_mex'};

end
