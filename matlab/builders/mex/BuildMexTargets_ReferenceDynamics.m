function strMexInfo = BuildMexTargets_ReferenceDynamics(charBuildDir)
%% PROTOTYPE
% strMexInfo = BuildMexTargets_ReferenceDynamics(charBuildDir)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Builds representative MEX targets for ComputeRefDynFcn:
%   - 6-state RHS plus optional Jacobian output;
%   - 6-state plus flattened 6x6 STM propagation.
%
% The sample signature intentionally includes spherical-harmonics and polyhedron-gravity fields so codegen
% covers the non-central gravity branches as well as the central reference dynamics path.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charBuildDir   Optional output directory for generated MEX files.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strMexInfo     Build directory and target names.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5      Add MEX build utility for reference dynamics propagation.
% 28-05-2026    Pietro Califano, Codex 5.5      Move to codegen builders and standardize builder name.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputeRefDynFcn()
% ComputePolyhedronFaceEdgeData()
% ComputeMeshModelVolumeAndCoM()
% -------------------------------------------------------------------------------------------------------------

%% Function code
if nargin < 1
    charBuildDir = "";
end
charBuildDir = ResolveMexBuildDirectory(charBuildDir, 'simulation_models', 'propagators');
addpath(charBuildDir);
charCallDir = pwd;
objCleanup = onCleanup(@() cd(charCallDir));
cd(charBuildDir);

cfg = coder.config('mex');

ui32FaceVertexIds = uint32([1 2 3; 1 4 2; 1 3 4; 2 4 3]);
dVerticesPos = [ 1.0  1.0  1.0;
                 1.0 -1.0 -1.0;
                -1.0  1.0 -1.0;
                -1.0 -1.0  1.0 ];
dDensity = 2500.0;
dGravConst = 6.67430e-11;

[ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData(ui32FaceVertexIds, dVerticesPos);
[dVolume, ~] = ComputeMeshModelVolumeAndCoM(ui32FaceVertexIds, dVerticesPos);

strDynParams = struct();
strDynParams.strMainData = struct();
strDynParams.strMainData.dGM = dGravConst * dDensity * dVolume;
strDynParams.strMainData.dRefRadius = 1.0;
strDynParams.strMainData.dSHcoeff = zeros(4, 2);
strDynParams.strMainData.ui16MaxSHdegree = uint16(2);
strDynParams.strMainData.strPolyhedronGravityData = struct( ...
    'ui32FaceVertexIds', ui32FaceVertexIds, ...
    'dVerticesPos', dVerticesPos, ...
    'dDensity', dDensity, ...
    'ui32EdgeVertexIds', ui32EdgeVertexIds, ...
    'dEdgeDyadics', dEdgeDyadics, ...
    'dFaceDyadics', dFaceDyadics, ...
    'dGravConst', dGravConst, ...
    'dGravParam', dGravConst * dDensity * dVolume);

codegen('-config', cfg, '-d', charBuildDir, '-o', 'ComputeRefDynFcnRHS_mex', ...
    'ComputeRefDynFcn', '-args', {0.0, zeros(6, 1), strDynParams});

codegen('-config', cfg, '-d', charBuildDir, '-o', 'ComputeRefDynFcnSTM_mex', ...
    'ComputeRefDynFcn', '-args', {0.0, zeros(42, 1), strDynParams});

strMexInfo = struct();
strMexInfo.charBuildDir = charBuildDir;
strMexInfo.cellMexTargets = {'ComputeRefDynFcnRHS_mex', 'ComputeRefDynFcnSTM_mex'};

end
