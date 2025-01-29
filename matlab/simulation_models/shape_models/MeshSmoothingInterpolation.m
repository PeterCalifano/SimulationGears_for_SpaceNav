close all
clear
clc


cd(fileparts(which(mfilename)))
addpath(genpath(fileparts(pwd)));
cspice_kclear()

% Path settings
[bflag, usrname] = system("whoami"); % Get user
if strcmpi(string(usrname(1:end-1)), "peterc") || contains(string(usrname(1:end-1)), "peterc-flip\pietr" )

    InitializeEnv; % Load environment and kernels (else manually run it before Main)

    % test_case_folder = '../data/milani/4 - Phase D/GNC/test_cases/functional_tests'; % Specify path to sharepoint folder here
    test_case_folder = '../data/rcs-1/pre-phase-A/simulator_input/'; % Specify path to sharepoint folder here
    path_to_SCobj   = '/home/peterc/devDir/projects-DART/data/milani/3 - Phase C/GNC/milani-input/shape_models';

    navcam_dumping_path = fullfile('artifacts', '.tmprcs1'); % ACHTUNG: this is not passed to py script for Blender where it is hardcoded!

    % Define paths to the Blender model and Python script
    % BlenderModelPath        = '../data/milani/Blender_models/';
    % BlenderModelPath = "/home/peterc/devDir/rendering-sw/corto_PeterCdev/input/OLD_ones_0.1/S7_Apophis/S7_Apophis.blend";
    BlenderModelPath = "/home/peterc/devDir/projects-DART/data/rcs-1/pre-phase-A/blender/Apophis_RGB.blend";

    % CORTO_pyInterface_path  = 'script/CORTO_interfaces/corto_PeterCdev/server_api/CORTO_UDP_TCP_interface.py';
    CORTO_pyInterface_path  = 'lib/corto_PeterCdev/server_api/CORTO_UDP_TCP_interface.py';

    % Target obj file
    charObjPath = fullfile("..", "data/rcs-1/pre-phase-A/");
    charObjFilename = 'apophis_v233s7_vert2_new_mod.obj';
end



% Experimental script to interpolate a mesh and increase the number of vertices and triangles.
% This is intended to create a smoother surface given a mesh with a low number of triangles
% GriddedInterpolant is used (not using linear interpolation obviously)

% Load mesh object and prepare data
charWorkDir = pwd;
cd(charObjPath)
objFilePath = which(charObjFilename); % Get absolute path
cd(charWorkDir);

strShapeModel = ObjRead(objFilePath);
ui32TrianglesIdx = uint32(strShapeModel.f);
dVertices = strShapeModel.v;

strShapeModel = struct();
strShapeModel.ui32TrianglesIdx = ui32TrianglesIdx;
strShapeModel.dVertices = 1000 * dVertices; % [m]

% Plot original mesh using trimesh
figure('Renderer', 'opengl'); %#ok<*FGREN>
trimesh(strShapeModel.ui32TrianglesIdx, strShapeModel.dVertices(:,1), strShapeModel.dVertices(:,2), strShapeModel.dVertices(:,3))
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
axis equal
axis padded

% Create mesh plot using Lidar toolbox (remeshes from PC)
figure('Renderer', 'opengl')
dOctreeDepth = 8;
objPC_initial = pointCloud(strShapeModel.dVertices);
objInitialSurfaceMesh = pc2surfacemesh(objPC_initial, "poisson", dOctreeDepth);
surfaceMeshShow(objInitialSurfaceMesh);
title('Original (re-)mesh')



% Convert mesh from cartesian coordinates to spherical coordinates
dAzValues       = zeros(size(strShapeModel.dVertices, 1), 1);
dElValues       = zeros(size(strShapeModel.dVertices, 1), 1);
dRangeValues    = zeros(size(strShapeModel.dVertices, 1), 1);

[dAzValues(:), dElValues(:), dRangeValues(:)] = cart2sph(strShapeModel.dVertices(:,1), strShapeModel.dVertices(:,2), strShapeModel.dVertices(:,3));

% Construct fine grid of spherical coordinates using liner interpolation of spherical coordinates (ACHTUNG: not uniformly distributed!
ui32PointsMultiplier = 10;
ui32NumOfPoints = length(dAzValues);
dSamplingGridSph1D = 1:ui32NumOfPoints;

ui32NumOfPoints_finer = ui32PointsMultiplier*ui32NumOfPoints;
dSamplingGridSph1D_finer = linspace(1, ui32NumOfPoints, ui32NumOfPoints_finer);

dAzValues_finer = interp1(dSamplingGridSph1D, dAzValues, dSamplingGridSph1D_finer, "linear")';
dElValues_finer = interp1(dSamplingGridSph1D, dElValues, dSamplingGridSph1D_finer, "linear")';

% Plot spherical coordinates values
figure
tiledlayout('vertical');
nexttile
plot(dSamplingGridSph1D, dAzValues, 'DisplayName', 'Original')
hold on
plot(dSamplingGridSph1D_finer, dAzValues_finer,  'DisplayName', 'Interpolated')
DefaultPlotOpts
title('Azimuth values')
nexttile
plot(dSamplingGridSph1D, dElValues)
hold on
plot(dSamplingGridSph1D_finer, dElValues_finer)
DefaultPlotOpts
title('Elevation values')

figure
tiledlayout('flow');
nexttile
[dSphereGridCoords] = ConvertSphericalGridToCart(dAzValues, dElValues, 1);
plot3(dSphereGridCoords(:,1), dSphereGridCoords(:, 2), dSphereGridCoords(:, 3), '.', 'Color', 'b', 'DisplayName', 'Original grid')
hold on
[dSphereGridCoords_finer] = ConvertSphericalGridToCart(dAzValues_finer, dElValues_finer, 1);
plot3(dSphereGridCoords_finer(:,1), dSphereGridCoords_finer(:, 2), dSphereGridCoords_finer(:, 3), '.', 'Color', 'r', 'DisplayName', 'Interpolated grid')
DefaultPlotOpts
axis equal

% Create 2D grids
[dQueryAzMatrix, dQueryElMatrix] = meshgrid(unique(dAzValues_finer), unique(dElValues_finer));
% TODO (PC): test gridding using Icosahedron sampling on sphere instead of interpolating

% Create scattered interpolant using original mesh spherical data
objSphericalScatterInterp = scatteredInterpolant(dAzValues, dElValues, dRangeValues, 'linear', 'linear');

% Query finer mesh to interpolate range values
dInterpRange_finer = objSphericalScatterInterp(dQueryAzMatrix, dQueryElMatrix);

% TODO (PC): try with interp3. The issue of the spherical coordinates interpolation is that the poles are
% really bad due to how the input mesh is formed.

% Construct gridded interpolant from interpolated values
objSphericalGriddedInterp = griddedInterpolant(dQueryAzMatrix', dQueryElMatrix', dInterpRange_finer', 'spline');

% Construct new linear uniform ndgrid to sample sphere
dAzValues_linUniform = linspace(-pi, pi, 100);
dElValues_linUniform = linspace(-pi/2, pi/2, 100);

[dQueryAzMatrix_grid, dQueryElMatrix_grid] = ndgrid(dAzValues_linUniform, dElValues_linUniform);

% Query gridded interpolant to get new point cloud
dInterpRange_grid = objSphericalGriddedInterp(dQueryAzMatrix_grid, dQueryElMatrix_grid);

% Get interpolated cartesian coordinates
[dInterpMeshCoords] = ConvertSphericalGridToCart(dQueryAzMatrix_grid(:), dQueryElMatrix_grid(:), dInterpRange_grid(:));

% Plot new point cloud superimposed to old mesh
figure('Renderer', 'opengl')
hold on
pcshow(dInterpMeshCoords)
axis equal
title('Interpolated point cloud')


% Construct new mesh
figure('Renderer', 'opengl')
dOctreeDepth = 8;
objPC = pointCloud(dInterpMeshCoords);

objInterpSurfaceMesh = pc2surfacemesh(objPC, "poisson", dOctreeDepth);
surfaceMeshShow(objInterpSurfaceMesh);
title('Interpolated mesh')




