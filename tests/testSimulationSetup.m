clear
close all
clc

%% Test script for simulation setup classes
includeDirectories();

%% TEST: CSPICEkerLoader
kernelsPATH = '/home/peterc/devDir/nav-backend/simulationCodes/data/SPICE_kernels';
CSPICEkerLoader(kernelsPATH, EnumScenarioName.Itokawa);

%% TEST: CShapeModel
% Instantiate object
kernelname = fullfile(kernelsPATH, 'Itokawa/dsk/hay_a_amica_5_itokawashape_v1_0_64q.bds');
objShapeModel = CShapeModel('cspice', kernelname, 'km', 'm');

% strShapeModel.i32triangVertexPtr = trianglesVertices;
% strShapeModel.dVerticesPos = modelVertices;

% Get struct data from shape model object
strShapeModel = objShapeModel.getShapeStruct();

% Prepare LM table for MEx function
strShapeModel = orderfields(strShapeModel);


%% TEST: CTargetEmulator
MAX_NUM_GT = 10000;
objTargetEmulator = CTargetEmulator(objShapeModel, uint32(MAX_NUM_GT));

shapeStruct = objTargetEmulator.getShapeStruct();
[targetPos, targetDCM] = objTargetEmulator.GetPose();

% Get landmarks from target emulators
rng(0);
random_cols = randi(MAX_NUM_GT, 1, MAX_NUM_GT);

objTargetEmulator = objTargetEmulator.GenerateSimulatedPoints_TB("random_uniform");
[dPointsPositionsGT_TB, ui32pointsIDs] = objTargetEmulator.GetPointsInTargetFrame(random_cols, true);

figure();
plot3(dPointsPositionsGT_TB(1, :), dPointsPositionsGT_TB(2, :), dPointsPositionsGT_TB(3,:), 'k.')
DefaultPlotOpts()
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal

% Verify the other branch to get points (using GT indices)
[dPointsPositionsGT_TB_check, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
assert(sum(ui32pointsIDs == ui32pointsIDs) == length(ui32pointsIDs))






