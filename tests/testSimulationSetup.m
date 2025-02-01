clear
close all
clc

%% Test script for simulation setup classes
includeDirectories();

%% TEST: CSPICEkerLoader
charKernelsPATH = '/home/peterc/devDir/nav-backend/simulationCodes/data/SPICE_kernels';
CSPICEkerLoader(charKernelsPATH, EnumScenarioName.Itokawa);

%% TEST: CShapeModel
% Instantiate object
charKernelname = fullfile(charKernelsPATH, 'Itokawa/dsk/hay_a_amica_5_itokawashape_v1_0_64q.bds');
objShapeModel = CShapeModel('cspice', charKernelname, 'km', 'm');

% strShapeModel.i32triangVertexPtr = trianglesVertices;
% strShapeModel.dVerticesPos = modelVertices;

% Get struct data from shape model object
strShapeModel = objShapeModel.getShapeStruct();

% Prepare LM table for MEx function
strShapeModel = orderfields(strShapeModel);


%% TEST: CTargetEmulator
ui32MAX_NUM_GT = 10000;
objTargetEmulator = CTargetEmulator(objShapeModel, uint32(ui32MAX_NUM_GT));

strShapeStruct = objTargetEmulator.getShapeStruct();
[targetPos, targetDCM] = objTargetEmulator.GetPose();

% Get landmarks from target emulators
rng(0);
random_cols = randi(ui32MAX_NUM_GT, 1, ui32MAX_NUM_GT);

objTargetEmulator = objTargetEmulator.GenerateSimulatedPoints_TB("random_uniform");
[dPointsPositionsGT_TB, ui32pointsIDs] = objTargetEmulator.GetPointsInTargetFrame(random_cols, true);

% Verify the other branch to get points (using GT indices)
[~, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
assert(sum(ui32pointsIDs == ui32pointsIDs_check) == length(ui32pointsIDs))

clear dPointsPositionsGT_TB_check ui32pointsIDs_check




