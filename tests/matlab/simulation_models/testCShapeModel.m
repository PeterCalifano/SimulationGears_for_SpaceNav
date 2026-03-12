close all
clear
clc

% Call script to setup options flags, nav-system build folder, latex options
charProjectDir = pwd;
LoadUserPathConfig; % Does nothing for now
SetupOptions; % TODO implement options overriding for MC mode (to prevent accidental changes while working on them manually)

%% SCENARIO SETUP
% Set scenario name for loading
enumTargetName_Dyn      = EnumScenarioName.Apophis;     % Apophis, Itokawa, Bennu_OREx
enumTargetName_Shape    = EnumScenarioName.Apophis;  % Apophis, Itokawa, Bennu_OREx
enumTrajName            = EnumTrajectoryNames.RTO_4t1_J13p0;

% Initialize scenario configuration object
objScenarioConfig = SScenarioConfiguration();
objScenarioConfig.enumWorldFrame    = EnumFrameName_RCS1.J2000;

cd(charProjectDir);
% Instantiate BlenderPyCommManager with automatic management of server
% Select Blender model
charScriptName              = 'BlenderPy_UDP_TCP_interface.py';

% Set dump folder for processed images with keypoints
charScenePlotDumpFolder = "./.tmp_output_GT";
charAlgorithmDumpFolder = "./.tmp_output_KLT";

% Build shape model object with kernels path for scenarios-dependent shape model
charNavBackendKernelsPATH = '/home/peterc/devDir/nav-backend/simulationCodes/data/SPICE_kernels';
CSPICEkerLoader(charNavBackendKernelsPATH, enumTargetName_Shape); % DEVNOTE: currently required to load frames and ephemerides

cd(charProjectDir);
% Shape model class: CShapeModel
[objShapeModel, strBpyCommManagerPaths] = DefineShapeModel(enumTargetName_Shape);
%% Plot shape
dCameraPosition_NavFrame = 1E3 * [1; 0.1; 0];
dSunPosition_NavFrame    = 1E9 * [1;0;0];
dBodyDCM_NavFrameFromOF  = eye(3);


[objFig, cellPlotObjs] = Visualize3dShapeModelWithPC(objShapeModel.getShapeStruct(), ...
                                                  dCameraPosition_NavFrame, ...
                                                  dSunPosition_NavFrame, ...
                                                  dBodyDCM_NavFrameFromOF);



