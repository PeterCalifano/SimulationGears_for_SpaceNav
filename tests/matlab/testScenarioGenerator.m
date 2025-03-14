% FROM PROTOTYPE_NAVSYSTEMV3
%% Include and options
% Call script to setup options flags, nav-system build folder, latex options

LoadUserPathConfig; % Does nothing for now
SetupOptions;
charProjectDir = pwd;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
enumConcurrentEstimatorType = "MSCKF";
enumFrontEndType = EnumFrontEnd.ALGORITHM;

if enumConcurrentEstimatorType ~= "SR_USKF"
    rmpath("/home/peterc/devDir/nav-backend/simulationCodes/matlab/src/SRUSKF");
    addpath("/home/peterc/devDir/SLAM-repos/MSCKF_for_SpaceNav/matlab/RCS1_tailoring")
else
    rmpath("/home/peterc/devDir/SLAM-repos/MSCKF_for_SpaceNav/matlab/RCS1_tailoring")
    addpath("/home/peterc/devDir/nav-backend/simulationCodes/matlab/src/SRUSKF");
end

% TEMPORARY for development
% charInputImageDataPath = "/home/peterc/devDir/nav-backend/simulationCodes/data/datasets/TestCase_ItokawaRCS1_RTO_3t1_J11p0_45dt";
% charInputImageDataPath = "/home/peterc/devDir/nav-backend/simulationCodes/data/datasets/TestCase_ItokawaRCS1_RTO_3t1_J11p0_45dt";
% charInputImageDataPath = "/home/peterc/devDir/nav-backend/simulationCodes/data/datasets/TestCase_ScaledBennu_OREx_RCS1_SSTO_14_60dt"; 
% charInputImageDataPath = "/home/peterc/devDir/nav-backend/simulationCodes/data/datasets/TestCase_TexturedDisplacedApophis_RCS1_SSTO_14_60dt"; 
% charInputImageDataPath = "/home/peterc/devDir/nav-backend/simulationCodes/data/datasets/TestCase_ParticlesApophis_RCS1_SSTO_12_60dt_5days";
charInputImageDataPath = "/home/peterc/devDir/nav-backend/simulationCodes/data/datasets/TestCase_ParticlesApophis_RCS1_RTO_4t1_J11p0_60dt_5days";

% Set trajectory name for loading

% Set scenario name for loading
enumTargetName_Dyn      = EnumScenarioName.Apophis;     % Apophis, Itokawa, Bennu_OREx
enumTargetName_Shape    = EnumScenarioName.Apophis;     % Apophis, Itokawa, Bennu_OREx
enumTrajName            = EnumTrajectoryNames.RTO_4t1_J11p0;

% Load data from RCS-1 dataset
[~, charRootFolder, ~] = fileparts(charInputImageDataPath);
objDataset = load( fullfile(charInputImageDataPath, sprintf("RefData_%s.mat", charRootFolder))).self;

if enumTargetName_Dyn == "Apophis"
    addpath('/home/peterc/devDir/projects-DART/rcs-1-gnc-simulator');
end

% Assert checks for scenario configuration
assert( contains(charInputImageDataPath, char(enumTrajName)), 'ERROR: selected trajectory name does not match dataset name.');
assert( contains(charInputImageDataPath, char(enumTargetName_Dyn)), 'ERROR: selected environment/body does not match dataset name.' );

% objDataset = load( fullfile(charInputImageDataPath, sprintf("RefData_TestCase_TexturedDisplacedApophis_RCS1_%s_60dt.mat", enumTrajName))).self;
charInertialFrame = objDataset.enumWorldFrame;
dET0 = objDataset.dTimestamps(1);

% Adjust units km to meters
objDataset.dPosSC_W             = 1000 * objDataset.dPosSC_W;
objDataset.dVelSC_W             = 1000 * objDataset.dVelSC_W;
objDataset.dTargetPosition_W    = 1000 * objDataset.dTargetPosition_W;
objDataset.dSunPosition_W       = 1000 * objDataset.dSunPosition_W;
objDataset.dEarthPosition_W     = 1000 * objDataset.dEarthPosition_W;

objDataset.dTimestamps = objDataset.dTimestamps - objDataset.dTimestamps(1); % Translate to start from zero

if not(exist("objTrialData", "var"))
    objTrialData = SSimulationRun();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% RUN CONFIGURATION % DEVNOTE: all options that may not be scattered go into config yaml
% Setup RNG
if bMC_TRIAL_MODE == true 

    ui32SEED_NUM = objTrialData.ui32SEED_NUM;
    rng(ui32SEED_NUM);

elseif bENABLE_RNG == true
    rng('shuffle'); % Automatically choose a seed based on the current time
else
    rng(ui32SEEN_RNG); % Automatically choose a seed based on the current time
end

% Retrieve rng generator info struct
strRNGstate = rng;
ui32SEED_NUM = strRNGstate.Seed; % Extract the seed
fprintf('\nRNG seed: %d\n', ui32SEED_NUM );


% objScenConfig = SScenarioConfiguration();
% Variables to add
% enumFrontEndType              = EnumFrontEnd.EMULATOR;
% bApplyGroundTruthScattering   = true; % TBD, may become a struct/class with separated flags for each option
% dInitialWorldEphTime          = 0
% dFinalWorldEphTime            = 0
% dSimulationStopTime           = 90000; % How much time the simulation covers in seconds
 
% Classes/data struct to define
% 1) SScenarioConfiguration 
% 2) SSensorsParams % This will be a factory for different type of sensors. Use enum + args{:}.
%                     Alternatively, one specifies the object directly (derive from common base for type
%                     checking reasons)
% 3) SActuatorsParams

% Maybe wrapped in --> SEnvironmentParams


% SSimulationManager will take these objects and store them similarly to SLX sims objects
objScenarioConfig = SScenarioConfiguration();

% Initialize scenario config (TEMPORARY)
objScenarioConfig.bScatterInitialConditions = false; % TODO this requires the scenario generator to be reworked!
objScenarioConfig.enumFrontEndType = enumFrontEndType;

objScenarioConfig.dDeltaTime = objDataset.dTimestamps(2) - objDataset.dTimestamps(1); % [s] Assumes DeltaTime is constant
objScenarioConfig.dFinalTime = objDataset.dTimestamps(end);

objScenarioConfig.dInitialTime      = max(objDataset.dTimestamps(1), 0.0);
objScenarioConfig.dInitialTimeID    = find(objDataset.dTimestamps == objScenarioConfig.dInitialTime, 1);

assert( not(isempty(objScenarioConfig.dInitialTimeID)), ...
    'ERROR: initial timestamp must be a valid entry in objDataset.dTimestamps grid due to current limitations of the simulation.' )

objScenarioConfig.enumWorldFrame    = EnumFrameName_RCS1.J2000;

% TODO improve initial state and scattering for MC mode and scenario generator
objScenarioConfig.dxState0_W        = [objDataset.dPosSC_W(:, 1); objDataset.dVelSC_W(:, 1)];
objScenarioConfig.dxStateCovScatter = diag( (0.1 * objScenarioConfig.dxState0_W).^2 );

% DEVNOTE: dTimestamps should be removed entirely in scenario configuration
objScenarioConfig.dTimestamps = objDataset.dTimestamps;
objScenarioConfig.ui32RNGseed = ui32SEEN_RNG;


if exist('ui32NUM_MAX_FRAMES_SIM', 'var')
    % Override final time
    objScenarioConfig.dFinalTime = (ui32NUM_MAX_FRAMES_SIM + 1) * objScenarioConfig.dDeltaTime;
end

% Used also as frequency of measurement acquisition in this simulation.
% DEVNOTE: this should not be defined in scenario, because it is a property of the navigation system.

fprintf('SETTINGS: \n Simulation initial and final times [%.6g, %.6g] [s], with frame each %4.2f [s]\n', ...
            objScenarioConfig.dInitialTime, objScenarioConfig.dFinalTime, objScenarioConfig.dDeltaTime)


%% BlenderPyCommManager Interface definition
% Instantiate BlenderPyCommManager with automatic management of server
% Select Blender model
charScriptName              = 'BlenderPy_UDP_TCP_interface.py';

bUseRCS1 = false;

if bUseRCS1 == true
    charRootPath = "/home/peterc/devDir/projects-DART/rcs-1-gnc-simulator";
    charBlenderModelPath = "/home/peterc/devDir/projects-DART/data/rcs-1/pre-phase-A/blender/Apophis_RGB.blend";
    charBlenderPyInterfacePath          = fullfile(charRootPath, "lib/corto_PeterCdev/server_api/BlenderPy_UDP_TCP_interface.py" );
    charStartBlenderServerScriptPath    = fullfile(charRootPath, "lib/corto_PeterCdev/server_api/StartBlenderServer.sh");
else
    charRootPath = "/home/peterc/devDir/rendering-sw/corto_PeterCdev";
    charBlenderModelPath        = fullfile(charRootPath, "data/scenarios/S2_Itokawa/S2_Itokawa.blend");
    charBlenderPyInterfacePath  = fullfile(charRootPath, "server_api/BlenderPy_UDP_TCP_interface.py" );
    charStartBlenderServerScriptPath    = fullfile(charRootPath, "server_api/StartBlenderServer.sh");
end

% Set dump folder for processed images with keypoints
charScenePlotDumpFolder = "./.tmp_output_GT";
charAlgorithmDumpFolder = "./.tmp_output_KLT";

charInterfaceRootPath  = fileparts(charBlenderPyInterfacePath);
charConfigYamlPath     = fullfile(charInterfaceRootPath, "BlenderPy_UDP_TCP_CONFIG.yml");

charStartBlenderCommand = sprintf('bash %s -m "%s" -p "%s"', ...
    charStartBlenderServerScriptPath, charBlenderModelPath, charBlenderPyInterfacePath);

charServerAddress = 'localhost';
ui32ServerPort = [30001, 51000]; % [TCP, UDP]
ui32TargetPort = 51001; % UDP recv
dCommTimeout = 30;

% HARDCODED FOR RCS-1 CAMERA
i64RecvTCPsize = int64(4 * 2048 * 1536 * 64/8); % Number of bytes to read: 4*64*NumOfpixels

objBlenderPyCommManager = BlenderPyCommManager(charServerAddress, ui32ServerPort, dCommTimeout, ...
    'bInitInPlace', false, 'charBlenderModelPath', charBlenderModelPath, ...
    'bAutoManageBlenderServer', false, 'charBlenderPyInterfacePath', charBlenderPyInterfacePath, ...
    'charStartBlenderServerCallerPath', charStartBlenderServerScriptPath, ...
    'ui32TargetPort', ui32TargetPort, 'i64RecvTCPsize', i64RecvTCPsize, ...
    "charConfigYamlFilename", charConfigYamlPath);

% Get camera used for the dataset
objCameraIntrinsics = objBlenderPyCommManager.objCameraIntrinsics; 

%% ENVIRONEMNT SETUP
% Build shape model object with kernels path for scenarios-dependent shape model
charNavBackendKernelsPATH = '/home/peterc/devDir/nav-backend/simulationCodes/data/SPICE_kernels';
CSPICEkerLoader(charNavBackendKernelsPATH, enumTargetName_Shape); % DEVNOTE: currently required to load frames and ephemerides

% TODO, add "LoadDefaultScenarioData" method in CScenarioGenerator to set these

switch enumTargetName_Shape

    case "Apophis"
        % Load user configuration from RCS-1 simulator
        LoadUserConfig;
        addpath( genpath('/home/peterc/devDir/projects-DART/rcs-1-gnc-simulator') );
        addpath('/home/peterc/devDir/projects-DART/rcs-1-gnc-simulator');
        cd(fileparts(which(mfilename)))

        charRootPath                        = "/home/peterc/devDir/projects-DART/rcs-1-gnc-simulator";
        charBlenderModelPath                = "/home/peterc/devDir/projects-DART/data/rcs-1/pre-phase-A/blender/Apophis_RGB.blend";
        charBlenderPyInterfacePath          = fullfile(charRootPath, "lib/corto_PeterCdev/server_api/BlenderPy_UDP_TCP_interface.py" );
        charStartBlenderServerScriptPath    = fullfile(charRootPath, "lib/corto_PeterCdev/server_api/StartBlenderServer.sh");
    
        charShapeModelFile = fullfile(path_to_shape_models, "apophis_v233s7_vert2_new.mod.obj");

        strShapeModelData = struct();
        [strShapeModelData] = read_shape_model(strShapeModelData, "strShapeModel", ...
            fullfile(path_to_shape_models, "apophis_v233s7_vert2_new.mod.obj"), 1000);

        strShapeModelData = strShapeModelData.strShapeModel.model;
        % Define shapeModel struct for plot
        strShapeModel.ui32triangVertexPtr   = strShapeModelData.i32triangVertexPtrs;
        strShapeModel.dVerticesPos          = strShapeModelData.dVerticesPositions;

        % Define shape model object
        % TODO test this branch!
        objShapeModel = CShapeModel('struct', strShapeModel, 'm', 'm');


    case "Itokawa"

        charRootPath = "/home/peterc/devDir/rendering-sw/corto_PeterCdev";
        charBlenderModelPath        = fullfile(charRootPath, "data/scenarios/S2_Itokawa/S2_Itokawa.blend");
        charBlenderPyInterfacePath  = fullfile(charRootPath, "server_api/BlenderPy_UDP_TCP_interface.py" );
        charStartBlenderServerScriptPath    = fullfile(charRootPath, "server_api/StartBlenderServer.sh");

        charKernelname = fullfile(charNavBackendKernelsPATH, 'Itokawa/dsk/hay_a_amica_5_itokawashape_v1_0_64q.bds');
        % Define shape model object
        objShapeModel = CShapeModel('cspice', charKernelname, 'km', 'm');

        % Define shapeModel struct for plot
        strShapeModel.ui32triangVertexPtr   = objShapeModel.ui32triangVertexPtr;
        strShapeModel.dVerticesPos          = objShapeModel.dVerticesPos;

    case "Bennu_OREx"

        charRootPath = "/home/peterc/devDir/rendering-sw/corto_PeterCdev";
        charBlenderModelPath        = fullfile(charRootPath, "data/scenarios/S4_Bennu/S4_Bennu.blend");
        charBlenderPyInterfacePath  = fullfile(charRootPath, "server_api/BlenderPy_UDP_TCP_interface.py" );
        charStartBlenderServerScriptPath    = fullfile(charRootPath, "server_api/StartBlenderServer.sh");
        
        % Obj models, prefer kernels instead
        % charShapeModelFile = fullfile(charNavBackendKernelsPATH, 'Bennu_OREx/g_00880mm_alt_ptm_0000n00000_v020.obj'); % Too large!
        % charShapeModelFile = fullfile(charNavBackendKernelsPATH, 'Bennu_OREx/Bennu_v20_200k.obj');

        % charKernelname = fullfile(charNavBackendKernelsPATH, 'Bennu_OREx/dsk/bennu_l_00050mm_alt_ptm_5595n04217_v021.bds'); %  Too large!
        charKernelname = fullfile(charNavBackendKernelsPATH, 'Bennu_OREx/dsk/bennu_g_03170mm_spc_obj_0000n00000_v020.bds'); 
        % charKernelname = fullfile(charNavBackendKernelsPATH, 'Bennu_OREx/dsk/bennu_g_01680mm_alt_obj_0000n00000_v021.bds');
        
        % Define shape model object
        objShapeModel = CShapeModel('cspice', charKernelname, 'km', 'm');

        % Define shapeModel struct for plot
        strShapeModel.ui32triangVertexPtr   = objShapeModel.ui32triangVertexPtr;
        strShapeModel.dVerticesPos          = objShapeModel.dVerticesPos;

    otherwise
        error('Data for selected scenarios are either not setup or unavailable.');
end

% Include here paths to ensure overriding TBC
addpath(genpath('/home/peterc/devDir/SLAM-repos/MSCKF_for_SpaceNav/matlab'));

% DEVNOTE: the kernels loaded 
% TODO: move shape model and target definition inside this function
% ACHTUNG: timgegrid MUST be in Ephemeris Time (ET, for CSPICE)
dEphemeridesTimegrid = objDataset.dTimestamps + dET0;
[strDynParams, strMainBodyRefData] = DefineEnvironmentProperties(dEphemeridesTimegrid, ...
                                                                 enumTargetName_Dyn, ...
                                                                 charInertialFrame, ...
                                                                "objDataset", objDataset);



% Build ephemerides
ui32EphemerisPolyDeg = 15;
ui32AttitudePolyDeg  = 25;

[strDynParams, strMainBodyRefData] = EphemeridesDataFactory(dEphemeridesTimegrid, ...
                                                            ui32EphemerisPolyDeg,...
                                                            ui32AttitudePolyDeg, ...
                                                            strDynParams, ...
                                                            strMainBodyRefData, ...
                                                            "bGroundTruthEphemerides", false, ...
                                                            "bEnableInterpValidation", true);

objTargetEmulator = CTargetEmulator(objShapeModel, uint32(ui32NUM_POINTS_GT));

% Define performance monitor object to collect simulation data
ui32NumOfAdditionalStates   = 5;
ui32MAX_NUM_HISTORY_ENTRIES = length(objScenarioConfig.dTimestamps); % Each entry corresponds to a frame where a navigation solution is available
objPerfMonitor = PerformanceMonitorFactory(ui32MAX_NUM_HISTORY_ENTRIES, ...
                                           ui32NumOfAdditionalStates, ...
                                           "IN");

% Initialize simulation objects
% objCameraPose_J2000 = SPose3(objDataset.dPosSC_W(:,1), objDataset.dDCM_SCfromW(:,:,1)');
objTargetPose_J2000 = SPose3(objDataset.dTargetPosition_W(:,1), objDataset.dDCM_TBfromW(:,:,1)');

objCameraPose_J2000 = SNavState(objScenarioConfig.dInitialTime, ...
                                objDataset.dPosSC_W(:,objScenarioConfig.dInitialTimeID), ...
                                objDataset.dVelSC_W(:,objScenarioConfig.dInitialTimeID), ...
                                objDataset.dDCM_SCfromW(:,:, objScenarioConfig.dInitialTimeID)');

% DEVNOTE ACHTUNG: assumes velocity of the target is always ZERO!
objCameraPose_TB = SNavState(objScenarioConfig.dInitialTime, ...
                            objTargetPose_J2000.dDCM_FrameFromPoseFrame' * ...
                            ( objCameraPose_J2000.translation() - objDataset.dTargetPosition_W(:, objScenarioConfig.dInitialTimeID) ), ...
                            objTargetPose_J2000.dDCM_FrameFromPoseFrame' * ...
                            ( objCameraPose_J2000.dVelocity_Frame - [0;0;0] ), ...
                            objTargetPose_J2000.dDCM_FrameFromPoseFrame' * objDataset.dDCM_SCfromW(:,:, objScenarioConfig.dInitialTimeID)' );

objSimState = CSimulationState(objCameraPose_J2000, objTargetPose_J2000, objCameraPose_TB);

objSimState.i32GraphFeaturesIDs_GT = -ones(1, ui32MAX_GRAPH_FEATURES_SIZE);
objSimState.dTimestamp = objDataset.dTimestamps(1);

objSimState.dSunPosition_W = objDataset.dSunPosition_W(:, 1);

%% testScenarioGenerator_instantiation
% Construct scenario generator
objScenarioGenerator = CScenarioGenerator(  [objDataset.dPosSC_W(:,objScenarioConfig.dInitialTimeID)
                                             objDataset.dVelSC_W(:,objScenarioConfig.dInitialTimeID)], ...
                                            objScenarioConfig.dTimestamps, ...
                                            strDynParams, ...
                                            "bProvideAccelerationData", true);

%% testScenarioGenerator_generateData
objDatasetReGen = objScenarioGenerator.generateData();


% Plot trajectory
ui32LastTimeID = 1584;

figure;
plot3(objDataset.dPosSC_W(1, 1:ui32LastTimeID), ...
        objDataset.dPosSC_W(2, 1:ui32LastTimeID), ...
        objDataset.dPosSC_W(3, 1:ui32LastTimeID), 'b--', 'DisplayName', 'Kernels');
hold on
plot3(objDatasetReGen.dPosSC_W(1, 1:ui32LastTimeID), ...
        objDatasetReGen.dPosSC_W(2, 1:ui32LastTimeID), ...
        objDatasetReGen.dPosSC_W(3, 1:ui32LastTimeID), 'r-', 'DisplayName', 'ScenarioGen');
DefaultPlotOpts()
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
legend()
axis equal

% Check position difference
dRefDeltaPosition = objDataset.dPosSC_W - objDatasetReGen.dPosSC_W;

figure
subplot(3,1,1)
plot(objScenarioConfig.dTimestamps(1:ui32LastTimeID), dRefDeltaPosition(1, 1:ui32LastTimeID))
hold on
xlabel('Time [s]')
ylabel('X [m]')
DefaultPlotOpts()

subplot(3,1,2)
plot(objScenarioConfig.dTimestamps(1:ui32LastTimeID), dRefDeltaPosition(2, 1:ui32LastTimeID))
hold on
xlabel('Time [s]')
ylabel('Y [m]')
DefaultPlotOpts()

subplot(3,1,3)
plot(objScenarioConfig.dTimestamps(1:ui32LastTimeID), dRefDeltaPosition(3, 1:ui32LastTimeID))
hold on
xlabel('Time [s]')
ylabel('Z [m]')
DefaultPlotOpts()

% Check velocity difference
dRefDeltaVelocity = objDataset.dVelSC_W - objDatasetReGen.dVelSC_W;
figure
subplot(3,1,1)
plot(objScenarioConfig.dTimestamps(1:ui32LastTimeID), dRefDeltaVelocity(1, 1:ui32LastTimeID))
hold on
xlabel('Time [s]')
ylabel('X [m/s]')
DefaultPlotOpts()

subplot(3,1,2)
plot(objScenarioConfig.dTimestamps(1:ui32LastTimeID), dRefDeltaVelocity(2, 1:ui32LastTimeID))
hold on
xlabel('Time [s]')
ylabel('Y [m/s]')
DefaultPlotOpts()

subplot(3,1,3)
plot(objScenarioConfig.dTimestamps(1:ui32LastTimeID), dRefDeltaVelocity(3, 1:ui32LastTimeID))
hold on
xlabel('Time [s]')
ylabel('Z [m/s]')
DefaultPlotOpts()

