close all
clear
clc


% TEST SETUP
dInitialRelTimestamp = 0; % DEVNOTE: this is a necessary assumption in the current implementation
dFramesDeltaTime = 60;
dTotalTimeDuration = 8 * 86400;
dTimegridVect = dInitialRelTimestamp : dFramesDeltaTime : dInitialRelTimestamp + dTotalTimeDuration;

%% test_LoadReferenceDataRCS1
% Test original implementation tailored for RCS-1 mission
charKernelLengthUnits = 'km';
enumTrajName = EnumTrajectoryNames.RTO_4t1_J11p0;

objDataset  = LoadReferenceDataRCS1(enumTrajName, ...
                                    dTimegridVect, ...
                                    "J2000", "APOPHIS_FIXED", ...
                                    "bLoadManoeuvres", true, ...
                                    "charKernelLengthUnits", charKernelLengthUnits);


%% test_LoadReferenceDataFromKernels
% Test on FUTURE kernels
charKernelPathRoot = "/home/peterc/devDir/projects-DART/data/future/phase-C/kernels/mk/";
charCurrentDir = pwd;
cd(charKernelPathRoot)
cspice_furnsh('kernels.mk')
cd(charCurrentDir);

varTargetID                 = int32(-10003001);
enumTrajectKernelName       = "kernelFUT_h600";

enumWorldFrame              = "J2000";
varReferenceCentre          = "EARTH";
enumTargetFrame             = "IAU_EARTH"; 

charTrajKernelFolderPath    = "/home/peterc/devDir/projects-DART/data/future/phase-C/kernels/trajectories";
varTargetBodyID             = "MOON";

cellAdditionalTargetsID     = {};
cellAdditionalTargetNames   = {};

charKernelLengthUnits       = "km";
charOutputLengthUnits       = "km";


[objReferenceMissionData, dStateSC_TargetFixed, dSunPosition_TargetFixed] = LoadReferenceDataFromKernels(varTargetID, ...
                                                                                                       enumTrajectKernelName, ...
                                                                                                       dTimegridVect, ...
                                                                                                       enumWorldFrame, ...
                                                                                                       varReferenceCentre, ...
                                                                                                       enumTargetFrame, ...
                                                                                                       "bLoadManoeuvres", false, ...
                                                                                                       "cellAdditionalTargetsID", cellAdditionalTargetsID, ...
                                                                                                       "cellAdditionalTargetNames", cellAdditionalTargetNames, ...
                                                                                                       "charKernelLengthUnits", charKernelLengthUnits,...
                                                                                                       "charTrajKernelFolderPath", charTrajKernelFolderPath, ...
                                                                                                       "charOutputLengthUnits", charOutputLengthUnits, ...
                                                                                                       "varTargetBodyID", varTargetBodyID, ...
                                                                                                       "bIsTimeGridRelative", true);

%% test_BuildTimegridFromWindows
charRootPath = "/home/peterc/devDir/projects-DART/data/future/phase-C";
charJsonPath = fullfile(charRootPath, "acquisition_windows_data", "altitude_600km", "windows_600_01042027_VIS_Moon.json");
dDeltaTime   = 5;

[dTimeGrid, bInsideWindowMask] = BuildTimegridFromWindows(charJsonPath, ...
                                                        dDeltaTime, ...
                                                        "ui32NumStepsAfter", 5, ...
                                                        "ui32NumStepsBefore", 5, ...
                                                        "ui32MaxNumWindows", 1000, ...
                                                        "dMinWindowDuration", 30);

% Build dataset from kernel
charKernelPathRoot = "/home/peterc/devDir/projects-DART/data/future/phase-C/kernels/mk/";
charCurrentDir = pwd;
cd(charKernelPathRoot)
cspice_furnsh('kernels.mk')
cd(charCurrentDir);

varTargetID                 = int32(-10003001);
enumTrajectKernelName       = "kernelFUT_h600";

enumWorldFrame              = "J2000";
varReferenceCentre          = "EARTH";
enumTargetFrame             = "IAU_EARTH"; 

charTrajKernelFolderPath    = "/home/peterc/devDir/projects-DART/data/future/phase-C/kernels/trajectories";
varTargetBodyID             = "EARTH";

cellAdditionalTargetsID          = {"MOON"};
cellAdditionalTargetNames        = {};
bRequireAdditionalBodiesAttitude = true(1,1);
cellAdditionalTargetsFrames      = {"IAU_MOON"};

charKernelLengthUnits       = "km";
charOutputLengthUnits       = "km";

[objReferenceMissionData, dStateSC_TargetFixed, dSunPosition_TargetFixed] = LoadReferenceDataFromKernels(varTargetID, ...
                                                                                                       enumTrajectKernelName, ...
                                                                                                       dTimeGrid, ...
                                                                                                       enumWorldFrame, ...
                                                                                                       varReferenceCentre, ...
                                                                                                       enumTargetFrame, ...
                                                                                                       "bLoadManoeuvres", false, ...
                                                                                                       "cellAdditionalTargetsID", cellAdditionalTargetsID, ...
                                                                                                       "cellAdditionalTargetNames", cellAdditionalTargetNames, ...
                                                                                                       "charKernelLengthUnits", charKernelLengthUnits,...
                                                                                                       "charTrajKernelFolderPath", charTrajKernelFolderPath, ...
                                                                                                       "charOutputLengthUnits", charOutputLengthUnits, ...
                                                                                                       "varTargetBodyID", varTargetBodyID, ...
                                                                                                       "bIsTimeGridRelative", false, ...
                                                                                                       "bAdditionalBodiesRequireAttitude", bRequireAdditionalBodiesAttitude, ...
                                                                                                       "cellAdditionalTargetFrames", cellAdditionalTargetsFrames);

% Plot position and velocity in world frame
PlotDatasetPositionVelocity(objReferenceMissionData);


% Plot 3D trajectory in target fixed frame
[objSceneFig] = VisualizeTrajectory3dSceneAndPoses(objReferenceMissionData);

