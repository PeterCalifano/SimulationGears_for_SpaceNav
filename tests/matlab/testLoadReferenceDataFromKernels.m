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
                                                                                                       "AdditionalTargetsID", cellAdditionalTargetsID, ...
                                                                                                       "AdditionalTargetNames", cellAdditionalTargetNames, ...
                                                                                                       "charKernelLengthUnits", charKernelLengthUnits,...
                                                                                                       "charTrajKernelFolderPath", charTrajKernelFolderPath, ...
                                                                                                       "charOutputLengthUnits", charOutputLengthUnits, ...
                                                                                                       "varTargetBodyID", varTargetBodyID);

% Run visualizer of datasets
% TODO

