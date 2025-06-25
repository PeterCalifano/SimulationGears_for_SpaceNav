close all
clear
clc


% TEST SETUP
dInitialRelTimestamp = 0; % DEVNOTE: this is a necessary assumption in the current implementation
dFramesDeltaTime
dTotalTimeDuration = 8 * 86400;
dTimegridVect = dInitialRelTimestamp : dFramesDeltaTime : dInitialRelTimestamp + dTotalTimeDuration;

%% test_LoadReferenceDataRCS1
% Test original implementation tailored for RCS-1 mission
charKernelLengthUnits = 'km';

objDataset  = LoadReferenceDataRCS1(enumTrajName, ...
                                    dTimegridVect, ...
                                    "J2000", "APOPHIS_FIXED", ...
                                    "bLoadManoeuvres", true, ...
                                    "charKernelLengthUnits", charKernelLengthUnits);


%% test_LoadReferenceDataFromKernels
% Test on FUTURE kernels

varTargetID                 = "";
enumTrajectKernelName       = "";

enumWorldFrame              = "J2000";
varReferenceCentre          = "EARTH";
enumTargetFrame             = "ECI"; 

charTrajKernelFolderPath    = "";
varTargetBodyID             = "";

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

