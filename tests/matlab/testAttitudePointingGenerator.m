close all
clear
clc


% TEST SETUP
ui32NumOfPoses          = 2e3;
dMinMaxRange            = [1200, 4e3];
dMinMaxAzimuth          = [-110, 110];
dMinMaxElevation        = [-85, 85];
dMinMaxFrame0RotZangle  = [0, 180];

% Scattering from spherical (uniform in spherical, not in R3 space)
% objPointCloudDataset = CRelativePoseUniformSampler.ScatterPositionsFromSphericalStatic(ui32NumOfPoses, ...
%                                                                                 dMinMaxRange, ...
%                                                                                 dMinMaxAzimuth, ...
%                                                                                 dMinMaxElevation, ...
%                                                                                 dMinMaxFrame0RotZangle, ...
%                                                                                 "bInputInDegrees", true, ...
%                                                                                 "dScalingDistance", 1e3);
% 
% figure;
% scatter3(objPointCloudDataset.dFrame1Position_W(1,:), objPointCloudDataset.dFrame1Position_W(2,:), ...
%             objPointCloudDataset.dFrame1Position_W(3,:), 10, 'b', 'Marker', '.');
% axis equal

% Uniform scatter
objPointCloudDataset = CRelativePoseUniformSampler.UniformScatterPositionsFromSphericalStatic(ui32NumOfPoses, ...
                                                                                            dMinMaxRange, ...
                                                                                            dMinMaxAzimuth, ...
                                                                                            dMinMaxElevation, ...
                                                                                            dMinMaxFrame0RotZangle, ...
                                                                                            "bInputInDegrees", true, ...
                                                                                            "dScalingDistance", 1e3, ...
                                                                                            "bUniformlyScatterFrame0Attidude", true);

figure;
scatter3(objPointCloudDataset.dFrame1Position_W(1,:), objPointCloudDataset.dFrame1Position_W(2,:), ...
            objPointCloudDataset.dFrame1Position_W(3,:), 30, 'b', 'Marker', '.');
axis equal

% Assign data buffers
dSunVector_Buffer_W             = objPointCloudDataset.dLightPosition_W; % Scale does not matter here.
dCameraOrigin_Buffer_W          = objPointCloudDataset.dFrame1Position_W; % Do not scale, model is in Blender meters (scaled by 1000)
dBodiesOrigin_Buffer_W          = objPointCloudDataset.dFrame0Position_W;
dBodiesAttDCM_Buffer_WfromOF    = pagetranspose( objPointCloudDataset.dDCM_Frame0FromW );

% Generate attitude assuming nadir pointing + PV panels axis constraint + random gaussian rotation around the Sun direction
dSigmaRotationAroundSun = 25; % [deg]

objPointingGenerator = CAttitudePointingGenerator(dCameraOrigin_Buffer_W, ...
                                                    dBodiesOrigin_Buffer_W, ...
                                                    dSunVector_Buffer_W, "bShowAttitudePointingPlot", true);
return
%% testLegacyMethods
[objPointingGenerator, dDCM_FrameFromPoseFrame]   = objPointingGenerator.pointToTarget_SunDirConstraint("dSigmaDegRotAboutBoresight", dSigmaRotationAroundSun);

return
%% test_pointToTarget_bare
[objPointingGenerator, dCameraAttDCM_Buffer_WfromOF]   = objPointingGenerator.pointToTarget();

return

%% test_pointToTarget_boresightRoll
[objPointingGenerator, dCameraAttDCM_Buffer_WfromOF]   = objPointingGenerator.pointToTarget("dSigmaDegRotAboutBoresight", dSigmaRotationAroundSun); %#ok<*UNRCH>

return

%% test_pointToTarget_offPointing
[objPointingGenerator, dRot3Param, dCameraAttDCM_Buffer_WfromOF, dOffPointingAngles]   = objPointingGenerator.pointToTarget("dSigmaDegRotAboutBoresight", 20, ...
                                                                                                                            "enumOffPointingMode", "randomAxis", ...
                                                                                                                            "dSigmaOffPointingDegAngle", 5); %#ok<*UNRCH>

figure;
histogram(dOffPointingAngles, "BinWidth", 0.2);
xlabel('Off-pointing half-cone angle [deg]')
return










