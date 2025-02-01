close all
clear
clc

%% Moon orbit simulation scenario setup
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-12-2023    Pietro Califano     Basic setup of SPICE and Simulink model.
% 11-12-2023    Pietro Califano     Added call to Blender for testing.
%                                   Image sequence generation verified.
% 15-12-2023    Pietro Califano     Added settings for full orbit
%                                   propagation model.
% 31-01-2024    Pietro Califano     Interface Simulator to Blender; added orbit of hypothetical LNS
%                                   satellite from Zanotti et al. 2023
% 28-04-2024    Pietro Califano     Update of call to Blender according to new API and attitude
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% planet3D()
% SPICE kernels
% -------------------------------------------------------------------------------------------------------------

%% SPICE INITIALIZATION
cspice_kclear();

% Lunar Reconnaissance Orbiter kernel
LRO_KERNELS_PATH = 'C:\\Users\\pietr\\OneDrive - Politecnico di Milano\\PoliMi - LM\\MATLABwideCodes\\SPICE_KERNELS\\LRO_kernels\\';
cspice_furnsh(strcat(LRO_KERNELS_PATH, 'lrorg_2023166_2023258_v01.bsp'));
cspice_furnsh(strcat(LRO_KERNELS_PATH, 'moon_pa_de440_200625.bpc'));
cspice_furnsh(strcat(LRO_KERNELS_PATH, 'moon_de440_200625.tf'));

% Default kernels loading
cspice_furnsh(fullfile(which('SPICE_MK_DEFAULT.tm')))
% cspice_ktotal('all');

%% OPTIONS 
model_name = "GNCsimulator_MoonOrbit";
bCALL_SIMULINK_MODEL = true;

bENABLE_3rdBodyAccel_EARTH = true;
bENABLE_3rdBodyAccel_SUN = true;
bENABLE_SHE_MOON = true;
bENABLE_3rdBodyAccel_VENUS = false;
bENABLE_CannonBallSRPaccel = true;

dReflectivityCoeff = 1.5;

hours2sec = 3600;
SIM_TIME = 0.2*24; % [hr]
plotEarth = false; 
LU = 'km';
TU = 's';

%% SCENARIO PARAMETERS 
% REFERENCE FRAMES SPICE NAMES
MoonInertialFrame = 'MOON';
MoonFixedFrame = 'MOON_PA_DE440';
EarthInertialFrame = 'J2000';
EarthFixedFrame = 'IAU_EARTH';
GlobalFrame = 'J2000';

% TODO: generate reduced mat file up to [50x50]
% Starting from (1, 1) as first row

% SPICE Time
dateString = 'Aug 30 9:45:10 PDT 2023';

et0 = cspice_str2et(dateString);
deltaET = hours2sec * SIM_TIME; % [s]
etSpan = et0:1:et0+deltaET;

bSHOW_IMAGE_SEQUENCE = true;

% gravityData = fopen('gggrx_1200a_sha.dat', 'r');
% [dataArray, dataFieldsCount] = fscanf(gravityData, '%d,%d,%f,%f,%f,%f');

% MOON
bENABLE_UNSCALING = true;
ui16lMax = 80;
modelCoeffFilePath = "GRGM1200A_84x84_Normalized.mat";

[i_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffFilePath, ui16lMax, bENABLE_UNSCALING);

R_Moon = mean(cspice_bodvrd('MOON', 'RADII', 3)); % [km]
R_Moon_SHref = 1738.0; % [km]

mu_Moon = cspice_bodvrd('MOON', 'GM', 1); % [km^3/s^2]
mu_Moon_SHref = 4902.80011526323; % [km^3/s^2]
maxSHEdegree = 30;

dDCM_Moon_fromTBtoIN = cspice_pxform(MoonFixedFrame, GlobalFrame, etSpan);

% EARTH
% Gravity model loading
mu_Earth = cspice_bodvrd('EARTH', 'GM', 1); % [km^2/s^3]
i_dEPH_EARTH = cspice_spkezr('EARTH', etSpan, GlobalFrame, 'NONE', MoonInertialFrame);

% dDCM_Earth_fromINtoTB = cspice_pxform('J2000', 'EARTH_FIXED', etSpan);

% SUN
mu_Sun = cspice_bodvrd('SUN', 'GM', 1); % [km^2/s^3]
i_dEPH_SUN = cspice_spkezr('SUN', etSpan, GlobalFrame, 'NONE', MoonInertialFrame);

% LRO for validation
% xState_LRO = cspice_spkezr('LUNAR RECONNAISSANCE ORBITER', etSpan, GlobalFrame, 'NONE', MoonInertialFrame);

%% SIMULINK MODEL SETUP
% Initial conditions
SMA   = 9750.7; % [km]
ECC   = 0.583;
% INCL  = deg2rad(51.01);
INCL  = deg2rad(80);
RAAN  = deg2rad(250);
OMEGA = deg2rad(90);
THETA = deg2rad(55);

SatID5A_keplParams = [SMA, ECC, INCL, RAAN, OMEGA, THETA]; % From Zanotti et al. 2023
xSCref0 = kepl2rv(SatID5A_keplParams, mu_Moon);

% Temporary assignment
muCB = mu_Moon;
muSB = mu_Earth;

SIM_TIME = deltaET; % [s]
simTspan = 0:1:deltaET; % Timegrid in [s]

% Prepare input data "From Workspace"
% REFERENCE ORBITAL Ephemerides
EARTH_EPH.time = simTspan';
EARTH_EPH.signals.values = i_dEPH_EARTH(1:3, :)';
EARTH_EPH.signals.dimensions = 3;

SUN_EPH.time = simTspan';
SUN_EPH.signals.values = i_dEPH_SUN(1:3, :)';
SUN_EPH.signals.dimensions = 3;

% REFERENCE ATTITUDE Ephemerides
dDCM_Moon_fromTBtoIN_SLX.time = simTspan';
dDCM_Moon_fromTBtoIN_SLX.signals.values = reshape(dDCM_Moon_fromTBtoIN, 9, [])';
dDCM_Moon_fromTBtoIN_SLX.signals.dimensions = 9;


if bCALL_SIMULINK_MODEL == true
    %% SIMULINK MODEL SIMULATION

    simInputObj = Simulink.SimulationInput(model_name);
    simInputObj = simInputObj.setModelParameter('StopTime', num2str(SIM_TIME));

    open(model_name);
    simOutputObj = sim(simInputObj);
    simOutputObj.SimulationMetadata.TimingInfo

    %% TEMPORARY: Simulator validation code
    xStateRef_SimOut = simOutputObj.xStateRef.Data;

    % optsMoon.Units = LU;
    % planet3D('Moon', optsMoon);
    % hold on;
    % if plotEarth == true
    %     optsEarth.Units = optsMoon.Units;
    %     optsEarth.Position = [384000; 0; 0];
    %     planet3D('Earth', optsEarth);
    % end
    % 
    % % LRO trajectory
    % plot3(xState_LRO(1, :), xState_LRO(2, :), xState_LRO(3, :), ...
    %     'b-', 'linewidth', 1.02, 'DisplayName', 'LRO trajectory');
    % 
    % % Plot options
    % DefaultPlotOpts();
    % xlabel(strcat("X [",LU, "]"));
    % ylabel(strcat("Y [",LU, "]"));
    % zlabel(strcat("Z [",LU, "]"));
    % hold on;
    % 
    % % Plot propagated orbit
    % plot3(xStateRef_PLOT(:, 1), xStateRef_PLOT(:, 2), xStateRef_PLOT(:, 3), ...
    %     'r-', 'linewidth', 1.02, 'DisplayName', 'LRO trajectory re-prop.');
    % 
    % % Evaluate state error
    % errReprop = xStateRef_PLOT' - xState_LRO;
    % posErrNorm = vecnorm(errReprop(1:3, :), 2, 1); % [km]
    % 
    % posErrPlot = figure;
    % plot(simTspan, errReprop(1, :), 'LineWidth', 1.02, 'Color', 'r');
    % hold on;
    % plot(simTspan, errReprop(2, :), 'LineWidth', 1.02, 'Color', 'g');
    % plot(simTspan, errReprop(3, :), 'LineWidth', 1.02, 'Color', 'b');
    % plot(simTspan, posErrNorm, 'LineWidth', 1.05, 'Color', 'k');
    % 
    % DefaultPlotOpts();
    % xlabel(strcat("Time [",TU, "]"));
    % ylabel(strcat("Position error [",LU, "]"));

end


%% TEST: Call to Blender
ImgAcquisitionDeltaTime = 10; % [s]
timegrid = 0:ImgAcquisitionDeltaTime:deltaET;
timeETgrid = timegrid + et0;
i_ui16Nimages = length(timegrid);

% Determine ID of State corresponding to image acquisition times
idStateVec = (0:ImgAcquisitionDeltaTime:size(xStateRef_SimOut, 1)) + 1;
xState_toBlender = xStateRef_SimOut(idStateVec, :)';

% Get ephemerides of Sun and Moon attitude
i_drSun = cspice_spkpos('SUN', timeETgrid, 'J2000', 'NONE', 'MOON');
DCM_fromJ2000toMoonTB = cspice_pxform('J2000', 'MOON_PA_DE440', timeETgrid);

% Convert to Right-Handed Scalar Last quaternion TB to IN
o_dqSVRPplus_fromINtoTB = DCM2quatSeq(DCM_fromJ2000toMoonTB, false);
o_dqSVRPplus_fromTBtoIN = qInvert(o_dqSVRPplus_fromINtoTB, false);

% Assign inputs and convert to BU for S6_MOON
BlenderModelScaleCoeff = 1;
% km2BU = BlenderModelScaleCoeff /(2*R_Moon); % [BU]
km2BU = BlenderModelScaleCoeff/1000; % [BU]

i_drCam = xState_toBlender(1:3, :)' * km2BU;
i_drTargetBody = repmat([0, 0, 0], i_ui16Nimages, 1);
i_drSun = i_drSun' * km2BU; 
SunPhaseAngle = zeros(i_ui16Nimages, 1);

cmap = autumn(length(timeETgrid));
clf(figure(1))

o_dqSVRPminus_fromInvCAMtoIN = zeros(length(timeETgrid), 4);
o_dqSVRPplus_fromCAMtoIN     = zeros(length(timeETgrid), 4);

for idt = 1:1:length(timeETgrid)
   
    % TO BLENDER
    o_dqSVRPminus_fromInvCAMtoIN(idt, :)  = simulateTBpointing_PosOnly(xState_toBlender(1:3, idt), ...
        [0; 0; 0], false, true, true);
    
    % REFERENCE
    o_dqSVRPplus_fromCAMtoIN(idt, :) = simulateTBpointing_PosOnly(xState_toBlender(1:3, idt), ...
        [0; 0; 0], false, false, false);

    % DEVNOTE: this is equivalent to inverting the quaternion! --> TBC: Blender wants the quaternion
    % corresponding to the DCM from IN to CAMERA not the opposite (what I was generating and passing to it)
    % Namely, a LEFT-Handed QUATERNION (Hamilton)!

    Dir_D1toSun = i_drSun(idt, :)./norm(i_drSun(idt, :));

    % Compute Sun Phase angle to check illumination
    SunPhaseAngle(idt) = acosd( dot(xState_toBlender(1:3, idt)./norm(xState_toBlender(1:3, idt)), Dir_D1toSun) );

    % Recompute camera boresight to check correct pointing
    DCM_fromCAMtoIN = Quat2DCM(o_dqSVRPplus_fromCAMtoIN(idt, :), false); 
    camBoresightDir = DCM_fromCAMtoIN(:, end);

    % TODO: MAKE A FUNCTION TO USE THIS SCRIPT CODE
    if mod(idt, 2) == 0
        figure(1)
        hold on;
        quiver3(0, 0, 0, Dir_D1toSun(1), Dir_D1toSun(2), Dir_D1toSun(3), 1.5*R_Moon,...
            'Color', cmap(idt, :), 'DisplayName', 'Sun Dir.', 'LineWidth', 1.05);

        posNormTmp = norm(xState_toBlender(1:3, idt));

        % Define vectors to plot
        i_dArrowLength = 0.5*R_Moon;
        Zvec = (i_dArrowLength + posNormTmp/2) * camBoresightDir;

        % Plot vectors
        quiver3(xState_toBlender(1, idt), xState_toBlender(2, idt), xState_toBlender(3, idt), ...
            Zvec(1), Zvec(2), Zvec(3), 1, 'DisplayName', 'Cam. Z', 'LineWidth', 1.05);

        plot3(xState_toBlender(1, idt), xState_toBlender(2, idt), xState_toBlender(3, idt), 'k*', ...
            'LineStyle', 'none', 'MarkerSize', 5, 'LineWidth', 1.3)
    end
end

% Plot Moon
optsMoon.Units = LU;
planet3D('Moon', optsMoon);
DefaultPlotOpts();


%% BLENDER Input definition
CameraData.fov = 20;
% DVS346: 346 x 260
CameraData.resx = 346;
CameraData.resy = 260;

BlenderOpts.encoding = 8;
BlenderOpts.rendSamples = 64;
BlenderOpts.scene_viewSamples = 64;
BlenderOpts.scattering = 0;

SceneData.scenarioName = "S6_Moon";
SceneData.rStateCam = i_drCam; % In TF frame
SceneData.rTargetBody = i_drTargetBody; % In TF frame
SceneData.rSun = i_drSun; % In TF frame
SceneData.qFromINtoCAM = o_dqSVRPminus_fromInvCAMtoIN; % Defined left handed
SceneData.qFromINtoTF = o_dqSVRPplus_fromINtoTB'; % Left handed given as Right Handed "inverted"

i_bVERBOSE_MODE = true;
i_bCALL_BLENDER = true;
i_bRUN_IN_BACKGROUND = true;

i_ui16Nposes = size(i_drCam, 1);

% o_strConfigJSON NOT ASSIGNED INSIDE!
% Allocate dimensional data as Reference: [km]
% MoonDiamInBlender = 3474.84; % [km]
scaleBU2km = 1000;

drStateCam_IN = scaleBU2km * i_drCam;

ReferenceData.drStateCam_IN = drStateCam_IN; % [km]
ReferenceData.dqSVRPplus_fromCAMtoIN = o_dqSVRPplus_fromCAMtoIN; % Not in Blender convention (inverted Z)
ReferenceData.dqSVRPplus_fromTBtoIN = o_dqSVRPplus_fromTBtoIN;
% ReferenceData.dAzCAM_TF = AZ_Scatter; % [deg]
% ReferenceData.dElCAM_TF = EL_Scatter; % [deg]
ReferenceData.dSunDir_IN = i_drSun./vecnorm(i_drSun, 2, 2); % [deg]

save('Input2CORTO_Labels_fullWS')

i_cPath2BlenderExe = '';
i_cPath2CORTO = 'C:\devDir\corto_PeterCdev';
[o_strConfigJSON, o_ui8ImgSequence, o_cOutputPath] = MATLAB_BlenderCORTO_API( ...
    i_ui16Nposes,  ...
    SceneData, ...
    CameraData, ...
    BlenderOpts, ...
    ReferenceData, ...
    i_bCALL_BLENDER, ...
    i_cPath2CORTO, ...
    i_cPath2BlenderExe, ...
    i_bRUN_IN_BACKGROUND, ...
    i_bVERBOSE_MODE);

if bSHOW_IMAGE_SEQUENCE == true
    % Check images
    figure;
    for id = 1:i_ui16Nimages
        imshow(o_ui8ImgSequence(:, :, id))
        pause(0.001)
    end
end

%% Generate video from image sequence
folderPath = fullfile(o_cOutputPath);
% imageSeq2Video(imgpath, 'Moon_LROscaled_24hours', i_ui16Nimages);
% imageSeq2Video(imgpath, 'Moon_5days_sat5A_MilaniCAM_Zanotti2022', i_ui16Nimages);

fileName = 'Moon_1.5days_TestForEvents_dvs346';
imageSeq2Video(imgpath, fileName);

