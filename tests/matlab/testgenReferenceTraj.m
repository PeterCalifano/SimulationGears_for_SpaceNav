close all
clear
clc

addpath("/home/peterc/devDir/MATLABcodes/mathUtils/interpolation")
%addpath("/home/peterc/devDir/MATLABcodes/simulationModels/dynamicsModels")
addpath("/home/peterc/devDir/nav-backend/simulationCodes/matlab/simulation")

DEBUG_BUILD = true;
POSVEC_POLYDEG_GT = 22;
ATTQUAT_POLYDEG_GT = 25;
SIM_STOPTIME = 1.5*24*3600;

%% TEST script for gerReferenceTraj function
% Created by PeterC, 05-07-2024, for implementation testing and validation of trajectory and attitude
% generation function with Spherical harmonics and SRP.

% NOTE: for TODO look for the expression as comment in this file
% ACHTUNG: quaternion interpolant stops working ok over 3 days! Max error is large. Likely there is a
% discontinuity that makes the discontinuity check to fail.
% How to address: simply stop interpolating the quaternion and use axis-angle representation. Axis is
% constant, angle is interpolated.


scenario_name = CScenarioName.Itokawa_Hayabusa1;

kernelsPATH = '/home/peterc/devDir/nav-backend/simulationCodes/matlab/SPICE_kernels';

projectDir = pwd;
cd(fullfile(kernelsPATH, "common"));
cspice_furnsh('mkcommon.mk');
cd(projectDir);

switch scenario_name
    case  CScenarioName.Didymos_Hera

        kernelsPATH = fullfile(kernelsPATH, "hera_v110");

        % Frame names definition
        TARGET_NAME = 'DIDYMOS';
        FixedFrame = 'DIDYMOS_FIXED';
        InertialFrame = 'ECLIPJ2000';

        dTARGET_GM_DEFAULT
        dTARGET_RADIUS_DEFAULT = 0.39*1000; % [m]


        MKfilename = 'hera_study_PO_EMA_2024_v110_20211109_001.tm';
        mkpath = fullfile(kernelsPATH, 'mk', MKfilename);
        MilaniSPKfileName = fullfile(kernelsPATH, 'spk/HERA_MILANI_sc_V6_20270326_20270627_v01.bsp');

        % Load SPICE kernels
        projectDir = pwd;

        cd(fullfile(kernelsPATH, 'mk'))
        cspice_furnsh( char(MKfilename) );
        cd(projectDir)

        % kernelsPATH = '/home/peterc/devDir/nav-backend/customExamples/matlab/SPICE_kernels/Milani_KERNELS';


        % NOTE: to make brief work, need to add mice to the system path!

        %% OPTIONS
        % EXAMPLE use of spkcov
        % The ID of is simply the ID of the manoeuvre in the plan, progressive from zero most likely
        maxSize = 1000; % Max size of the output array, determining how many manoeuvre can be read.

        % Get ID of the objects in the SPK kernel
        objID = cspice_spkobj(MilaniSPKfileName, 1000);

        manoeuvresTimesET = cspice_spkcov(MilaniSPKfileName, objID, maxSize);
        [manCount, ~] = size(manoeuvresTimesET);

        % Get manoeuvre times for each entry
        fprintf('Coverage for object %d\n', objID);
        for j = 1:2:manCount

            timstr = cspice_et2utc(manoeuvresTimesET(j:j+1)', 'c',0);
            fprintf('   Start: %s\n', timstr(1,:));
            fprintf('    Stop: %s\n', timstr(2,:));

        end

        i_strDynParams.strMainData.dSHcoeff = [];
        i_strDynParams.strMainData.ui16MaxSHdegree = [];
        ET0 = cspice_str2et('Feb 15 2027 00:00:00');


    case CScenarioName.Itokawa_Hayabusa1

        projectDir = pwd;
        kernelsPATH = fullfile(kernelsPATH, "Itokawa_Hayabusa1/mk");

        cd(kernelsPATH)
        tmpKernelsPATH = char("metakernel.mk");
        cspice_furnsh(tmpKernelsPATH);
        cd(projectDir);


        % Frame names definition
        TARGET_NAME = 'ITOKAWA';
        FixedFrame = 'Itokawa_fixed'; % Check corresponding tf file
        InertialFrame = 'ECLIPJ2000';

        dTARGET_GM_DEFAULT = 2.36; % m^3/s^2 % From (Scheeres, 2006)
        dTARGET_RADIUS_DEFAULT = 1000*0.161915; % [m] ACHTUNG: Value used for Gravity SH expansion!

        ET0 = cspice_str2et('Sept 11 2005 00:00:00'); % Taken from Hayabusa mission
        % State in [m, m/s]
        xState0 = 1E3 * [0.800000000000000; -0.400000000000000; -0.400000000000000; 0; 4.907815108311225e-05; 0]; % Provided by Alban

        % Itokawa Spherical harmonics expansion coefficients (Scheeres, 2006)
        ClmSlm_normalized = [0.0,       0.0;
            -0.145216,  0.0;
            0.0,        0.0;
            0.219420,   0.0;
            0.036115,   0.0;
            -0.028139,  -0.006137;
            -0.046894,  -0.046894;
            0.069022,   0.033976;
            0.087852,   0.0;
            0.034069,   0.004870;
            -0.123263,  0.000098;
            -0.030673,  -0.015026;
            0.150282,   0.011627];

        % Compute "un-normalization" factors
        i_strDynParams.strMainData.ui16MaxSHdegree = uint16(4);

        scaleFactors = ExtSHE_normFactors(i_strDynParams.strMainData.ui16MaxSHdegree);
        % Compute unnormalized coefficients
        i_strDynParams.strMainData.dSHcoeff = ClmSlm_normalized./scaleFactors;
         

end


%assert(SIM_STOPTIME <= nextManET)
%assert(SIM_STOPTIME >= prevManET)

ET_SPAN = ET0:ET0 + SIM_STOPTIME;

i_ui8PolyDeg_attQuat = ATTQUAT_POLYDEG_GT;
i_ui8PolyDeg = POSVEC_POLYDEG_GT;

%% Reference trajectory generation
% NOTE: All quaternions are right-handed, scalar first.
i_bENABLE_AUTO_CHECK = true;

% ACHTUNG: do not use absolute ET as interpolation domain as this determines divergence of the polynomials
i_dInterpDomain = ET_SPAN - ET_SPAN(1);
i_dDomainLB = min(i_dInterpDomain);
i_dDomainUB = max(i_dInterpDomain);

% Spacecraft data (from MA_I4R1)
i_strDynParams.strSCdata.dReflCoeff = 1.25;  % Global CR 
i_strDynParams.strSCdata.dSCmass    = 12; % [kg]
i_strDynParams.strSCdata.dA_SRP     = 0.51; % [m^2]

% Main body data
% Attitude
% [o_dChbvCoeffs, o_dScaledInterpDomain, o_strfitStats] = fitChbvPolynomials(i_ui8PolyDeg, i_dInterpDomain, ...
%     i_dDataMatrix, i_dDomainLB, i_dDomainUB, i_bENABLE_AUTO_CHECK)

% cspice_spkcov(spkFileName, )
strMainBodyRefData.dDCM_INfromTB = cspice_pxform(FixedFrame, InertialFrame, ET_SPAN);
% strMainBodyRefData.dSHcoeff

strMainBodyRefData.dQuat_INfromTB = DCM2quatSeq(strMainBodyRefData.dDCM_INfromTB, false);

[strMainBodyRefData.dChbvCoeffs, dScaledInterpDomain, strMainBodyRefData.dswitchIntervals, ...
    strMainBodyRefData.strfitStats] = fitAttQuatChbvPolynmials( ...
                                        i_ui8PolyDeg_attQuat, ...
                                        i_dInterpDomain, ...
                                        strMainBodyRefData.dQuat_INfromTB, ...
                                        i_dDomainLB, ...
                                        i_dDomainUB, ...
                                        i_bENABLE_AUTO_CHECK);

i_strDynParams.strMainData.strAttData.ui8PolyDeg           = i_ui8PolyDeg_attQuat;
i_strDynParams.strMainData.strAttData.dChbvPolycoeffs      = strMainBodyRefData.dChbvCoeffs;
i_strDynParams.strMainData.strAttData.dsignSwitchIntervals = strMainBodyRefData.dswitchIntervals;
i_strDynParams.strMainData.strAttData.dTimeLowBound        = i_dDomainLB;
i_strDynParams.strMainData.strAttData.dTimeUpBound         = i_dDomainUB;


try
    i_strDynParams.strMainData.dGM = cspice_bodvrd(TARGET_NAME, 'GM', 1)*1E9; % [m^2/s^3]
catch
    warning('Main body GM not found in SPICE KERNELS. Using manually set value...')
    i_strDynParams.strMainData.dGM = dTARGET_GM_DEFAULT;
end

try
    error('FORCE TO USE MANUAL FOR NOW')
    i_strDynParams.strMainData.dRefRadius = mean(cspice_bodvrd(TARGET_NAME, 'RADII', 3)); % [m^2/s^3]
catch
    warning('Main body RADII not found in SPICE KERNELS. Using manually set value...')
    i_strDynParams.strMainData.dRefRadius = dTARGET_RADIUS_DEFAULT;
end

% 3rd bodies
% SUN Orbit
strMainBodyRefData.dSunPosition_IN = 1000 * cspice_spkpos('SUN', ET_SPAN, 'ECLIPJ2000', 'none', TARGET_NAME);

% ACHTUNG: Make sure that unit of measure for distance matches.

distFromSunAU = mean(vecnorm(strMainBodyRefData.dSunPosition_IN, 2, 1), "all") /150e9; % Input distance in [m]
i_strDynParams.strSRPdata.dP_SRP = 1367/physconst('lightspeed') * (1/(distFromSunAU)^2); % [N/m^2]

% Printing of key value: Distance in AU from the SUN
fprintf('\nAverage distance from the SUN in AU over ET_SPAN: %3.4f AU\n', distFromSunAU);

[o_dChbvCoeffs, ~, ~] = fitChbvPolynomials(i_ui8PolyDeg, i_dInterpDomain, ...
    strMainBodyRefData.dSunPosition_IN, i_dDomainLB, i_dDomainUB, i_bENABLE_AUTO_CHECK);

i_strDynParams.strBody3rdData(1).strOrbitData.ui8PolyDeg      = i_ui8PolyDeg;
i_strDynParams.strBody3rdData(1).strOrbitData.dChbvPolycoeffs = o_dChbvCoeffs;
i_strDynParams.strBody3rdData(1).strOrbitData.dTimeLowBound   = i_dDomainLB;
i_strDynParams.strBody3rdData(1).strOrbitData.dTimeUpBound    = i_dDomainUB;

i_strDynParams.strBody3rdData(1).dGM = cspice_bodvrd('SUN', 'GM', 1); % TODO: check measurement unit



% Dimorphos Orbit 
% [o_dChbvCoeffs, o_dScaledInterpDomain, o_strfitStats] = fitChbvPolynomials(i_ui8PolyDeg, i_dInterpDomain, ...
%     i_dDataMatrix, i_dDomainLB, i_dDomainUB, i_bENABLE_AUTO_CHECK)

% strMainBodyRefData.dDimorphosPosition_IN = cspice_spkpos('DIMORPHOS', ET_SPAN, 'ECLIPJ2000', 'none', 'DIDYMOS');
% 
% [o_dChbvCoeffs, ~, ~] = fitChbvPolynomials(i_ui8PolyDeg, i_dInterpDomain, ...
%     strMainBodyRefData.dDimorphosPosition_IN, i_dDomainLB, i_dDomainUB, i_bENABLE_AUTO_CHECK);
% 
% i_strDynParams.strBody3rdData(2).strOrbitData.ui8PolyDeg      = i_ui8PolyDeg;
% i_strDynParams.strBody3rdData(2).strOrbitData.dChbvPolycoeffs = o_dChbvCoeffs;
% i_strDynParams.strBody3rdData(2).strOrbitData.dTimeLowBound   = i_dDomainLB;
% i_strDynParams.strBody3rdData(2).strOrbitData.dTimeUpBound    = i_dDomainUB;
% 
% i_strDynParams.strBody3rdData(2).dGM = cspice_bodvrd('DIMORPHOS', 'GM', 1)*1E9;


% Simulation scenario configuration
strScenConfig.strInitConditions.xState0 = xState0;
strScenConfig.dSimOptions.odeopts = odeset('RelTol', 1E-12, 'AbsTol', 1E-12);
% strScenConfig.dSimOptions.dSigmaAKE

strScenConfig.deltaTime = 1;
strScenConfig.finalTime = SIM_STOPTIME;

% SPACECRAFT
% Milani data (TO CHANGE)
% dSunlitAreaSRP = ; % [km^2]
% dMassSC = ; % [kg]
% DragCoefficient = ; % [-]
% dReflectivityCoeff = ; % [-]

% FUTURE data
% dSunlitAreaSRP = 0.5032/1E6; % [km^2]
% dMassSC = 14.5; % [kg]
% DragCoefficient = 2.2; % [-]
% dReflectivityCoeff = 1.85; % [-]

NtimeInstants = length(ET_SPAN);

% Function handle for ode integrator
strScenConfig.dynFuncHandle = @(i_dStateTimetag, i_dxState_IN) computeRefDynFcn(i_dStateTimetag,...
                                                                                i_dxState_IN,...
                                                                                i_strDynParams);
strScenConfig.dynParams = i_strDynParams;

%% TEST CALL
[o_dXSC_ref, o_dTimestamps, o_dqCAMwrtIN_ref, o_dqCAMwrtIN_est] = GenerateReferenceTraj(strScenConfig, DEBUG_BUILD);

% Plot propagation


