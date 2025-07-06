classdef CScenarioGenerator < CGeneralPropagator
    %% DESCRIPTION
    % Generator class constructing dataset object to define 3D scene over time, with spacecraft trajectory and 
    % attitude, Sun position, target position and attitude, ephemerides of additional bodies. Acceleration
    % info and plots are enabled based on settings. Dynamics can be arbitrarily defined assigning it as
    % function handle. By default it uses the function "computeRefDynFcn", which expects data in the
    % strDynParams struct format (for interoperability with EstimationGears library functions).
    % Ephemerides are evaluated using Chebyshev polynomials data stored in strDynParams or using SPICE
    % kernels (TODO).
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 12-03-2025        Pietro Califano     First experimental version (tested)
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % Method1: Description
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % Property1: Description, dtype, nominal size
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = protected, GetAccess = public)
        
        % Orbit dynamics and attitude data
        strDynParams % TODO, transform to object

        % Configuration
        enumEphemerisMode
        enumGenerationMode
        enumWorldFrameName
        bDefaultConstructed         = true;
        bProvideAccelerationData    = false;

        % Attitude poiting generator
        objAttitudeGenerator = CAttitudePointingGenerator(); % TODO Currently not used
        
        % Sensors object
        objCamera = CCameraIntrinsics() % TODO currently a placeholder

        % TO ADD
    end


    methods (Access = public)
        % CONSTRUCTOR
        function self = CScenarioGenerator(dPosVelState0, ...
                                            dRelativeTimegrid, ...
                                            strDynParams, ...
                                            kwargs, ...
                                            settings)
            arguments
                dPosVelState0     (6,:) double {ismatrix, isnumeric} = zeros(6,1)
                dRelativeTimegrid (1,:) double {isvector, isnumeric} = 0.0
                strDynParams      (1,1) struct {isstruct} = struct();
            end
            arguments
                kwargs.enumWorldFrameName (1,:) {mustBeA(kwargs.enumWorldFrameName, "SEnumFrameName")} = EnumFrameName.J2000
                kwargs.dEphemerisTimegrid (1,:) double {isvector, isnumeric} = 0.0
                kwargs.objOrbitDynamicFcnHandle = [] % Assign if not empty
            end
            arguments
                % To select between Chbv polynomials and SPICE
                settings.enumGenerationMode         (1,:) string {mustBeMember(settings.enumGenerationMode, ["OrbitDyn", "OrbitPointing", ""])} = "OrbitPointing"
                settings.enumEphemerisMode          (1,:) string {mustBeMember(settings.enumEphemerisMode, ["SPICE", "interpolants"])} = "interpolants"
                settings.bEnablePlots               (1,1) logical {islogical, isscalar} = false % TODO
                settings.bProvideAccelerationData   (1,1) logical {islogical, isscalar} = false 
            end

            if nargin > 1
                self.bDefaultConstructed = false;
            end

            % Store data members
            self.dPosVelState0_W    = dPosVelState0;
            self.dRelativeTimegrid  = dRelativeTimegrid;
            self.dEphemerisTimegrid = kwargs.dEphemerisTimegrid;
            self.strDynParams       = strDynParams;

            self.enumWorldFrameName = kwargs.enumWorldFrameName;
            self.enumEphemerisMode  = settings.enumEphemerisMode;
            self.enumGenerationMode = settings.enumGenerationMode;
            self.bProvideAccelerationData = settings.bProvideAccelerationData;

            if length(self.dEphemerisTimegrid) == 1
                self.dEphemerisTimegrid = self.dRelativeTimegrid;
            end

            if not(isempty(kwargs.objOrbitDynamicFcnHandle))
                self.objOrbitDynamicFcnHandle = kwargs.objOrbitDynamicFcnHandle;
            else
                self.objOrbitDynamicFcnHandle = @(dTimestamp, dxState) computeRefDynFcn(dTimestamp,...
                                                                                        dxState,...
                                                                                        self.strDynParams);
            end

        end

        % GETTERS

        % SETTERS

        % METHODS
    end

    methods (Access = public)
        
        % MAIN ENTRY POINT FUNCTION
        function [objReferenceMissionData] = generateData(self)
        
            % TODO add code to write ephemerides to strDynParams if scenario generator is set to generate
            % attitudes as well!
            assert(not(isempty(self.strDynParams)), 'ERROR: Dynamics parameters struct cannot be empty. Ensure to have it configure properly for SimulationGears library.')

            % Define variables
            strAccelInfoData = [];
            ui32NumOfTimestamps     = length(self.dEphemerisTimegrid);
            dSunPosition_W          = zeros(3, ui32NumOfTimestamps);
            dStateSC_W              = zeros(6, ui32NumOfTimestamps);
            dDCM_TBfromW            = zeros(3,3, ui32NumOfTimestamps);
            dMainTargetPosition_W   = zeros(3, ui32NumOfTimestamps);

            %% Generate ephemerides data
            if not(isfield(self.strDynParams, 'strBody3rdData'))
                ui8NumOf3rdBodies = self.strDynParams.ui8NumOf3rdBodies;
            else
                ui8NumOf3rdBodies = uint8(length(self.strDynParams.strBody3rdData)); 
            end

            if ui8NumOf3rdBodies > 1
                d3rdBodyEphemerides = zeros(3*(ui8NumOf3rdBodies-1), ui32NumOfTimestamps);
            end

            if strcmpi(self.enumEphemerisMode, "interpolants")
                % Chebyshev interpolants for ephemerides evaluation TODO replace with interpolant object!
                for idT = 1:ui32NumOfTimestamps
                    dEvalPoint = self.dRelativeTimegrid(idT);


                    % Evaluate Sun interpolant (assumed first 3rd body in struct)
                    dSunPosition_W(:,idT) = evalChbvPolyWithCoeffs(self.strDynParams.strBody3rdData(1).strOrbitData.ui32PolyDeg, ...
                                                                            3, dEvalPoint,...
                                                                            self.strDynParams.strBody3rdData(1).strOrbitData.dChbvPolycoeffs, ...
                                                                            self.strDynParams.strBody3rdData(1).strOrbitData.dTimeLowBound, ...
                                                                            self.strDynParams.strBody3rdData(1).strOrbitData.dTimeUpBound);
    

                    if ui8NumOf3rdBodies > 1
                        % Evaluate position ephemerides of other bodies if required
                        dPtrAlloc = 1;
                        for idB = 1:ui8NumOf3rdBodies-1

                            d3rdBodyEphemerides(dPtrAlloc:dPtrAlloc+2, idT) = evalChbvPolyWithCoeffs(self.strDynParams.strBody3rdData(idB).strOrbitData.ui32PolyDeg, ...
                                                                                            3, dEvalPoint,...
                                                                                            self.strDynParams.strBody3rdData(idB).strOrbitData.dChbvPolycoeffs, ...
                                                                                            self.strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound, ...
                                                                                            self.strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound);
                            dPtrAlloc = dPtrAlloc + 3;
                        end
                    end

                    % Evaluate target ephemerides
                    dTmpQuat = evalAttQuatChbvPolyWithCoeffs(self.strDynParams.strMainData.strAttData.ui32PolyDeg, 4, dEvalPoint,...
                                                            self.strDynParams.strMainData.strAttData.dChbvPolycoeffs, ...
                                                            self.strDynParams.strMainData.strAttData.dsignSwitchIntervals, ...
                                                            self.strDynParams.strMainData.strAttData.dTimeLowBound, ...
                                                            self.strDynParams.strMainData.strAttData.dTimeUpBound);

                    dDCM_TBfromW(1:3, 1:3, idT) = Quat2DCM(dTmpQuat, true);

                    % Check if strDynParams.strMainData contains orbit data for position
                    % TODO
                    % dMainTargetPosition_W(1:3, idT)
                    % dMainTargetPosition_W(1:3, idT) = evalChbvPolyWithCoeffs(self.strDynParams.strBody3rdData(idB).strOrbitData.ui32PolyDeg, ...
                    %                                                                 3, dEvalPoint,...
                    %                                                                 self.strDynParams.strBody3rdData(idB).strOrbitData.dChbvPolycoeffs, ...
                    %                                                                 self.strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound, ...
                    %                                                                 self.strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound);
                end

            elseif strcmpi(self.enumEphemerisMode, "SPICE")
                error('Not implemented yet')
                % self.dEphemerisTimegrid
            end
            
            %% Determine generator function 
            if strcmpi(self.enumGenerationMode, "OrbitDyn") || strcmpi(self.enumGenerationMode, "OrbitPointing")

                % Propagate orbit trajectory
                [dTmpState] = self.propagateOrbitTrajectory();
                dStateSC_W(:,:) = dTmpState';

                if strcmpi(self.enumGenerationMode, "OrbitPointing")
                    % Construct attitude pointing for camera (assuming coincident frame with SC)
                    self.objAttitudeGenerator = CAttitudePointingGenerator( dStateSC_W(1:3,:), ...
                                                                        dMainTargetPosition_W, ...
                                                                        dSunPosition_W);

                    [self.objAttitudeGenerator, dCameraAttDCM_NavframeFromOF] = self.objAttitudeGenerator.pointToTarget_SunDirConstraint();
                    dDCM_SCfromW = pagetranspose(dCameraAttDCM_NavframeFromOF);
                end

                if self.bProvideAccelerationData
                    % Recompute RHS at each point of the trajectory
                    for idT = 1:length(self.dRelativeTimegrid)

                        [~, strTmpAccelInfoData] = self.objOrbitDynamicFcnHandle(self.dRelativeTimegrid(idT), ...
                                                                                dStateSC_W(:, idT));
                        if isempty(strAccelInfoData)
                            % Allocate first based on strAccelInfoData data
                            cellAccelFields = fieldnames(strTmpAccelInfoData);

                            for idF = 1:length(cellAccelFields)
                                strAccelInfoData.(cellAccelFields{idF}) = zeros(3, length(self.dRelativeTimegrid));
                            end
                        end

                        for idF = 1:length(cellAccelFields)
                            % Allocate ith acceleration value
                            strAccelInfoData.(cellAccelFields{idF})(:,idT) = strTmpAccelInfoData.(cellAccelFields{idF});
                        end


                    end

                end

                % Package dataset object
                [objReferenceMissionData] = CScenarioGenerator.packageDataset(self.objCamera, ...
                                                            self.enumWorldFrameName, ...
                                                            self.dEphemerisTimegrid, ...
                                                            dStateSC_W, ...
                                                            dDCM_SCfromW, ...
                                                            dDCM_TBfromW, ...
                                                            dMainTargetPosition_W, ...
                                                            dSunPosition_W, ...
                                                            "strAccelInfoData", strAccelInfoData);
                return
            % elseif strcmpi(self.enumGenerationMode, "OrbitDyn") 
            % TODO
            else
                error('Unsupported or invalid generation mode.')
            end

        end
    end

    methods (Access = protected)

        function [] = buildOdeObject_(self)
            % TODO
        end

        function assertInitialization(self)
            % TODO
        end

    end

    methods (Static, Access = public)
        % STATIC METHODS
        function [charTargetName, charTargetFixedFrame, strDynParams] = LoadDefaultScenarioData(enumScenarioName, ...
                                                                                                strDynParams, ...
                                                                                                kwargs, ...
                                                                                                settings)
            arguments
                enumScenarioName EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])}
                strDynParams (1,1) = struct()
            end
            arguments
                % TODO load from file if specified
                kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
            end
            arguments
                settings.bAddNonSphericalGravityCoeffs (1,1) logical {islogical, isscalar} = false;
            end
            %% INPUT
            % arguments
            %     enumScenarioName EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])}
            %     strDynParams (1,1) = struct()
            % end
            % arguments
            %     % TODO load from file if specified
            %     kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
            % end
            % arguments
            %     settings.bAddNonSphericalGravityCoeffs (1,1) logical {islogical, isscalar} = false;
            % end
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % charTargetName
            % charTargetFixedFrame
            % strDynParams
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 14-03-2025    Pietro Califano     First version implemented from legacy codes
            % 15-06-2025    Pietro Califano     Fix incorrect measurement unit for Apophis radius
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Future upgrades
            % [-]
            % -------------------------------------------------------------------------------------------------------------

            % Define empty fields if not provided
            if isfield(strDynParams, 'strMainData')
                if not(isfield(strDynParams.strMainData, 'dSHcoeff'))
                    strDynParams.strMainData.dSHcoeff = [];
                end

                if not(isfield(strDynParams.strMainData, 'ui16MaxSHdegree'))
                    strDynParams.strMainData.ui16MaxSHdegree = [];
                end
            else
                strDynParams.strMainData.dSHcoeff = [];
                strDynParams.strMainData.ui16MaxSHdegree = [];
            end
            

            switch enumScenarioName
                case EnumScenarioName.Itokawa
                    % REFERENCE source: (Scheeres, 2006)
                    charTargetName = 'ITOKAWA';
                    charTargetFixedFrame = "ITOKAWA_FIXED";

                    % try
                    %     dTargetReferenceRadius  = mean(cspice_bodvrd(num2str(ui32ID),'RADII',3)); % [m] ACHTUNG: Value used for Gravity SH expansion!
                    %     dTargetGravityParameter = cspice_bodvrd(num2str(ui32ID),'GM',1)*1e+09;            % [m^3/(s^2)]
                    % catch
                        dTargetGravityParameter = 2.36; % m^3/s^2
                        dTargetReferenceRadius  = 1E+03 * 0.161915; % [m] ACHTUNG: Value used for Gravity SH expansion!
                    % end

                case EnumScenarioName.Apophis
                    % REFERENCE source: TODO
                    charTargetName = 'APOPHIS';
                    charTargetFixedFrame = "APOPHIS_FIXED";

                    try
                        ui32ID = 20099942;
                        dTargetReferenceRadius  = 1E+03 * mean(cspice_bodvrd(num2str(ui32ID),'RADII',3)); % [m] ACHTUNG: Value used for Gravity SH expansion!
                        dTargetGravityParameter = 1E+09 * cspice_bodvrd(num2str(ui32ID),'GM',1) ;         % [m^3/(s^2)]
                    catch
                        warning('Fetch of Apophis data from kernels failed. Fallback to hardcoded data...')
                        dTargetReferenceRadius  = 1E+03 * 0.175930344; % [m] ACHTUNG: Value used for Gravity SH expansion!
                        dTargetGravityParameter = 3.003435675;         % [m^3/(s^2)]
                    end

                case EnumScenarioName.Bennu_OREx

                    charTargetName = 'BENNU';
                    charTargetFixedFrame = 'IAU_BENNU'; % Check corresponding tf file
                    dTargetGravityParameter = 4.892;
                    dTargetReferenceRadius  = 245; % [m] ACHTUNG: Value used for Gravity SH expansion!

                case EnumScenarioName.Didymos_Hera
                    error('To implement')

                otherwise
                    error('Invalid scenario name. See EnumScenarioName enum class for supported ones.')
            end


            % Store basic data
            strDynParams.strMainData.dGM        = dTargetGravityParameter;
            strDynParams.strMainData.dRefRadius = dTargetReferenceRadius;


            % Handle request of Spherical Harmonics coefficients
            if settings.bAddNonSphericalGravityCoeffs == true

                % Get data depending on scenario
                CScenarioGenerator.LoadSpherHarmCoefficients(enumScenarioName, kwargs.charSpherHarmCoeffInputFileName);

                strDynParams.strMainData.ui16MaxSHdegree = ui16MaxSHdegree;
                dScaleFactors = ExtSHE_normFactors(strDynParams.strMainData.ui16MaxSHdegree);

                % Compute unnormalized coefficients
                strDynParams.strMainData.dSHcoeff = dClmSlm_normalized./dScaleFactors;

            end

        end
    

        function [dClmSlm_normalized, ui16MaxSHdegree] = LoadSpherHarmCoefficients(enumScenarioName, charSpherHarmCoeffInputFileName)
            arguments
                enumScenarioName EnumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])}
                charSpherHarmCoeffInputFileName (1,:) string {mustBeA(charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
            end
            
            switch enumScenarioName
                case EnumScenarioName.Itokawa
                    % REFERENCE source: (Scheeres, 2006)
                    % Itokawa Spherical harmonics expansion coefficients (Scheeres, 2006)

                    if strcmpi(charSpherHarmCoeffInputFileName, "")
                        % Use hardcoded values
                        dClmSlm_normalized = [0.0,       0.0;
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

                        ui16MaxSHdegree = 4;
                    else

                        % TODO: modify to use: [o_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffDataPath, ui16lMax, bENABLE_UNSCALING)
                        % Normalized coefficients from l=1, m=1 as required by ExtSHE_AccTB function

                        % Load Spherical Harmonics coeffs from file
                        error('Not implemented yet >.<')
                    end

                case EnumScenarioName.Apophis
                    % REFERENCE source: TODO

                    % TODO: modify to use: [o_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffDataPath, ui16lMax, bENABLE_UNSCALING)
                    % Normalized coefficients from l=1, m=1 as required by ExtSHE_AccTB function

                    if strcmpi(charSpherHarmCoeffInputFileName, "")
                        % Use hardcoded values
                        dClmSlm_normalized = [];
                        ui16MaxSHdegree = 0;
                        warning('Unavailable hardcoded data')
                    else

                        % TODO: modify to use: [o_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffDataPath, ui16lMax, bENABLE_UNSCALING)
                        % Normalized coefficients from l=1, m=1 as required by ExtSHE_AccTB function

                        % Load Spherical Harmonics coeffs from file
                        error('Not implemented yet >.<')
                    end


                case EnumScenarioName.Bennu_OREx

                    if strcmpi(charSpherHarmCoeffInputFileName, "")
                        % Use hardcoded values
                        dClmSlm_normalized = [];
                        ui16MaxSHdegree = 0;
                    else

                        % TODO: modify to use: [o_dCSlmCoeffCols, o_dlmPairs] = loadSHEcoeffModel(modelCoeffDataPath, ui16lMax, bENABLE_UNSCALING)
                        % Normalized coefficients from l=1, m=1 as required by ExtSHE_AccTB function

                        % Load Spherical Harmonics coeffs from file
                        error('Not implemented yet >.<')
                    end

                    warning('Unavailable hardcoded data')

                case EnumScenarioName.Didymos_Hera
                    error('To implement')

                otherwise
                    error('Invalid scenario name. See EnumScenarioName enum class for supported ones.')
            end

        end


        function [objReferenceMissionData] = packageDataset(objCamera, ...        
                                                            enumWorldFrame, ...   
                                                            dTimestamps, ...      
                                                            dStateSC_W, ...  
                                                            dDCM_SCfromW, ...
                                                            dDCM_TBfromW, ...     
                                                            dTargetPosition_W, ...
                                                            dSunPosition_W, ...   
                                                            dEarthPosition_W, ... 
                                                            dRelativeTimestamps, ...
                                                            optional)
            arguments
                % Reference definition
                objCamera                    (1,1)      {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
                enumWorldFrame               (1,1)      {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached
                dTimestamps                  (1,:)      double {isnumeric, isvector} = [];
                dStateSC_W                   (6, :)     double {isnumeric, ismatrix} = [];
                dDCM_SCfromW                 (3, 3, :)  double {isnumeric, ismatrix} = [];
                dDCM_TBfromW                 (3, 3, :)  {isnumeric, ismatrix} = [];
                dTargetPosition_W            (3,:)      {isnumeric, ismatrix} = [];
                dSunPosition_W               (3,:)      {isnumeric, ismatrix} = [];
                dEarthPosition_W             (3,:)      {isnumeric, ismatrix} = [];
                dRelativeTimestamps          (1,:)      {isnumeric, isvector} = [];
            end
            arguments
                optional.strAccelInfoData = []
                optional.dPrimaryPointingWhileMan_W   (3, :, :)  double {isnumeric, ismatrix} = [] % TBC, primary pointing axis during manoeuvres
                optional.dSecondPointingWhileMan_W    (3, :, :)  double {isnumeric, ismatrix} = [] % TBC, secondary axis during manoeuvres
                optional.dManoeuvresTimegrids         (3, :)     double {isnumeric, ismatrix} = [];
                optional.dManoeuvresStartTimestamps   (1, :)     double {isnumeric, isvector} = [];
                optional.dManoeuvresDeltaV_SC         (3, :)     double {isnumeric, ismatrix} = [];
            end

            % Determine relative timegrid if not provided
            if isempty(dRelativeTimestamps)
                dRelativeTimestamps = dTimestamps - dTimestamps(1);
            end
            
            % Build dataset object
            objReferenceMissionData = SReferenceImagesDataset(objCamera, ...
                                                              enumWorldFrame, ...
                                                              dTimestamps, ...
                                                              dStateSC_W, ...
                                                              dDCM_TBfromW, ...
                                                              dTargetPosition_W, ...
                                                              dSunPosition_W, ...
                                                              dEarthPosition_W, ...
                                                              "dRelativeTimestamps", dRelativeTimestamps, ...
                                                              "dDCM_SCfromW", dDCM_SCfromW);

            % Add acceleration info if provided
            if not(isempty(optional.strAccelInfoData))
                objReferenceMissionData.strAccelInfoData = optional.strAccelInfoData;
            end
        end



        
        % function [] = GenerateOrbitTrajectoryStatic()
        %     % TODO: integrates equations of motions of attitude kinematics
        % end
        % 
        % function [] = GenerateFreeAttitudeProfileStatic()
        %     % TODO: integrates equations of motions of attitude kinematics
        % end
        % 
        % function [] = GenerateNavPoseTrajectoryStatic()
        %     % TODO: integrates equations of motions of orbit dynamics + attitude kinematics
        % end
        % 
        % function [] = GeneratePoseDynamicsTrajectoryStatic()
        %     % TODO: integrates equations of motions of orbit and attitude dynamics + kinematics
        % end

    end


end

