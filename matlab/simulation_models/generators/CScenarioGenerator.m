classdef CScenarioGenerator < handle
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

        dPosVelState0_W
        dAttDCM0_W
        
        dRelativeTimegrid
        dEphemerisTimegrid

        % Orbit dynamics and attitude data
        strDynParams % TODO, transform to object

        % Configuration
        enumEphemerisMode
        enumGenerationMode
        enumWorldFrameName
        bDefaultConstructed         = true;
        bEnableScenarioPlots        = false;
        bProvideAccelerationData    = false;

        % Attitude poiting generator
        objAttitudeGenerator = CAttitudePointingGenerator(); % TODO Currently not used
        
        % Sensors object
        objCamera = CCameraIntrinsics() % TODO currently a placeholder

        % Handles
        objOdeSolution            {mustBeScalarOrEmpty} = []
        objOrbitDynamicFcnHandle  {mustBeScalarOrEmpty} = []
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
                kwargs.enumWorldFrameName (1,:) {mustBeA(kwargs.enumWorldFrameName, "SEnumFrameName")} = EnumFrameName.IN
                kwargs.dEphemerisTimegrid (1,:) double {isvector, isnumeric} = 0.0
                kwargs.objOrbitDynamicFcnHandle = [] % Assign if not empty
            end
            arguments
                % To select between Chbv polynomials and SPICE
                settings.enumGenerationMode         (1,:) string {mustBeMember(settings.enumGenerationMode, ["OrbitDyn", "OrbitPointing", ""])} = "OrbitPointing"
                settings.enumEphemerisMode          (1,:) string {mustBeMember(settings.enumEphemerisMode, ["SPICE", "interpolants"])} = "interpolants"
                settings.bEnableScenarioPlots       (1,1) logical {islogical, isscalar} = false % TODO
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

        function [dxStateTrajectory, dTimegrid] = propagateOrbitTrajectory(self, varargparams, settings)
            arguments
                self
            end
            arguments (Repeating)
                varargparams
            end
            arguments
                settings.dTimestep (1,1) double {isscalar, isnumeric} = 0.0 % Default
                settings.objOdeOpts {isstruct} = odeset('RelTol', 1E-12, 'AbsTol', 1E-12) % Default
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78"])}
            end 

            % TODO: integrates equations of motions of attitude kinematics

            % [dxStateTrajectory, dTimegrid] = propagateState(objDynamicFcnHandle, ...
            %                                                 self.dEphemerisTimegrid, ...
            %                                                 self.dPosVelState0, ...
            %                                                 varargparams, ...
            %                                                 "dTimestep", settings.dTimestep, ...
            %                                                 "objOdeOpts", settings.objOdeOpts, ...
            %                                                 "enumOdeFunctioName", settings.enumOdeFunctioName);
            
            % Unwrap settings to cell
            cellSettings = CScenarioGenerator.unwrapSettings(settings);
            
            % Call ODE-based propagator 
            [dxStateTrajectory, dTimegrid] = CScenarioGenerator.propagateState(self.objOrbitDynamicFcnHandle, ...
                                                                    self.dEphemerisTimegrid, ...
                                                                    self.dPosVelState0_W, ...
                                                                    varargparams, ...
                                                                    cellSettings{:});
        end


        function [dxStateTrajectory, dTimegrid] = generateOrbitTrajectory(self, varargparams, settings)
            arguments
                self
            end
            arguments (Repeating)
                varargparams
            end
            arguments
                settings.dTimestep (1,1) double {isscalar, isnumeric} = 0.0 % Default
                settings.objOdeOpts {isstruct} = odeset('RelTol', 1E-12, 'AbsTol', 1E-12) % Default
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78"])}
            end 

            % TODO: integrates equations of motions of attitude kinematics

            % [dxStateTrajectory, dTimegrid] = propagateState(objDynamicFcnHandle, ...
            %                                                 self.dEphemerisTimegrid, ...
            %                                                 self.dPosVelState0, ...
            %                                                 varargparams, ...
            %                                                 "dTimestep", settings.dTimestep, ...
            %                                                 "objOdeOpts", settings.objOdeOpts, ...
            %                                                 "enumOdeFunctioName", settings.enumOdeFunctioName);

            cellSettings = CScenarioGenerator.unwrapSettings(settings);

            [dxStateTrajectory, dTimegrid] = propagateState(objDynamicFcnHandle, ...
                                                            self.dEphemerisTimegrid, ...
                                                            self.dPosVelState0_W, ...
                                                            varargparams, ...
                                                            cellSettings{:});
        end


        function [] = propagateAttitudePoitingProfile(self)
            % Generate attitude pointing profile from CAttitudeGenerator class

        end

        function [] = propagateFreeAttitudeProfile(self)
            % TODO: integrates equations of motions of attitude kinematics
        end

        function [] = propagateNavPoseTrajectory(self)
            % TODO: integrates equations of motions of orbit dynamics + attitude kinematics
        end

        function [] = propagatePoseDynamicsTrajectory(self)
            % TODO: integrates equations of motions of orbit and attitude dynamics + kinematics
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

        function [dxStateTrajectory, dTimegrid] = propagateState(objDynamicFcnHandle, ...
                                                                dTimegrid, ...
                                                                dxState0, ...
                                                                varargparams, ...
                                                                settings)
            arguments
                objDynamicFcnHandle {mustBeA(objDynamicFcnHandle, 'function_handle')}
                dTimegrid           (1,:) double {isvector, isnumeric}
                dxState0            (:,1) double {isvector, isnumeric}
            end
            arguments (Repeating)
                varargparams
            end
            arguments
                settings.dTimestep (1,1) double {isscalar, isnumeric} = 0.0 % Default
                settings.objOdeOpts {isstruct} = odeset('RelTol', 1E-12, 'AbsTol', 1E-12) % Default
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78"])} = "ode113"
            end 


            % objDynamicFcnHandle = @(dTime, dxState) objDynamicFcnHandle(dTime, dxState);
            % cellOdeInput = {objDynamicFcnHandle, dTimegrid, dxState0, settings.objOdeOpts};

            assert(length(dTimegrid) >= 2, 'ERROR: invalid timegrid. It must contain at least two time instants.')

            % Determine timegrid if required
            if length(dTimegrid) == 2 && not(settings.dTimestep == 0)
                dTimegrid = dTimegrid(1):settings.dTimestep:dTimegrid(2);
            end

            switch settings.enumOdeFunctioName
                case "ode113"

                    [dTimegrid, dxStateTrajectory] = ode113(objDynamicFcnHandle, ...
                                                            dTimegrid, ...
                                                            dxState0, ...
                                                            settings.objOdeOpts);

                case "ode45"

                    [dTimegrid, dxStateTrajectory] = ode45(objDynamicFcnHandle, ...
                                                                dTimegrid, ...
                                                                dxState0, ...
                                                                settings.objOdeOpts);
                case "ode78"

                    [dTimegrid, dxStateTrajectory] = ode78(objDynamicFcnHandle, ...
                                                                dTimegrid, ...
                                                                dxState0, ...
                                                                settings.objOdeOpts);

                otherwise
                    error('Unsupported ode function')
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


        function cellSettings = unwrapSettings(settings)
            arguments
                settings (1,1) {isstruct}
            end

            cellFieldNames = fieldnames(settings);
            cellSettings = cell(1, length(cellFieldNames));

            ui32AllocCounter = 1;

            for idF = 1:length(cellSettings)

                % Store key
                cellSettings{ui32AllocCounter} = cellFieldNames{idF};
                ui32AllocCounter = ui32AllocCounter + 1;

                % Store value
                cellSettings{ui32AllocCounter} = settings.(cellFieldNames{idF});
                ui32AllocCounter = ui32AllocCounter + 1;

            end
        end

    end


end

