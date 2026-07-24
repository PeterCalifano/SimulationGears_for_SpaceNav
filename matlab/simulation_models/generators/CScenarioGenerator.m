classdef CScenarioGenerator < CGeneralPropagator
    %% DESCRIPTION
    % Generator class constructing dataset object to define 3D scene over time, with spacecraft trajectory and 
    % attitude, Sun position, target position and attitude, ephemerides of additional bodies. Acceleration
    % info and plots are enabled based on settings. Dynamics can be arbitrarily defined assigning it as
    % function handle. By default it uses the function "ComputeRefDynFcn", which expects data in the
    % strDynParams struct format (for interoperability with EstimationGears library functions).
    % Ephemerides are evaluated using Chebyshev polynomials data stored in strDynParams or using SPICE
    % kernels (TODO).
    %
    % Static construction utilities resolve target constants and embedded
    % spherical-harmonics gravity through CScenarioRegistry. Explicit
    % coefficient files remain caller-selected overrides.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 12-03-2025        Pietro Califano     First experimental version (tested)
    % 24-07-2026        Pietro Califano, Codex     Document registry-owned embedded gravity defaults.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % CScenarioGenerator: Construct a stateful reference-scenario generator.
    % generateData: Propagate and package the configured reference scenario.
    % LoadDefaultScenarioData: Resolve registry-backed target and gravity data.
    % LoadSpherHarmCoefficients: Load explicit or registry-selected coefficients.
    % BuildReferenceScenarioDataset: Build a complete reference dataset from supplied states.
    % packageDataset: Package state and environment histories into the dataset object.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % strDynParams: Dynamics parameters used by the reference propagator.
    % enumEphemerisMode: Selected ephemeris evaluation mode.
    % enumGenerationMode: Selected orbit and pointing generation mode.
    % enumWorldFrameName: World frame attached to generated data.
    % bProvideAccelerationData: Include acceleration histories when enabled.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CGeneralPropagator
    % CScenarioRegistry
    % SSphericalHarmonicsGravityData
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
                dPosVelState0     (6,:) double {mustBeNumeric} = zeros(6,1)
                dRelativeTimegrid (1,:) double {mustBeNumeric} = 0.0
                strDynParams      (1,1) struct  = struct();
            end
            arguments
                kwargs.enumWorldFrameName (1,:) {mustBeA(kwargs.enumWorldFrameName, ["EnumFrameName", "string", "char"])} = EnumFrameName.J2000
                kwargs.dEphemerisTimegrid (1,:) double {mustBeNumeric} = 0.0
                kwargs.objOrbitDynamicFcnHandle = [] % Assign if not empty
            end
            arguments
                % To select between Chbv polynomials and SPICE
                settings.enumGenerationMode         (1,:) string {mustBeMember(settings.enumGenerationMode, ["OrbitDyn", "OrbitPointing", ""])} = "OrbitPointing"
                settings.enumEphemerisMode          (1,:) string {mustBeMember(settings.enumEphemerisMode, ["SPICE", "interpolants"])} = "interpolants"
                settings.bEnablePlots               (1,1) logical = false % TODO
                settings.bProvideAccelerationData   (1,1) logical = false 
            end
            %% SIGNATURE
            % self = CScenarioGenerator(dPosVelState0, dRelativeTimegrid, strDynParams, kwargs, settings)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Construct a scenario generator from initial spacecraft state, time grids, dynamics parameters,
            % and generation settings. If no custom dynamics function is provided, the generator uses
            % ComputeRefDynFcn with strDynParams.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % dPosVelState0: Initial spacecraft position/velocity state in the selected world frame.
            % dRelativeTimegrid: Relative propagation timestamps.
            % strDynParams: Dynamics-parameter structure consumed by ComputeRefDynFcn-compatible models.
            % kwargs.enumWorldFrameName: World-frame enum/name attached to generated state data.
            % kwargs.dEphemerisTimegrid: Time grid used for ephemeris evaluation; scalar defaults to dRelativeTimegrid.
            % kwargs.objOrbitDynamicFcnHandle: Optional custom orbit-dynamics function handle.
            % settings.enumGenerationMode: Generation mode, currently OrbitDyn or OrbitPointing.
            % settings.enumEphemerisMode: Ephemeris source mode, currently interpolants or SPICE placeholder.
            % settings.bEnablePlots: Plot-enable flag reserved for future use.
            % settings.bProvideAccelerationData: Request acceleration component traces in the output dataset.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self: Initialized CScenarioGenerator object.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ComputeRefDynFcn
            % -------------------------------------------------------------------------------------------------------------

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

            if isscalar(self.dEphemerisTimegrid)
                self.dEphemerisTimegrid = self.dRelativeTimegrid;
            end

            if not(isempty(kwargs.objOrbitDynamicFcnHandle))
                self.objOrbitDynamicFcnHandle = kwargs.objOrbitDynamicFcnHandle;
            else
                self.objOrbitDynamicFcnHandle = @(dTimestamp, dxState) ComputeRefDynFcn(dTimestamp,...
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
            %% SIGNATURE
            % objReferenceMissionData = generateData(self)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Generate reference mission data by evaluating target/Sun ephemerides, propagating the spacecraft
            % trajectory, optionally generating pointing, and packaging the result as SReferenceImagesDataset.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self: Configured CScenarioGenerator object with non-empty strDynParams.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objReferenceMissionData: Reference image dataset containing trajectory, pointing, target, Sun,
            % Earth, and optional acceleration data.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CAttitudePointingGenerator, CScenarioGenerator.packageDataset, SReferenceImagesDataset
            % -------------------------------------------------------------------------------------------------------------
        
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
                strDynParams (1,1) struct = struct()
            end
            arguments
                kwargs.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(kwargs.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
                kwargs.bUseKilometersScale             (1,1) logical = false;
                kwargs.ui16MaxSHdegree                 (1,1) uint16 = uint16(4);
            end
            arguments
                settings.bAddNonSphericalGravityCoeffs (1,1) logical = false;
            end
            %% SIGNATURE
            % [charTargetName, charTargetFixedFrame, strDynParams] = LoadDefaultScenarioData(enumScenarioName, ...
            %     strDynParams, kwargs, settings)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Resolve registry-backed default target dynamics data for a scenario and optionally attach
            % spherical-harmonics gravity coefficients. A caller-provided
            % schema-backed file takes precedence and is validated against the
            % requested scenario. Otherwise CScenarioRegistry returns an exact
            % truncation of the scenario's embedded coefficient family.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % enumScenarioName                       Registered scenario enum or name.
            % strDynParams                           Existing dynamics payload to populate or extend.
            % kwargs.charSpherHarmCoeffInputFileName Optional schema-backed coefficient-file override.
            % kwargs.bUseKilometersScale             Return dimensional gravity data in km-based units.
            % kwargs.ui16MaxSHdegree                 Requested maximum SH degree; zero disables SH loading.
            % settings.bAddNonSphericalGravityCoeffs Attach SH coefficients when true.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % charTargetName                         Registry-selected SPICE target name.
            % charTargetFixedFrame                   Registry-selected target-fixed frame.
            % strDynParams                           Dynamics payload with target constants and optional SH data.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 14-03-2025    Pietro Califano     First version implemented from legacy codes
            % 15-06-2025    Pietro Califano     Fix incorrect measurement unit for Apophis radius
            % 22-07-2025    Pietro Califano     Add new scenarios, updates to support future-nav simulations
            % 01-07-2026    Pietro Califano     Add support for user-defined Spherical Harmonics coefficients
            % 24-07-2026    Pietro Califano, Codex     Document embedded registry coefficient selection.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CScenarioRegistry
            % SSphericalHarmonicsGravityData
            % -------------------------------------------------------------------------------------------------------------

            if kwargs.bUseKilometersScale
                charLengthUnits = "km";
            else
                charLengthUnits = "m";
            end

            % Define empty fields if not provided
            if isfield(strDynParams, 'strMainData')
                if not(isfield(strDynParams.strMainData, 'dSHcoeff'))
                    strDynParams.strMainData.dSHcoeff = [];
                end

                if not(isfield(strDynParams.strMainData, 'ui16MaxSHdegree'))
                    strDynParams.strMainData.ui16MaxSHdegree = [];
                end
            else
                % Define empty fields
                strDynParams.strMainData.dSHcoeff = [];
                strDynParams.strMainData.ui16MaxSHdegree = [];
            end

            % Get specification of scenario from registry for tagged scenario name
            strScenarioSpec = CScenarioRegistry.GetScenarioSpec(enumScenarioName, charLengthUnits=charLengthUnits);
            if ~strScenarioSpec.bHasGravityDefaults
                error('CScenarioGenerator:MissingScenarioDefaults', ...
                    'Scenario %s does not define registry-backed default dynamics data.', string(enumScenarioName));
            end

            charTargetName = char(strScenarioSpec.charSPICETargetName);
            charTargetFixedFrame = strScenarioSpec.charTargetFixedFrame;
            dTargetReferenceRadius = strScenarioSpec.dReferenceRadius;
            dTargetGravityParameter = strScenarioSpec.dGravParam;

            % Store basic data
            strDynParams.strMainData.dGM        = dTargetGravityParameter;
            strDynParams.strMainData.dRefRadius = dTargetReferenceRadius;

            % Handle request of Spherical Harmonics coefficients
            if settings.bAddNonSphericalGravityCoeffs == true
                
                % Check input values
                mustBeNonnegative(kwargs.ui16MaxSHdegree)

                ui32RequestedDegree = uint32(kwargs.ui16MaxSHdegree);

                if ui32RequestedDegree > uint32(0)
                    % Get data depending on scenario
                    if strlength(kwargs.charSpherHarmCoeffInputFileName) > 0

                        objSHdata = SSphericalHarmonicsGravityData.fromFile(kwargs.charSpherHarmCoeffInputFileName);
                        objSHdata.validate(ui32RequestedDegree, ...
                            "strScenarioSpec", strScenarioSpec, ...
                            "bRequireScenarioMatch", true);

                        strSHgravityData = objSHdata.toGravityDataStruct(ui32RequestedDegree);
                        strSHmeta = objSHdata.toMetadataStruct();
                        fprintf(['Loaded file-backed spherical harmonics for %s: degree %u/%u, ' ...
                            'units %s, normalization %s, source %s, %s\n'], ...
                            char(string(enumScenarioName)), ui32RequestedDegree, strSHmeta.ui32FileMaxDegree, ...
                            char(strSHmeta.charLengthUnits), char(strSHmeta.charNormalization), ...
                            char(strSHmeta.charSource), char(strSHmeta.charSourceUrl));

                        dTargetGravityParameter = strSHgravityData.dGravParam;
                        dTargetReferenceRadius = strSHgravityData.dBodyRadiusRef;
                        strDynParams.strMainData.dGM = dTargetGravityParameter;
                        strDynParams.strMainData.dRefRadius = dTargetReferenceRadius;
                    else
                        % Load registry-backed spherical harmonics data
                        [strSHgravityData, strSHmeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                            enumScenarioName, ui32RequestedDegree, charLengthUnits);

                        if ~strSHmeta.bHasHardcodedCoefficients || ...
                                ui32RequestedDegree > strSHmeta.ui32HardcodedMaxDegree
                            error('CScenarioGenerator:RegistrySHUnavailable', ...
                                ['Registry SH data for %s are unavailable at requested degree %u. ' ...
                                 'Available hardcoded max degree is %u.'], ...
                                string(enumScenarioName), ui32RequestedDegree, ...
                                strSHmeta.ui32HardcodedMaxDegree);
                        end
                        fprintf(['Loaded registry spherical harmonics for %s: degree %u/%u, ' ...
                            'units %s, normalization %s, source %s, %s\n'], ...
                            char(string(enumScenarioName)), ui32RequestedDegree, ...
                            strSHmeta.ui32HardcodedMaxDegree, ...
                            char(charLengthUnits), char(strSHmeta.charNormalization), ...
                            char(strSHmeta.charSource), char(strSHmeta.charSourceUrl));
                    end

                    strDynParams.strMainData.ui16MaxSHdegree = uint16(strSHgravityData.ui32MaxDegree);
                    strDynParams.strMainData.dSHcoeff = strSHgravityData.dCSlmCoeffCols;
                end

            end

        end
    

        function [dCSlmCoeffCols, ui16MaxSHdegree] = LoadSpherHarmCoefficients(enumScenarioName, charSpherHarmCoeffInputFileName)
            arguments
                enumScenarioName {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])}
                charSpherHarmCoeffInputFileName (1,:) string {mustBeA(charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
            end
            %% SIGNATURE
            % [dCSlmCoeffCols, ui16MaxSHdegree] = LoadSpherHarmCoefficients(enumScenarioName, charSpherHarmCoeffInputFileName)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Load unnormalized [Clm, Slm] spherical-harmonics coefficient columns for a scenario from a
            % schema-backed file when provided, otherwise from embedded
            % registry metadata. Explicit files are validated against the
            % requested scenario; registry lookup returns its complete
            % hardcoded coefficient family.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % enumScenarioName: Scenario enum/name used to validate registry or file-backed coefficients.
            % charSpherHarmCoeffInputFileName: Optional SSphericalHarmonicsGravityData MAT/JSON file path.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dCSlmCoeffCols: Unnormalized [Clm, Slm] coefficient columns.
            % ui16MaxSHdegree: Maximum spherical-harmonics degree represented by dCSlmCoeffCols.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 24-07-2026  Pietro Califano, Codex     Document embedded registry default loading.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CScenarioRegistry, SSphericalHarmonicsGravityData
            % -------------------------------------------------------------------------------------------------------------

            if strlength(charSpherHarmCoeffInputFileName) > 0
                % Load spherical harmonics coefficients from user-provided file
                objSHdata = SSphericalHarmonicsGravityData.fromFile(charSpherHarmCoeffInputFileName);
                strScenarioSpec = CScenarioRegistry.GetScenarioSpec(enumScenarioName, ...
                                                    charLengthUnits=string(objSHdata.charLengthUnits));

                objSHdata.validate(objSHdata.ui32MaxDegree, ...
                    "strScenarioSpec", strScenarioSpec, ...
                    "bRequireScenarioMatch", true);

                strSHgravityData = objSHdata.toGravityDataStruct(objSHdata.ui32MaxDegree);
                dCSlmCoeffCols = strSHgravityData.dCSlmCoeffCols;
                ui16MaxSHdegree = uint16(strSHgravityData.ui32MaxDegree);
                return
            end

            % Load the complete embedded family so callers can retain its
            % declared maximum degree with the coefficient rows.
            strScenarioSpec = CScenarioRegistry.GetScenarioSpec(enumScenarioName);
            strSHmeta = strScenarioSpec.strSphericalHarmonics;
            if ~strSHmeta.bHasHardcodedCoefficients
                dCSlmCoeffCols = zeros(0, 2);
                ui16MaxSHdegree = uint16(0);
                return
            end

            dCSlmCoeffCols = strSHmeta.dCSlmCoeffCols;
            ui16MaxSHdegree = uint16(strSHmeta.ui32HardcodedMaxDegree);
        end

        function [objReferenceMissionData, strEnvironmentData, strDynParams, strScenarioMetadata] = BuildReferenceScenarioDataset(enumScenarioName, ...
                                    dStateSC_W, ...
                                    dTimestamps, ...
                                    options)
            arguments
                enumScenarioName (1,:) {mustBeA(enumScenarioName, ["EnumScenarioName", "string", "char"])}
                dStateSC_W       (6,:) double {mustBeNumeric}
                dTimestamps      (1,:) double {mustBeNumeric}
                options.objCamera (1,1) {mustBeA(options.objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics()
                options.enumWorldFrame (1,:) {mustBeA(options.enumWorldFrame, ["EnumFrameName", "string", "char"])} = EnumFrameName.J2000
                options.dDCM_SCfromW (3,3,:) double {mustBeNumeric} = []
                options.dDCM_TBfromW (3,3,:) double {mustBeNumeric} = []
                options.dTargetPosition_W (3,:) double {mustBeNumeric} = []
                options.dSunPosition_W (3,:) double {mustBeNumeric} = []
                options.dEarthPosition_W (3,:) double {mustBeNumeric} = []
                options.dRelativeTimestamps (1,:) double {mustBeNumeric} = []
                options.bCompleteFromReferences (1,1) logical = false
                options.bUseKilometersScale (1,1) logical = false
                options.bAddNonSphericalGravityCoeffs (1,1) logical = false
                options.charSpherHarmCoeffInputFileName (1,:) string {mustBeA(options.charSpherHarmCoeffInputFileName, ["string", "char"])} = ""
                options.ui16MaxSHdegree (1,1) uint16 = uint16(4)
            end
            %% SIGNATURE
            % [objReferenceMissionData, strEnvironmentData, strDynParams, strScenarioMetadata] = ...
            %     BuildReferenceScenarioDataset(enumScenarioName, dStateSC_W, dTimestamps, options)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Build a reference scenario dataset directly from supplied spacecraft states and timestamps,
            % resolving default environment/dynamics metadata from the scenario registry.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % enumScenarioName: Scenario enum/name used for registry metadata and target defaults.
            % dStateSC_W: Spacecraft position/velocity state history, one column per timestamp.
            % dTimestamps: Absolute timestamps associated with dStateSC_W.
            % options.objCamera: Camera intrinsics object attached to the reference dataset.
            % options.enumWorldFrame: World frame associated with the state and vector data.
            % options.dDCM_SCfromW: Optional spacecraft attitude DCM history.
            % options.dDCM_TBfromW: Optional target-body attitude DCM history.
            % options.dTargetPosition_W: Optional target position history.
            % options.dSunPosition_W: Optional Sun position history.
            % options.dEarthPosition_W: Optional Earth position history.
            % options.dRelativeTimestamps: Optional relative timestamps; packageDataset derives them when empty.
            % options.bCompleteFromReferences: Fill missing reference arrays with identity/zero defaults.
            % options.bUseKilometersScale: Resolve registry distances and GM in kilometers-based units.
            % options.bAddNonSphericalGravityCoeffs: Attach spherical-harmonics gravity data to strDynParams.
            % options.charSpherHarmCoeffInputFileName: Optional SSphericalHarmonicsGravityData MAT/JSON file path.
            % options.ui16MaxSHdegree: Requested SH degree; 0 disables SH data.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objReferenceMissionData: Packaged SReferenceImagesDataset.
            % strEnvironmentData: Target/environment metadata for downstream scenario builders.
            % strDynParams: Dynamics parameters initialized from the scenario registry.
            % strScenarioMetadata: Registry scenario spec and generation metadata.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CScenarioRegistry, CScenarioGenerator.LoadDefaultScenarioData, CScenarioGenerator.packageDataset
            % -------------------------------------------------------------------------------------------------------------

            if size(dStateSC_W, 2) ~= numel(dTimestamps)
                error('CScenarioGenerator:InvalidScenarioDatasetInput', ...
                    'dStateSC_W must have one column per timestamp.');
            end

            [charTargetName, charTargetFixedFrame, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                enumScenarioName, struct(), ...
                "bUseKilometersScale", options.bUseKilometersScale, ...
                "charSpherHarmCoeffInputFileName", options.charSpherHarmCoeffInputFileName, ...
                "ui16MaxSHdegree", options.ui16MaxSHdegree, ...
                "bAddNonSphericalGravityCoeffs", options.bAddNonSphericalGravityCoeffs);

            if options.bUseKilometersScale
                charLengthUnits = "km";
            else
                charLengthUnits = "m";
            end

            strScenarioSpec = CScenarioRegistry.GetScenarioSpec(enumScenarioName, charLengthUnits=charLengthUnits);
            ui32NumSamples = uint32(numel(dTimestamps));

            dDCM_TBfromW = options.dDCM_TBfromW;
            dTargetPosition_W = options.dTargetPosition_W;
            dSunPosition_W = options.dSunPosition_W;
            dEarthPosition_W = options.dEarthPosition_W;

            if options.bCompleteFromReferences
                if isempty(dDCM_TBfromW)
                    dDCM_TBfromW = repmat(eye(3), 1, 1, double(ui32NumSamples));
                end
                if isempty(dTargetPosition_W)
                    dTargetPosition_W = zeros(3, double(ui32NumSamples));
                end
                if isempty(dSunPosition_W)
                    dSunPosition_W = zeros(3, double(ui32NumSamples));
                end
                if isempty(dEarthPosition_W)
                    dEarthPosition_W = zeros(3, double(ui32NumSamples));
                end
            end

            % Make reference mission datastruct
            objReferenceMissionData = CScenarioGenerator.packageDataset( ...
                options.objCamera, options.enumWorldFrame, dTimestamps, dStateSC_W, ...
                options.dDCM_SCfromW, dDCM_TBfromW, dTargetPosition_W, dSunPosition_W, ...
                dEarthPosition_W, options.dRelativeTimestamps);

            strEnvironmentData = struct( ...
                'charTargetName', charTargetName, ...
                'charTargetFixedFrame', charTargetFixedFrame, ...
                'enumScenarioName', strScenarioSpec.enumScenarioName, ...
                'charCanonicalName', strScenarioSpec.charCanonicalName, ...
                'dReferenceRadius', strScenarioSpec.dReferenceRadius, ...
                'dGravParam', strScenarioSpec.dGravParam);

            strScenarioMetadata = struct( ...
                'strScenarioSpec', strScenarioSpec, ...
                'bCompleteFromReferences', options.bCompleteFromReferences, ...
                'charLengthUnits', charLengthUnits);
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
                enumWorldFrame               (1,1)      {mustBeA(enumWorldFrame, ["EnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached
                dTimestamps                  (1,:)      double {mustBeNumeric} = [];
                dStateSC_W                   (6,:)      double {mustBeNumeric} = [];
                dDCM_SCfromW                 (3,3,:)    double {mustBeNumeric} = [];
                dDCM_TBfromW                 (3,3,:)    {mustBeNumeric} = [];
                dTargetPosition_W            (3,:)      {mustBeNumeric} = [];
                dSunPosition_W               (3,:)      {mustBeNumeric} = [];
                dEarthPosition_W             (3,:)      {mustBeNumeric} = [];
                dRelativeTimestamps          (1,:)      {mustBeNumeric} = [];
            end
            arguments
                optional.strAccelInfoData = []
                optional.dPrimaryPointingWhileMan_W   (3,:,:)  double {mustBeNumeric} = [] % TBC, primary pointing axis during manoeuvres
                optional.dSecondPointingWhileMan_W    (3,:,:)  double {mustBeNumeric} = [] % TBC, secondary axis during manoeuvres
                optional.dManoeuvresTimegrids         (3,:)    double {mustBeNumeric} = [];
                optional.dManoeuvresStartTimestamps   (1,:)    double {mustBeNumeric} = [];
                optional.dManoeuvresDeltaV_SC         (3,:)    double {mustBeNumeric} = [];
            end
            %% SIGNATURE
            % objReferenceMissionData = packageDataset(objCamera, enumWorldFrame, dTimestamps, dStateSC_W, ...
            %     dDCM_SCfromW, dDCM_TBfromW, dTargetPosition_W, dSunPosition_W, dEarthPosition_W, ...
            %     dRelativeTimestamps, optional)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Package state, attitude, ephemeris, and optional acceleration data into the standard
            % SReferenceImagesDataset container used by scenario and rendering workflows.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % objCamera: Camera intrinsics object attached to the dataset.
            % enumWorldFrame: Frame enum/name associated with the world-frame quantities.
            % dTimestamps: Absolute dataset timestamps.
            % dStateSC_W: Spacecraft position/velocity state history in the world frame.
            % dDCM_SCfromW: Optional spacecraft attitude DCM history.
            % dDCM_TBfromW: Target-body attitude DCM history.
            % dTargetPosition_W: Target position history in the world frame.
            % dSunPosition_W: Sun position history in the world frame.
            % dEarthPosition_W: Earth position history in the world frame.
            % dRelativeTimestamps: Optional relative timestamps; derived from dTimestamps when empty.
            % optional.strAccelInfoData: Optional acceleration component histories.
            % optional.dPrimaryPointingWhileMan_W: Optional primary pointing vectors during manoeuvres.
            % optional.dSecondPointingWhileMan_W: Optional secondary pointing vectors during manoeuvres.
            % optional.dManoeuvresTimegrids: Optional manoeuvre-relative time grids.
            % optional.dManoeuvresStartTimestamps: Optional manoeuvre start timestamps.
            % optional.dManoeuvresDeltaV_SC: Optional manoeuvre delta-V vectors in spacecraft frame.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objReferenceMissionData: Packaged reference image dataset.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SReferenceImagesDataset
            % -------------------------------------------------------------------------------------------------------------

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
