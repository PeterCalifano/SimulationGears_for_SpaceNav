classdef SReferenceImagesDataset < SReferenceMissionDesign % TODO the name of this class should change to something like SReferenceVisualNavDataset
    %% DESCRIPTION
    % Store mission-design states, source-provided target rates and camera data on one timegrid.
    % Preserve the mission-design frame, rate convention and length units during image-dataset conversion.
    % REQUIRED
    % Supply timestamped mission states, target attitudes and body positions through the base-class contract.
    % OPTIONAL
    % Attach camera intrinsics, the camera mounting rotation and acquisition masks.
    % Retain supplied target-model rates and mission metadata without resampling them.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 17-02-2025    Pietro Califano     Derived from SReferenceMissionDesign to add data necessary to use
    %                                   datasets for navigation simulations (e.g. camera params)
    % 29-06-2025    Pietro Califano     Complete extension to handle multiple bodies data
    % 22-12-2025    Pietro Califano     Extend conversion pipeline with intermediate representation class
    % 29-09-2026    Pietro Califano, Codex gpt-6    Preserve target rates and units during conversion.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % Construct directly or convert mission-design and simulation-state datasets through the static adapters.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % objCamera                 Camera intrinsics or projective camera model.
    % dDCM_CamFromSCB            Rotation from spacecraft body to camera coordinates.
    % bImageAcquisitionMask     Image-acquisition flags aligned with the mission timegrid.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % SReferenceMissionDesign, CCameraIntrinsics
    % -------------------------------------------------------------------------------------------------------------


    properties (SetAccess = public, GetAccess = public)
        objCamera {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
        dDCM_CamFromSCB        (3,3) double = eye(3);
        bImageAcquisitionMask (1,:) logical = false(0,0); % Mask for images acquisition
    end


    methods (Access = public)
        % CONSTRUCTOR
        function self = SReferenceImagesDataset(objCamera, ...
                                                enumWorldFrame, ...
                                                dTimestamps, ...
                                                dStateSC_W, ...
                                                dDCM_TBfromW, ...
                                                dTargetPosition_W, ...
                                                dSunPosition_W, ...
                                                dEarthPosition_W, ...
                                                optional)
            %% SIGNATURE
            % self = SReferenceImagesDataset(objCamera, enumWorldFrame, dTimestamps, dStateSC_W, ...)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Store mission-design data and camera parameters without deriving target angular velocity.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % objCamera         Camera model or intrinsics.
            % enumWorldFrame    Frame of the supplied mission states.
            % dTimestamps       Sample epochs [s].
            % dStateSC_W        Spacecraft position and velocity samples.
            % dDCM_TBfromW      Target attitude samples.
            % dTargetPosition_W, dSunPosition_W, dEarthPosition_W    Body positions in the selected length units.
            % optional          Mission metadata, including optional target-model rates [rad/s] on this grid.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self              Image dataset retaining the supplied mission metadata.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026    Pietro Califano, Codex gpt-6    Document and retain optional target rates.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SReferenceMissionDesign
            % -------------------------------------------------------------------------------------------------
            arguments (Input)
                % Reference definition
                objCamera                    (1,1)     {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
                enumWorldFrame               (1,:) char {mustBeA(enumWorldFrame, ["EnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached
                dTimestamps                  (1,:)     {mustBeNumeric} = [];
                dStateSC_W                   (6,:)    {mustBeNumeric} = [];
                dDCM_TBfromW                 (3,3,:)  {mustBeNumeric} = [];
                dTargetPosition_W            (3,:)    {mustBeNumeric} = [];
                dSunPosition_W               (3,:)    {mustBeNumeric} = [];
                dEarthPosition_W             (3,:)    {mustBeNumeric} = [];
            end
            arguments (Input)
                optional.dPrimaryPointingWhileMan_W   (3,:,:)  double {mustBeNumeric} = [] % TBC, primary pointing axis during manoeuvres
                optional.dSecondPointingWhileMan_W    (3,:,:)  double {mustBeNumeric} = [] % TBC, secondary axis during manoeuvres
                optional.dManoeuvresTimegrids         (3,:)    double {mustBeNumeric} = [];
                optional.dManoeuvresStartTimestamps   (1,:)    double {mustBeNumeric} = [];
                optional.dManoeuvresDeltaV_SC         (3,:)    double {mustBeNumeric} = [];
                optional.dRelativeTimestamps          (1,:)    double {mustBeNumeric} = [];   
                optional.dDCM_SCfromW                 (3,3,:)  double {mustBeNumeric} = [];
                optional.dDCM_CamFromSCB              (3,3) double = eye(3);
                optional.dTargetAngVel_IN             (3,:) double {mustBeNumeric} = [];
                optional.dTargetSpinAxis_TB           (3,:) double {mustBeNumeric} = [];
                optional.charTargetSpinAxisSource     (1,1) string {mustBeMember(optional.charTargetSpinAxisSource, ...
                    ["", "DEFAULT_PLUS_Z", "SCENARIO_DECLARED"])} = "";

                optional.charLengthUnits            char {mustBeA(optional.charLengthUnits, ["string", "char"])} = '';
            end
            arguments (Output)
                self (1,1) SReferenceImagesDataset
            end

            % Forward states and source metadata together to preserve their common timegrid.
            self = self@SReferenceMissionDesign(enumWorldFrame, ...
                                                dTimestamps, ...
                                                dStateSC_W, ...
                                                dDCM_TBfromW, ...
                                                dTargetPosition_W, ...
                                                dSunPosition_W, ...
                                                dEarthPosition_W, ...
                                                "dPrimaryPointingWhileMan_W", optional.dPrimaryPointingWhileMan_W, ...
                                                "dSecondPointingWhileMan_W", optional.dSecondPointingWhileMan_W,...
                                                "dManoeuvresTimegrids", optional.dManoeuvresTimegrids,...
                                                "dManoeuvresStartTimestamps", optional.dManoeuvresStartTimestamps,...
                                                "dManoeuvresDeltaV_SC", optional.dManoeuvresDeltaV_SC,...
                                                "dRelativeTimestamps", optional.dRelativeTimestamps, ...
                                                "dDCM_SCfromW", optional.dDCM_SCfromW, ...
                                                "dTargetAngVel_IN", optional.dTargetAngVel_IN, ...
                                                "dTargetSpinAxis_TB", optional.dTargetSpinAxis_TB, ...
                                                "charTargetSpinAxisSource", optional.charTargetSpinAxisSource);

            % Attach the camera model and its spacecraft mounting rotation.
            self.objCamera       = objCamera; 
            self.dDCM_CamFromSCB = optional.dDCM_CamFromSCB;

            % Retain the caller's unit label without rescaling numerical states.
            self.charLengthUnits = optional.charLengthUnits;
        end

        % GETTERS

        % SETTERS


        % METHODS

    end


    methods (Static)
        function objDataset = FromSimulationStates(objSimStatesArray, kwargs)
            arguments
                objSimStatesArray (1,:) {mustBeA(objSimStatesArray, "CSimulationState")}
            end
            arguments
                kwargs.objCamera                    (1,1)     {mustBeA(kwargs.objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
                kwargs.enumWorldFrame               (1,:)     char = ""
                kwargs.dManoeuvresTimegrids         (3,:)     double {mustBeNumeric} = []
                kwargs.dManoeuvresStartTimestamps   (1,:)     double {mustBeNumeric} = []
                kwargs.dManoeuvresDeltaV_SC         (3,:)     double {mustBeNumeric} = []
                kwargs.dEarthPosition_W             double {mustBeNumeric} = []
                kwargs.cellAdditionalBodiesTags     cell = {}
                kwargs.cellAdditionalTargetFrames   cell = {}
                kwargs.charLengthUnits              char {mustBeA(kwargs.charLengthUnits, ["string", "char"])} = "";
                kwargs.bImageAcquisitionMask        (1,:) logical = false(0,0);
            end
            % Method to convert from simulation states array to dataset object

            % Call base class method
            objMissionDataset = SReferenceMissionDesign.FromSimulationStates(objSimStatesArray, ...
                                                    "enumWorldFrame", kwargs.enumWorldFrame, ...
                                                    "dManoeuvresTimegrids", kwargs.dManoeuvresTimegrids, ...
                                                    "dManoeuvresStartTimestamps", kwargs.dManoeuvresStartTimestamps, ...
                                                    "dManoeuvresDeltaV_SC", kwargs.dManoeuvresDeltaV_SC, ...
                                                    "dEarthPosition_W", kwargs.dEarthPosition_W, ...
                                                    "cellAdditionalBodiesTags", kwargs.cellAdditionalBodiesTags, ...
                                                    "cellAdditionalTargetFrames", kwargs.cellAdditionalTargetFrames, ...
                                                    "charLengthUnits", kwargs.charLengthUnits);


            % Construct an instance of this and assign
            objDataset = SReferenceImagesDataset.FromSReferenceMissionDesign(objMissionDataset);
            objDataset.objCamera              = kwargs.objCamera;
            objDataset.bImageAcquisitionMask  = kwargs.bImageAcquisitionMask;
        end

        function objDataset = FromSimStatesIntermediateRepr(objSimStatesIntermediateRepr)
            arguments
                objSimStatesIntermediateRepr (1,1) SDatasetFromSimStateIntermediateRepr {mustBeA(objSimStatesIntermediateRepr, "SDatasetFromSimStateIntermediateRepr")}
            end

            % Call base class method
            objMissionDataset = SReferenceMissionDesign.FromSimStatesIntermediateRepr(objSimStatesIntermediateRepr);

            % Construct an instance of this and assign
            objDataset = SReferenceImagesDataset.FromSReferenceMissionDesign(objMissionDataset);
            objDataset.objCamera              = objSimStatesIntermediateRepr.objCamera;
            objDataset.bImageAcquisitionMask  = objSimStatesIntermediateRepr.bImageAcquisitionMask;
        end

        function [objSimStatesArray, dManoeuvresStartTimestamps, ...
                    dManoeuvresDeltaV_W, dManoeuvresTimegrids, ...
                    objCamera, bImageAcquisitionMask] = toSimulationStates(objDataset)
            arguments
                objDataset (1,1) {mustBeA(objDataset, "SReferenceImagesDataset")}
            end

            [objSimStatesArray, dManoeuvresStartTimestamps, dManoeuvresDeltaV_W, dManoeuvresTimegrids] = SReferenceMissionDesign.toSimulationStates(objDataset);
            
            % This class specific attributes
            objCamera             = objDataset.objCamera;
            bImageAcquisitionMask = self.bImageAcquisitionMask;

        end

        function objDataset = FromSReferenceMissionDesign(objReferenceMissionDesign)
            %% SIGNATURE
            % objDataset = SReferenceImagesDataset.FromSReferenceMissionDesign(objReferenceMissionDesign)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Convert mission-design data while preserving source rates, their timegrid and length units.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % objReferenceMissionDesign    Source mission-design dataset.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objDataset                   Image dataset with default intrinsics and unchanged mission data.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026    Pietro Califano, Codex gpt-6    Preserve target-model angular velocity and units.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CCameraIntrinsics, SReferenceImagesDataset
            % -------------------------------------------------------------------------------------------------
            arguments (Input)
                objReferenceMissionDesign (1,1) SReferenceMissionDesign {mustBeA(objReferenceMissionDesign, "SReferenceMissionDesign")}
            end
            arguments (Output)
                objDataset (1,1) SReferenceImagesDataset
            end

            % Use default camera intrinsics
            objCamera = CCameraIntrinsics();

            % Forward supplied rates with the states; do not infer them from sampled attitudes.
            objDataset = SReferenceImagesDataset(objCamera, ...
                                           objReferenceMissionDesign.enumWorldFrame, ...
                                           objReferenceMissionDesign.dTimestamps, ...
                                           objReferenceMissionDesign.dStateSC_W, ...
                                           objReferenceMissionDesign.dDCM_TBfromW, ...
                                           objReferenceMissionDesign.dTargetPosition_W, ...
                                           objReferenceMissionDesign.dSunPosition_W, ...
                                           objReferenceMissionDesign.dEarthPosition_W, ...
                                           'dPrimaryPointingWhileMan_W',  objReferenceMissionDesign.dPrimaryPointingWhileMan_W, ...
                                           'dSecondPointingWhileMan_W',   objReferenceMissionDesign.dSecondPointingWhileMan_W, ...
                                           'dManoeuvresTimegrids',        objReferenceMissionDesign.dManoeuvresTimegrids, ...
                                           'dManoeuvresStartTimestamps',  objReferenceMissionDesign.dManoeuvresStartTimestamps, ...
                                           'dManoeuvresDeltaV_SC',        objReferenceMissionDesign.dManoeuvresDeltaV_SC, ...
                                           'dRelativeTimestamps',         objReferenceMissionDesign.dRelativeTimestamps, ...
                                           'dDCM_SCfromW',                objReferenceMissionDesign.dDCM_SCfromW, ...
                                           'dTargetAngVel_IN',            objReferenceMissionDesign.dTargetAngVel_IN, ...
                                           'dTargetSpinAxis_TB',          objReferenceMissionDesign.dTargetSpinAxis_TB, ...
                                           'charTargetSpinAxisSource',    objReferenceMissionDesign.charTargetSpinAxisSource, ...
                                           'charLengthUnits',             objReferenceMissionDesign.charLengthUnits);
        end
    end


    methods (Access = protected)


    end
end
