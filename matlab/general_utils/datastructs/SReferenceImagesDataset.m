classdef SReferenceImagesDataset < SReferenceMissionDesign % TODO the name of this class should change to something like SReferenceVisualNavDataset
    %% DESCRIPTION
    % Datastruct containing essential information for spacecraft orbit and attitude as sequence of discrete
    % states on a discrete timegrid from SReferenceMissionDesign. Additionally, it stores information about
    % the camera parameters.
    % REQUIRED
    % TODO
    % OPTIONAL
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 17-02-2025    Pietro Califano     Derived from SReferenceMissionDesign to add data necessary to use
    %                                   datasets for navigation simulations (e.g. camera params)
    % 29-06-2025    Pietro Califano     Complete extension to handle multiple bodies data
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------


    properties (SetAccess = public, GetAccess = public)
        objCamera {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
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
            arguments
                % Reference definition
                objCamera                    (1,1)     {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
                enumWorldFrame               (1,:) char {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached
                dTimestamps                  (1,:)     {mustBeNumeric} = [];
                dStateSC_W                   (6,:)    {mustBeNumeric} = [];
                dDCM_TBfromW                 (3,3,:)  {mustBeNumeric} = [];
                dTargetPosition_W            (3,:)    {mustBeNumeric} = [];
                dSunPosition_W               (3,:)    {mustBeNumeric} = [];
                dEarthPosition_W             (3,:)    {mustBeNumeric} = [];
            end
            arguments
                optional.dPrimaryPointingWhileMan_W   (3,:,:)  double {mustBeNumeric} = [] % TBC, primary pointing axis during manoeuvres
                optional.dSecondPointingWhileMan_W    (3,:,:)  double {mustBeNumeric} = [] % TBC, secondary axis during manoeuvres
                optional.dManoeuvresTimegrids         (3,:)    double {mustBeNumeric} = [];
                optional.dManoeuvresStartTimestamps   (1,:)    double {mustBeNumeric} = [];
                optional.dManoeuvresDeltaV_SC         (3,:)    double {mustBeNumeric} = [];
                optional.dRelativeTimestamps          (1,:)    double {mustBeNumeric} = [];   
                optional.dDCM_SCfromW                 (3,3,:)  double {mustBeNumeric} = []
                optional.charLengthUnits            char {mustBeA(optional.charLengthUnits, ["string", "char"])} = '';
            end

            % Instantiate base by passing in all data
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
                                                "dDCM_SCfromW", optional.dDCM_SCfromW);

            % Store camera data as fields
            self.objCamera = objCamera; 

            % Store additional fields
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
                kwargs.dRelativeTimestamps          (1,:)     double {mustBeNumeric} = []
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
                                                    "dRelativeTimestamps", kwargs.dRelativeTimestamps, ...
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
            objMissionDataset = SReferenceMissionDesign.fromSimStatesIntermediateRepr(objSimStatesIntermediateRepr);

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
            arguments
                objReferenceMissionDesign (1,1) SReferenceMissionDesign {mustBeA(objReferenceMissionDesign, "SReferenceMissionDesign")}
            end

            % Use default camera intrinsics
            objCamera = CCameraIntrinsics();

            % Build the new SReferenceImagesDataset by forwarding all the mission‐design data (including optionals).
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
                                           'dDCM_SCfromW',                objReferenceMissionDesign.dDCM_SCfromW ...
                                           );
        end
    end


    methods (Access = protected)


    end
end
