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
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = public, GetAccess = public)
        objCamera {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
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
                enumWorldFrame               (1,1)     {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached
                dTimestamps                  (1,:)      {isnumeric, isvector} = [];
                dStateSC_W                   (6, :)     {isnumeric, ismatrix} = [];
                dDCM_TBfromW                 (3, 3, :)  {isnumeric, ismatrix} = [];
                dTargetPosition_W            (3, :)     {isnumeric, ismatrix} = [];
                dSunPosition_W               (3, :)     {isnumeric, ismatrix} = [];
                dEarthPosition_W             (3, :)     {isnumeric, ismatrix} = [];
            end
            arguments
                optional.dPrimaryPointingWhileMan_W   (3, :, :)  double {isnumeric, ismatrix} = [] % TBC, primary pointing axis during manoeuvres
                optional.dSecondPointingWhileMan_W    (3, :, :)  double {isnumeric, ismatrix} = [] % TBC, secondary axis during manoeuvres
                optional.dManoeuvresTimegrids         (3, :)     double {isnumeric, ismatrix} = [];
                optional.dManoeuvresStartTimestamps   (1, :)     double {isnumeric, isvector} = [];
                optional.dManoeuvresDeltaV_SC         (3, :)     double {isnumeric, ismatrix} = [];
                optional.dRelativeTimestamps          (1, :)     double {isnumeric, ismatrix} = [];   
                optional.dDCM_SCfromW                 (3, 3, :)  double {isnumeric, ismatrix} = []
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

        function objImagesDatasetFormatESA = exportAsFormatESA(self, charRootFolder, bSaveAllToDisk)
            arguments (Input)
                self            (1,1) SPoses3PointCloudImagesDataset
                charRootFolder  (1,:) char {mustBeA(charRootFolder, ["string", "char"])}
                bSaveAllToDisk  {islogical, isscalar}
            end
            arguments (Output)
                objImagesDatasetFormatESA (1,1)  {mustBeA(objImagesDatasetFormatESA, "SImagesDatasetFormatESA")}
            end
            %%% SIGNATURE
            % objImagesDatasetFormatESA = exportAsFormatESA(self, charRootFolder, bSaveAllToDisk)
            % -------------------------------------------------------------------------------------------------------------
            %%% DESCRIPTION
            % Function exporting the dataset object instance to an equivalent objImagesDatasetFormatESA
            % filling all available fields. Optionally, the method to export to disk is also called
            % -------------------------------------------------------------------------------------------------------------
            %%% INPUT
            % charRootFolder  (1,:) char {mustBeA(charRootFolder, ["string", "char"])}
            % bSaveAllToDisk  {islogical, isscalar}
            % -------------------------------------------------------------------------------------------------------------
            %%% OUTPUT
            % objImagesDatasetFormatESA (1,1)  {mustBeA(objImagesDatasetFormatESA, "SImagesDatasetFormatESA")}
            % -------------------------------------------------------------------------------------------------------------
            %%% CHANGELOG
            % 28-05-2025    Pietro Califano     Design and implementation of prototype
            % -------------------------------------------------------------------------------------------------------------

            % TODO, class specific

            % Convert to ESA format dataset object
            objImagesDatasetFormatESA = SImagesDatasetFormatESA(length(self.dTimestamps));
            % TODO
            
            if bSaveAllToDisk
                objImagesDatasetFormatESA.exportAllToDisk(charRootFolder);
            end
        end
    end


    methods (Static)
        function self = fromSReferenceMissionDesign(objReferenceMissionDesign)
            arguments
                objReferenceMissionDesign (1,1) SReferenceMissionDesign {mustBeA(objReferenceMissionDesign, "SReferenceMissionDesign")}
            end

            % Use default camera intrinsics
            objCamera = CCameraIntrinsics();

            % Build the new SReferenceImagesDataset by forwarding all the mission‐design data (including optionals).
            self = SReferenceImagesDataset( ...
                objCamera, ...
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

