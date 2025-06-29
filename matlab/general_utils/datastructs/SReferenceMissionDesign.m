classdef SReferenceMissionDesign < CBaseDatastructWithTimes
    %% DESCRIPTION
    % Datastruct containing essential information for spacecraft orbit and attitude as sequence of discrete
    % states on a discrete timegrid. The following data are included:
    % REQUIRED
    % TODO
    % OPTIONAL
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 01-02-2025    Pietro Califano     First prototype implementation
    % 11-02-2025    Pietro Califano     Upgrade and adaptation to use class for RCS1 trajectory data
    % 25-06-2025    Pietro Califano     Add new attributes to store additional bodies position data
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
        
        % Reference definition
        enumWorldFrame                  (1,1) {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached

        % Orbit and attitude data
        dPosSC_W                        (3, :) double {isnumeric, ismatrix} = zeros(3,0);
        dVelSC_W                        (3, :) double {isnumeric, ismatrix} = zeros(3,0);
        dDCM_SCfromW                    (3, 3, :) double {isnumeric, ismatrix} = zeros(3,3,0)
        strAccelInfoData                {isstruct} = struct() % Structure containing acceleration data           

        % Target body data 
        dDCM_TBfromW                    (3, 3, :) double {isnumeric, ismatrix} = []
        dTargetPosition_W               (3, :) double {isnumeric, ismatrix} = []

        % Manoeuvres plan data
        dPrimaryPointingWhileMan_W      (3, :, :) double {isnumeric, ismatrix} = [] % TBC, primary pointing axis during manoeuvres
        dSecondPointingWhileMan_W       (3, :, :) double {isnumeric, ismatrix} = [] % TBC, secondary axis during manoeuvres

        dManoeuvresTimegrids            (3, :) double {isnumeric, ismatrix} = [];    % TODO clarify what this is         
        dManoeuvresStartTimestamps      (1, :) double {isnumeric, isvector} = [];
        dManoeuvresDeltaV_SC            (3, :) double {isnumeric, ismatrix} = [];

        % Additional data 
        dSunPosition_W                  (3, :) double {isnumeric, ismatrix} = [];
        dEarthPosition_W                (3, :) double {isnumeric, ismatrix} = [];
        dRelativeTimestamps             (1, :) double {isnumeric, ismatrix} = [];

        cellAdditionalBodiesPos_W         {iscell} = {};
        cellAdditionalBodiesDCM_TBfromW   {iscell} = {};
        cellAdditionalTargetFrames        {iscell} = {};
        cellAdditionalBodiesTags          {iscell} = {};

        charLengthUnits char {mustBeA(charLengthUnits, ["string", "char"])} = '';
    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = SReferenceMissionDesign(enumWorldFrame, ...
                                                dTimestamps, ...
                                                dStateSC_W, ...
                                                dDCM_TBfromW, ...
                                                dTargetPosition_W, ...
                                                dSunPosition_W, ...
                                                dEarthPosition_W, ...
                                                optional)
            arguments
                % Reference definition
                enumWorldFrame               (1,1)     {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])} = EnumFrameName.IN  % Enumeration class indicating the W frame to which the data are attached
                dTimestamps                  (1, :)     {isnumeric, isvector} = [];
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
                optional.dDCM_SCfromW                 (3, 3, :)  double {isnumeric, ismatrix} = [];
            end
            
            % TODO (PC)
            % Add validity asserts, at least on timegrid and data size!

            % Instantiate base class
            self = self@CBaseDatastructWithTimes(dTimestamps);

            % Assign REQUIRED data
            self.enumWorldFrame             = enumWorldFrame               ;

            self.dPosSC_W                   = dStateSC_W(1:3, :)           ;
            self.dVelSC_W                   = dStateSC_W(4:6, :)           ;

            self.dDCM_TBfromW               = dDCM_TBfromW                 ;
            self.dTargetPosition_W          = dTargetPosition_W            ;
            
            self.dSunPosition_W             = dSunPosition_W               ;
            self.dEarthPosition_W           = dEarthPosition_W             ;

            % Automatic filling of target data if world frame is target fixed
            if enumWorldFrame == EnumFrameName.TB

                if isempty(self.dDCM_TBfromW)
                    self.dDCM_TBfromW = repmat(eye(3,3), 1, 1, length(dTimestamps));
                end

                if isempty(self.dTargetPosition_W)
                    self.dTargetPosition_W = zeros(3, length(dTimestamps));
                end

            end
            
            % Store OPTIONAL data
            self.dDCM_SCfromW               = optional.dDCM_SCfromW; 
            self.dPrimaryPointingWhileMan_W = optional.dPrimaryPointingWhileMan_W   ;
            self.dSecondPointingWhileMan_W  = optional.dSecondPointingWhileMan_W    ;
            self.dManoeuvresTimegrids       = optional.dManoeuvresTimegrids         ;
            self.dManoeuvresStartTimestamps = optional.dManoeuvresStartTimestamps   ;
            self.dManoeuvresDeltaV_SC       = optional.dManoeuvresDeltaV_SC         ;
            self.dRelativeTimestamps        = optional.dRelativeTimestamps          ;

            if nargin > 0 && not(isempty(dTimestamps))
                self.bDefaultConstructed = false;
            end
        end

        function self = scaleLengthUnits(self, dScaleFactor)
            arguments
                self
                dScaleFactor (1,1) double {isnumeric, isscalar}
            end
            self.dPosSC_W             = dScaleFactor * self.dPosSC_W;
            self.dVelSC_W             = dScaleFactor * self.dVelSC_W;
            self.dTargetPosition_W    = dScaleFactor * self.dTargetPosition_W;
            self.dSunPosition_W       = dScaleFactor * self.dSunPosition_W;
            self.dEarthPosition_W     = dScaleFactor * self.dEarthPosition_W;
            self.dManoeuvresDeltaV_SC = dScaleFactor * self.dManoeuvresDeltaV_SC;
        end
        % GETTERS
        
        % SETTERS
    end


    methods (Access = protected)


    end

    methods (Access = public, Static)

        function [objPosesPointCloudDataset] = ConvertToSPoses3PointCloudImagesDataset(objDataset)
            arguments (Input)
                objDataset (1,1) {mustBeA(objDataset, "SReferenceMissionDesign")}
            end
            arguments (Output)
                objPosesPointCloudDataset (1,1) {mustBeA(objPosesPointCloudDataset, "SPoses3PointCloud")}
            end

            % Define dataset object and copy data
            objPosesPointCloudDataset = SPoses3PointCloudImagesDataset();

            objPosesPointCloudDataset.dLightPosition_W  = objDataset.dSunPosition_W;
            objPosesPointCloudDataset.dFrame0Position_W = objDataset.dTargetPosition_W;
            objPosesPointCloudDataset.dFrame1Position_W = objDataset.dPosSC_W;
            objPosesPointCloudDataset.dDCM_Frame0FromW  = objDataset.dDCM_TBfromW;
            objPosesPointCloudDataset.dDCM_Frame1FromW  = objDataset.dDCM_SCfromW;

            try
                objPosesPointCloudDataset.objCameraIntrinsics = objDataset.objCameraIntrinsics;
            catch ME
                warning('Assignment of camera intrinsics failed with error: %s.', string(ME.message));
            end

        end

    end

end
