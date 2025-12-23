classdef SDatasetFromSimStateIntermediateRepr < CBaseDatastruct
    %% DESCRIPTION
    % 
    % REQUIRED
    % TODO
    % OPTIONAL
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 22-12-2025    Pietro Califano     First implementation to improve conversion pipeline
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
        objSimStatesArray (1,:) {mustBeA(objSimStatesArray, "CSimulationState")} = CSimulationState()
        objCamera         (1,1) {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
        enumWorldFrame                                                = []
        dManoeuvresTimegrids         (3,:)     double {mustBeNumeric} = [] % TODO may be not used? TBD
        dManoeuvresStartTimestamps   (1,:)     double {mustBeNumeric} = []
        dManoeuvresDeltaV_W          (3,:)     double {mustBeNumeric} = []
        dEarthPosition_W             double {mustBeNumeric} = []
        dRelativeTimestamps          (1,:)     double {mustBeNumeric} = []
        cellAdditionalBodiesTags     cell = {}
        cellAdditionalTargetFrames   cell = {}
        charLengthUnits              char {mustBeA(charLengthUnits, ["string", "char"])} = "";
        bImageAcquisitionMask        (1,:) logical = false(0,0);
    end

    properties (SetAccess = private, GetAccess = public)
        ui32NextAllocPtr (1,1) uint32 = uint32(1);
    end

    methods (Access = public)
        function self = SDatasetFromSimStateIntermediateRepr(ui32NumOfEntries, ...
                                                            objSimStatesArray, ...
                                                            objCamera, ...
                                                            enumWorldFrame, ...
                                                            charLengthUnits, ...
                                                            bImageAcquisitionMask, ...
                                                            dManoeuvresTimegrids, ...
                                                            dManoeuvresStartTimestamps, ...
                                                            dManoeuvresDeltaV_SC, ...
                                                            dEarthPosition_W, ...
                                                            cellAdditionalBodiesTags, ...
                                                            cellAdditionalTargetFrames)
            arguments
                ui32NumOfEntries  (1,1) uint32 = uint32(1);
                objSimStatesArray (1,:) {mustBeA(objSimStatesArray, "CSimulationState")} = CSimulationState()
                objCamera         (1,1) {mustBeA(objCamera, ["CCameraIntrinsics", "cameraIntrinsics", "CProjectiveCamera"])} = CCameraIntrinsics();
                enumWorldFrame               char                             = ""
                charLengthUnits              char {mustBeA(charLengthUnits, ["string", "char"])} = "";
                bImageAcquisitionMask        (1,:) logical = false(0,0);
                dManoeuvresTimegrids         (3,:)     double {mustBeNumeric} = []
                dManoeuvresStartTimestamps   (1,:)     double {mustBeNumeric} = []
                dManoeuvresDeltaV_SC         (3,:)     double {mustBeNumeric} = []
                dEarthPosition_W                       double {mustBeNumeric} = []
                cellAdditionalBodiesTags     cell = {} % TODO verify that 3rd bodies are converted from simulation states in conversion methods!
                cellAdditionalTargetFrames   cell = {}
            end

            if nargin > 0
                self.bDefaultConstructed = false;
            end

            % Store data from simulation states
            self.objSimStatesArray = objSimStatesArray;
            self.objCamera = objCamera;
            self.enumWorldFrame = enumWorldFrame;
            self.dManoeuvresTimegrids = dManoeuvresTimegrids;
            self.dManoeuvresStartTimestamps = dManoeuvresStartTimestamps;
            self.dManoeuvresDeltaV_W = dManoeuvresDeltaV_SC;
            self.dEarthPosition_W = dEarthPosition_W;
            self.cellAdditionalBodiesTags = cellAdditionalBodiesTags;
            self.cellAdditionalTargetFrames = cellAdditionalTargetFrames;
            self.charLengthUnits = charLengthUnits;
            self.bImageAcquisitionMask = bImageAcquisitionMask;

            % Pre-allocate arrays if nargin is = 1
            if nargin == 1
                self.objSimStatesArray = repmat(CSimulationState(), 1, ui32NumOfEntries);
                self.dEarthPosition_W = zeros(3, ui32NumOfEntries);
                self.bImageAcquisitionMask = false(1,ui32NumOfEntries);
            else
                self.ui32NextAllocPtr = uint32(numel(objSimStatesArray)) + uint32(1);
            end
        end

        function self = appendSimState(self, objSimState, dNewManoeuvreDeltaV_W, dNewManoeuvreTimestamp)
            arguments
                self
                objSimState (1,1) {mustBeA(objSimState, "CSimulationState")}
                dNewManoeuvreDeltaV_W (3,1) double {mustBeNumeric} = []
                dNewManoeuvreTimestamp (1,1) double {mustBeNumeric} = []
            end

            % Append simulation state and relevant data
            self.objSimStatesArray(self.ui32NextAllocPtr) = objSimState;
            self.dEarthPosition_W(:,self.ui32NextAllocPtr) = objSimState.dPosition_Earth_W;
            % self.dRelativeTimestamps(self.ui32NextAllocPtr) = objSimState.dRelativeTimestamp;

            %%% Add additional entries if input
            % Manoeuvres
            if ~isempty(dNewManoeuvreDeltaV_W)
                assert(~isempty(dNewManoeuvreTimestamp), 'Timestamp for the new manoeuvre must be provided if Delta-V is given.');
                self.dManoeuvresDeltaV_W(:,end+1) = dNewManoeuvreDeltaV_W;
            end
            if ~isempty(dNewManoeuvreTimestamp)
                assert(~isempty(dNewManoeuvreDeltaV_W), 'Delta-V for the new manoeuvre must be provided if timestamp is given.');
                self.dManoeuvresStartTimestamps(end+1) = dNewManoeuvreTimestamp;
            end

            % Third bodies data if any in the simulation state
            if isempty(self.cellAdditionalBodiesTags)
                self.cellAdditionalBodiesTags   = {};
                self.cellAdditionalTargetFrames = {};
            end

            % Bodies tags
            if ~isempty(objSimState.cellAdditionalBodiesTags)
                for ii = 1:length(objSimState.cellAdditionalBodiesTags)

                    charTag = objSimState.cellAdditionalBodiesTags{ii};
                    if ~ismember(charTag, self.cellAdditionalBodiesTags)
                        self.cellAdditionalBodiesTags{end+1} = charTag;
                        self.cellAdditionalTargetFrames{end+1} = objSimState.cellAdditionalTargetFrames{ii};
                    end

                end
            end

            % Update allocation pointer
            self.ui32NextAllocPtr = self.ui32NextAllocPtr + uint32(1);
        end
    end
end