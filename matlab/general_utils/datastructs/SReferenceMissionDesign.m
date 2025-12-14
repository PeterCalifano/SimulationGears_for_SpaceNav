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
    % 16-07-2025    Pietro Califano     Update class with new conveniency methods to produce plots and 
    %                                   attitude data directly from dataset
    % 14-12-2025    Pietro Califano     Implement conversion methods to/from simulation states arrays
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
        
        % Reference definition
        enumWorldFrame                  (1,1) {mustBeA(enumWorldFrame, ["SEnumFrameName", "string", "char"])} = "IN"  % Enumeration class indicating the W frame to which the data are attached

        % Orbit and attitude data
        dPosSC_W                        (3, :) double {mustBeNumeric} = zeros(3,0);
        dVelSC_W                        (3, :) double {mustBeNumeric} = zeros(3,0);
        dDCM_SCfromW                    (3, 3, :) double {mustBeNumeric} = zeros(3,3,0)
        strAccelInfoData                {isstruct} = struct() % Structure containing acceleration data           

        % Target body data 
        dDCM_TBfromW                    (3, 3, :) double {mustBeNumeric} = []
        dTargetPosition_W               (3, :) double {mustBeNumeric} = []

        % Manoeuvres plan data
        dPrimaryPointingWhileMan_W      (3, :, :) double {mustBeNumeric} = [] % TBC, primary pointing axis during manoeuvres
        dSecondPointingWhileMan_W       (3, :, :) double {mustBeNumeric} = [] % TBC, secondary axis during manoeuvres

        dManoeuvresTimegrids            (3, :) double {mustBeNumeric} = [];    % TODO clarify what this is         
        dManoeuvresStartTimestamps      (1, :) double {mustBeNumeric} = [];
        dManoeuvresDeltaV_SC            (3, :) double {mustBeNumeric} = [];

        % Additional data 
        dSunPosition_W                  (3, :) double {mustBeNumeric} = [];
        dEarthPosition_W                (3, :) double {mustBeNumeric} = [];
        dRelativeTimestamps             (1, :) double {mustBeNumeric} = [];
        enumTimeScale                   (1, :) char {mustBeText, mustBeMember(enumTimeScale, ...
                                                     ["TAI", "TDB", "TDT", "TT", "ET", "JDTDB", "JDTDT", "JED", "GPS", "N/D"])} = "N/D";
        
        ui32TargetPointingID              (1,:) uint32 {isnumeric} = []
        cellAdditionalBodiesPos_W         {mustBeA(cellAdditionalBodiesPos_W      , "cell")} = {};
        cellAdditionalBodiesDCM_TBfromW   {mustBeA(cellAdditionalBodiesDCM_TBfromW, "cell")} = {};
        cellAdditionalTargetFrames        {mustBeA(cellAdditionalTargetFrames     , "cell")} = {};
        cellAdditionalBodiesTags          {mustBeA(cellAdditionalBodiesTags       , "cell")} = {};

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
                dStateSC_W                   (6, :)     {mustBeNumeric} = [];
                dDCM_TBfromW                 (3, 3, :)  {mustBeNumeric} = [];
                dTargetPosition_W            (3, :)     {mustBeNumeric} = [];
                dSunPosition_W               (3, :)     {mustBeNumeric} = [];
                dEarthPosition_W             (3, :)     {mustBeNumeric} = [];
            end
            arguments
                optional.dPrimaryPointingWhileMan_W   (3, :, :)  double {mustBeNumeric} = [] % TBC, primary pointing axis during manoeuvres
                optional.dSecondPointingWhileMan_W    (3, :, :)  double {mustBeNumeric} = [] % TBC, secondary axis during manoeuvres
                optional.dManoeuvresTimegrids         (3, :)     double {mustBeNumeric} = [];
                optional.dManoeuvresStartTimestamps   (1, :)     double {isnumeric, isvector} = [];
                optional.dManoeuvresDeltaV_SC         (3, :)     double {mustBeNumeric} = [];
                optional.dRelativeTimestamps          (1, :)     double {mustBeNumeric} = [];   
                optional.dDCM_SCfromW                 (3, 3, :)  double {mustBeNumeric} = [];
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
                dScaleFactor (1,1) double {mustBeNumeric}
            end
            
            self.dPosSC_W             = dScaleFactor * self.dPosSC_W;
            self.dVelSC_W             = dScaleFactor * self.dVelSC_W;
            self.dTargetPosition_W    = dScaleFactor * self.dTargetPosition_W;
            self.dSunPosition_W       = dScaleFactor * self.dSunPosition_W;
            self.dEarthPosition_W     = dScaleFactor * self.dEarthPosition_W;
            self.dManoeuvresDeltaV_SC = dScaleFactor * self.dManoeuvresDeltaV_SC;

            if not(isempty(self.cellAdditionalBodiesPos_W))
                for idB = 1:length(self.cellAdditionalBodiesPos_W)
                    self.cellAdditionalBodiesPos_W{idB} = dScaleFactor * self.cellAdditionalBodiesPos_W{idB};
                end
            end

        end
        % GETTERS
        
        % SETTERS

        % METHODS
        function [dStateSC_W, self] = dStateSC_W(self, dStateSC_W)
            arguments
                self
                dStateSC_W {mustBeNumeric} = [];
            end

            if not(isempty(dStateSC_W))
                self.dPosSC_W = dStateSC_W(1:3, :);
                self.dVelSC_W = dStateSC_W(4:6, :);
            else
                dStateSC_W = [self.dPosSC_W; self.dVelSC_W];
            end

        end
    
        function plotInterpDatasetComparison(self, ...
                                            dTimeGridOriginal, ...
                                            dTimegrid, ...
                                            dInterpPosSC_W, ...
                                            dInterpVelSC_W, ...
                                            dInterpSunPosition_W, ...
                                            dInterpEarthPosition_W, ...
                                            dInterpTargetPosition_W, ...
                                            dInterpDCM_TBfromW)
            arguments
                self
                dTimeGridOriginal           double {mustBeVector}
                dTimegrid             double {mustBeVector}
                dInterpPosSC_W              double {ismatrix}
                dInterpVelSC_W              double {ismatrix}
                dInterpSunPosition_W        double {ismatrix}
                dInterpEarthPosition_W      double {ismatrix}
                dInterpTargetPosition_W     double {ismatrix}
                dInterpDCM_TBfromW          (:,:,:) double 
            end

            % Collect data from self dataset
            cellstrVectorNames = { ...
                'PosSC_W', 'VelSC_W', 'SunPosition_W', ...
                'EarthPosition_W', 'TargetPosition_W', "dDCM_TBfromW" ...
                };

            cellDataOrig   = { ...
                self.dPosSC_W, self.dVelSC_W, self.dSunPosition_W, ...
                self.dEarthPosition_W, self.dTargetPosition_W, self.dDCM_TBfromW, ...
                };

            cellDataInterp = { ...
                dInterpPosSC_W, dInterpVelSC_W, dInterpSunPosition_W, ...
                dInterpEarthPosition_W, dInterpTargetPosition_W, dInterpDCM_TBfromW ...
                };

            % Loop over each vector
            for ui32VecIdx = 1:numel(cellstrVectorNames)
                
                dTmpOrig        = cellDataOrig{ui32VecIdx};
                dTmpInterp      = cellDataInterp{ui32VecIdx};

                figure('Name', ...
                    sprintf('Compare - %s', cellstrVectorNames{ui32VecIdx}), ...
                    'NumberTitle','off');

                if contains(cellstrVectorNames{ui32VecIdx}, 'DCM')
                    % Convert DCM to quaternion sequence
                    dTmpOrig    = DCM2quatSeq(dTmpOrig, false);
                    dTmpInterp  = DCM2quatSeq(dTmpInterp, false);
                end

                ui32NumStates   = size(dTmpOrig,1);

                for ui32StateIdx = uint32(1):ui32NumStates

                    subplot(ui32NumStates, 1, double(ui32StateIdx));
                    plot(dTimegrid,    dTmpInterp(ui32StateIdx,:), ...
                        '.-', 'LineWidth', 1.2); 
                    hold on;
                    plot(dTimeGridOriginal,  dTmpOrig(ui32StateIdx,:), ...
                            '*',  'MarkerSize', 1, ...
                            'LineStyle', 'none');

                    ylabel(sprintf('%s (%u)', strrep(cellstrVectorNames{ui32VecIdx}, '_', ' '), ui32StateIdx));

                    if ui32StateIdx == ui32NumStates
                        xlabel('Time [s]');
                    else
                        set(gca,'XTickLabel',[]);
                    end

                    legend('Interpolated', 'Original', 'Location', 'best');
                    grid on;
                end

                sgtitle(sprintf('Interpolated vs Original - %s', strrep(cellstrVectorNames{ui32VecIdx}, '_', ' ') ));
            end
        end


        function objInterpDataset = linearInterpolate(self, kwargs)
            arguments
                self
            end
            arguments
                kwargs.dTargetTimegrid      double {isvector, isnumeric} = [];
                kwargs.dDeltaTimeMultiplier double {mustBeScalarOrEmpty} = [];
                kwargs.bDisplayComparison   logical {isscalar} = false;
            end

            objInterpDataset = copy(self);

            if not(isempty(kwargs.dTargetTimegrid))
                dTimegrid = kwargs.dTargetTimegrid;

            elseif not(isempty(kwargs.dDeltaTimeMultiplier))
                % Dataset timegrid definition
                dNewDeltaTime = kwargs.dDeltaTimeMultiplier * (self.dTimestamps(2) - self.dTimestamps(1));
                dTimegrid = objDataset.dTimestamps(1):dNewDeltaTime:objDataset.dTimestamps(end);
            else
                error('Invalid inputs: either dTargetTimegrid or dDeltaTimeMultiplier must be provided.')
            end

            dRelTimeGridTarget = dTimegrid - dTimegrid(1);

            % Interpolate position/velocity data
            dTimeGridOriginal = self.dTimestamps;
                objSplinerHandle = @(dInputData) spline(dTimeGridOriginal, dInputData, dTimegrid);

                dInterpPosSC_W          = objSplinerHandle( self.dPosSC_W );
                dInterpVelSC_W          = objSplinerHandle( self.dVelSC_W );
                dInterpSunPosition_W    = objSplinerHandle( self.dSunPosition_W );
                dInterpEarthPosition_W  = objSplinerHandle( self.dEarthPosition_W );
                dInterpTargetPosition_W = objSplinerHandle( self.dTargetPosition_W );

                % Interpolate target attitude
                dInterpDCM_TBfromW = zeros(3,3, length(dTimegrid));
                ui32InterpGridCheckID_  = zeros(1, length(dTimegrid), 'uint32');

                ui32InterPtr = uint32(1);

                for idQ = 1:length(dTimeGridOriginal)-1

                    % Get idQth interval limits
                    dTime0_ = dTimeGridOriginal(idQ);
                    dTime1_ = dTimeGridOriginal(idQ + 1);

                    % Get all times in the target timegrid within it idQ interval
                    bInitMask = dTimegrid >= dTime0_;
                    bEndMask  = dTimegrid < dTime1_;
                    bExtractionIdx = bInitMask & bEndMask;

                    dInterpTargetTimesInInterval_ = dTimegrid(bExtractionIdx);
                    ui32TargetIndices = find(bExtractionIdx > 0);

                    % Compute quaternion from DCMs
                    dDCM0_ = self.dDCM_TBfromW(:,:, idQ);
                    dDCM1_ = self.dDCM_TBfromW(:,:, idQ+1);

                    % Interpolate quaternions
                    dQuat0_ = DCM2quatSeq(dDCM0_, false);
                    dQuat1_ = DCM2quatSeq(dDCM1_, false);

                    dNormalizedInterpTargetTimesInInterval_ = (dInterpTargetTimesInInterval_ - dTime0_) / (dTime1_ - dTime0_); % Normalize timegrid to [0,1]
                    dInterpTargetBodyQuat_TBfromW = InterpolateSlerp(dQuat0_, dQuat1_, dNormalizedInterpTargetTimesInInterval_);

                    % Convert back to DCMs and allocate
                    ui32NumNewInterpEntries = uint32(length(dInterpTargetTimesInInterval_));
                    dInterpDCM_TBfromW(1:3, 1:3, ui32InterPtr:ui32InterPtr+ui32NumNewInterpEntries-1) = QuatSeq2DCM(dInterpTargetBodyQuat_TBfromW, false);
                    ui32InterpGridCheckID_(ui32InterPtr:ui32InterPtr+ui32NumNewInterpEntries-1) = ui32TargetIndices;

                    % Update ptr
                    ui32InterPtr = ui32InterPtr + ui32NumNewInterpEntries;
                end


                % TODO interpolate dInterpDCM_SCfromW
                dInterpDCM_SCfromW = [];

                % Add last time instant
                dInterpDCM_TBfromW(:,:,end) = self.dDCM_TBfromW(:,:, end);
                assert(length(unique(ui32InterpGridCheckID_)) == length(dTimegrid), 'ERROR: duplicated indices found in interpolation index. Please check allocation logic!');

                if kwargs.bDisplayComparison
                    % Plot interpolated vs original data
                    self.plotInterpDatasetComparison(dTimeGridOriginal, ...
                                                dTimegrid         , ...
                                                dInterpPosSC_W          , ...
                                                dInterpVelSC_W          , ...
                                                dInterpSunPosition_W    , ...
                                                dInterpEarthPosition_W  , ...
                                                dInterpTargetPosition_W, ...
                                                dInterpDCM_TBfromW);
                end


                % Update fields
                objInterpDataset.dTimestamps       = dTimegrid;
                objInterpDataset.dPosSC_W          = dInterpPosSC_W;
                objInterpDataset.dVelSC_W          = dInterpVelSC_W;
                objInterpDataset.dSunPosition_W    = dInterpSunPosition_W;
                objInterpDataset.dEarthPosition_W  = dInterpEarthPosition_W;
                objInterpDataset.dTargetPosition_W = dInterpTargetPosition_W;
                objInterpDataset.dDCM_TBfromW      = dInterpDCM_TBfromW;
                    
                objInterpDataset.dRelativeTimestamps = dRelTimeGridTarget;

                if not(isempty(objInterpDataset.dDCM_SCfromW)) && not(isempty(dInterpDCM_SCfromW))
                    objInterpDataset.dDCM_SCfromW = dInterpDCM_SCfromW;
                end

        end


        function [objFig_PosVel, objFig_PhaseAngle, objTargetAttitudeFig] = plotDatasetData(self, kwargs)
            arguments
                self            (1,1) {mustBeA(self, "SReferenceMissionDesign")}
            end
            arguments
                kwargs.ui32TargetIndex          (1,1) uint32 {isscalar} = 0
                kwargs.bPlotPhaseAngles         (1,1) logical {islogical, isscalar} = true
                kwargs.bPlotTargetAttitude      (1,1) logical {islogical, isscalar} = false
                kwargs.bPlotSpacecraftAttitude  (1,1) logical {islogical, isscalar} = false
                kwargs.dTargetAttitudeSet2          double {ismatrix} = [];
                kwargs.charLblTargetAttitudeSet2    char {mustBeText} = '2nd attitude set';
                kwargs.charTargetAttitudePlotTitle  char {mustBeText} = 'Target attitude quaternion states',
            end
            % DEVNOTE: basic version of method to plot visualizations of data contained in the dataset
            % object. May be expanded with many functionalities and options

            % Plot position and velocity in world frame
            objFig_PosVel = PlotDatasetPositionVelocity(self);

            % Plot phase angle evolution in time
            objFig_PhaseAngle = [];
            if kwargs.ui32TargetIndex == 0
                dTarget_W = self.dTargetPosition_W;
            else
                try
                    dTarget_W = self.cellAdditionalBodiesPos_W{kwargs.ui32TargetIndex};
                catch ME
                    warning('Error in getting position of target body ID%d: %s. Are you sure that the dataset contain any additional body?', ...
                        kwargs.ui32TargetIndex, string(ME.message));
                    return
                end
            end

            if kwargs.bPlotPhaseAngles
                objFig_PhaseAngle = figure;

                dCamPosFromTarget_W = self.dPosSC_W - dTarget_W;
                dSunPosFromTarget_W = self.dSunPosition_W - dTarget_W;
                dPhaseAngles = acosd(dot(dCamPosFromTarget_W./vecnorm(dCamPosFromTarget_W, 2, 1), dSunPosFromTarget_W./vecnorm(dSunPosFromTarget_W, 2, 1)));

                plot(self.dTimestamps, dPhaseAngles, 'r-', 'DisplayName', 'Dataset poses', 'LineWidth', 1.05);
                % plot(objDataset.dTimestamps(bImageAcquisitionMask), dPhaseAngles(bImageAcquisitionMask), ...
                %     'r.', 'DisplayName', 'Image acquisition', 'linestyle', 'none');
                xlabel('Time ET [s]')
                ylabel('Phase angle [deg]')
                grid minor
                legend();
            end

            if kwargs.bPlotTargetAttitude
                % Plot target attitude if required
                dDataVec1 = self.dDCM_TBfromW;
                dDataVec2 = kwargs.dTargetAttitudeSet2;
                assert(isempty(kwargs.dTargetAttitudeSet2) || (ndims(dDataVec2) == 3 && size(dDataVec2,1) == 3 && size(dDataVec2,2) == 3), ...
                    'ERROR: set 2 for target attitude is not a valid array. Must be [3,3,N] containing DCM data or empty.');

                % Plot target attitude data
                objTargetAttitudeFig = self.plotVectorData_(dDataVec1, ...
                                                         dDataVec2, ...
                                                        "bIsDataDCM", true, ...
                                                        "cellSetNames", {'Reference', kwargs.charLblTargetAttitudeSet2}, ...
                                                        "charFigTitle", kwargs.charTargetAttitudePlotTitle);
            else
                objTargetAttitudeFig = [];
            end

            if kwargs.bPlotSpacecraftAttitude
                % Plot target attitude if required
                % TODO
            end
        end

        function [objSceneFig] = visualizeTrajectory3dSceneAndPoses(self)
            arguments
                self (1,1) {mustBeA(self, "SReferenceMissionDesign")}
            end
            % Plot 3D trajectory in target fixed frame
            [objSceneFig] = VisualizeTrajectory3dSceneAndPoses(self);
        end
    end

    methods (Access = protected)
        function [objFig] = plotVectorData_(self, dDataVec1, dDataVec2, kwargs)
            arguments
                self
                dDataVec1  {isnumeric}
                dDataVec2  = []
            end
            arguments
                kwargs.cellSetNames     {iscell} = {'Set1', 'Set2'};
                kwargs.cellStatesNames  {iscell} = {}
                kwargs.bIsDataDCM       (1,1) logical = false
                kwargs.ui32Decimation   (1,1) uint32 = 1
                kwargs.charFigTitle     {mustBeText} = ""
            end

            % TODO improve robustness to incorrect sizes and inputs!

            % objFig = figure('Name', ...
            %                 sprintf('%s', charFigName), ...
            %                 'NumberTitle','off');
            objFig = figure();
            set(objFig, 'Renderer', 'opengl'); %#ok<FGREN>
            dTimegrid = self.dTimestamps;
            assert(length(kwargs.cellSetNames) <= 2, 'ERROR: current implementation only supports plots containing 2 sets of vector data.');
                
            ui32Size = length(dTimegrid);
            bDecimationMask = true(1, ui32Size);
            if kwargs.ui32Decimation > 1
                bDecimationMask(2:kwargs.ui32Decimation:ui32Size) = false;
            end

            if kwargs.bIsDataDCM
                % Convert DCM to quaternion sequence
                dDataVec1    = DCM2quatSeq(dDataVec1, false);

                if not(isempty(dDataVec2))
                    dDataVec2  = DCM2quatSeq(dDataVec2, false);
                end
            end

            % Get number of plot entries
            ui32NumStates   = size(dDataVec1,1);
            ui32NumCols = floor(ui32NumStates/2);
            ui32NumRows = ceil( ui32NumStates / ui32NumCols);

            % Plot states in tiled layout
            tiledlayout(ui32NumRows, ui32NumCols, ...
                       "TileSpacing", "compact");

            for ui32StateIdx = uint32(1):ui32NumStates
                nexttile;
                    
                % Plot data set 1
                cellObjPlots{1,1} = plot(dTimegrid(bDecimationMask), dDataVec1(ui32StateIdx, bDecimationMask), '.-', ...
                                        'LineWidth', 1.0, ...
                                        'MarkerSize', 4, ...
                                        'Marker', 'x', ...
                                        'DisplayName', string(kwargs.cellSetNames{1})); 
                hold on;

                if not(isempty(dDataVec2))
                    % Plot data set 2
                    cellObjPlots{1,2} = plot(dTimegrid(bDecimationMask),  dDataVec2(ui32StateIdx, bDecimationMask), ...
                                        '--',  'MarkerSize', 1, ...
                                        'LineWidth', 1.3, ...
                                        'LineStyle', '--', ...
                                        'DisplayName', string(kwargs.cellSetNames{2}));
                end

                if not(isempty(kwargs.cellStatesNames))
                    ylabel(sprintf('%s (%u)', strrep(kwargs.cellStatesNames{ui32VecIdx}, '_', ' '), ui32StateIdx));
                else
                    ylabel(sprintf('State %d', ui32StateIdx));
                end

                if ui32StateIdx == ui32NumStates || mod(ui32StateIdx, ui32NumRows) == 0
                    xlabel('Time [s]');
                else
                    set(gca,'XTickLabel',[]);
                end

                if ui32StateIdx == 1
                    legend([cellObjPlots{:}], 'Location', 'best');
                end
                grid on;

            end

            sgtitle(kwargs.charFigTitle)
        end

    end

    methods (Access = public, Static)
        function [objDataset] = fromSimulationStates(objSimStatesArray, kwargs)
            arguments
                objSimStatesArray (1,:) {mustBeA(objSimStatesArray, "CSimulationState")}
            end
            arguments
                kwargs.enumWorldFrame               = []
                kwargs.dManoeuvresTimegrids         (3,:) double {mustBeNumeric} = []
                kwargs.dManoeuvresStartTimestamps   (1,:) double {mustBeNumeric} = []
                kwargs.dManoeuvresDeltaV_W          (3,:) double {mustBeNumeric} = []
                kwargs.dEarthPosition_W                   double {mustBeNumeric} = []
                kwargs.dRelativeTimestamps          (1,:) double {mustBeNumeric} = []
                kwargs.cellAdditionalBodiesTags         cell = {}
                kwargs.cellAdditionalTargetFrames       cell = {}
                kwargs.charLengthUnits              char {mustBeA(kwargs.charLengthUnits, ["string", "char"])} = ""
            end
            % Method to convert from simulation states array to dataset object

            %%% Function code
            ui32NumStates = numel(objSimStatesArray);
            assert(ui32NumStates > 0, 'ERROR: fromSimulationStates called with empty input array.');

            % Initialize variables
            dPosSC_W            = zeros(3, ui32NumStates);
            dVelSC_W            = zeros(3, ui32NumStates);
            dDCM_SCfromW        = zeros(3,3, ui32NumStates);
            dTargetPosition_W   = zeros(3, ui32NumStates);
            dDCM_TBfromW        = zeros(3,3, ui32NumStates);
            dSunPosition_W      = zeros(3, ui32NumStates);
            dTimestamps         = zeros(1, ui32NumStates);
            dRelativeTimestamps = zeros(1, ui32NumStates);

            cellAdditionalBodiesPos_W       = {};
            cellAdditionalBodiesDCM_TBfromW = {};

            % Loop over simulation states to fetch data at each time ID
            bHasVelocity = isprop(objSimStatesArray(1).objCameraPose_W, 'dVelocity_Frame');

            for idT = 1:ui32NumStates
                
                % Get ith state
                objState = objSimStatesArray(idT);

                % Get time data
                dTimestamps(idT) = objState.dTimestamp;
                dRelativeTimestamps(idT) = objState.dRelativeTimestamp;

                % Get scene data (geometric)
                % Camera data
                dPosSC_W(:, idT) = objState.objCameraPose_W.translation();

                if bHasVelocity
                    dVelSC_W(:, idT) = objState.objCameraPose_W.dVelocity_Frame;
                end

                dRotCam_WfromSC = objState.objCameraPose_W.rotation();

                if isempty(dRotCam_WfromSC)
                    dRotCam_WfromSC = eye(3);
                end

                dDCM_SCfromW(:,:, idT) = transpose(dRotCam_WfromSC);

                % Target data
                dTargetPosition_W(:, idT) = objState.objTargetPose_W.translation();
                dRotTarget_WfromOF = objState.objTargetPose_W.rotation();

                if isempty(dRotTarget_WfromOF)
                    dRotTarget_WfromOF = eye(3);
                end

                dDCM_TBfromW(:,:, idT) = transpose(dRotTarget_WfromOF);

                % Sun position in World frame
                if not(isempty(objState.dSunPosition_W))
                    dSunPosition_W(:, idT) = objState.dSunPosition_W;
                end

                % Handle additional bodies if present
                if objState.ui32Num3rdBodies > 0

                    % Define cells for storage
                    if isempty(cellAdditionalBodiesPos_W)
                        cellAdditionalBodiesPos_W       = cell(1, objState.ui32Num3rdBodies);
                        cellAdditionalBodiesDCM_TBfromW = cell(1, objState.ui32Num3rdBodies);
                    end

                    % Get data
                    for idB = 1:objState.ui32Num3rdBodies

                        % Initialize storage
                        if isempty(cellAdditionalBodiesPos_W{idB})
                            cellAdditionalBodiesPos_W{idB} = zeros(3, ui32NumStates); %#ok<AGROW>
                        end

                        cellAdditionalBodiesPos_W{idB}(:, idT) = objState.obj3rdTargetPose_W(idB).translation(); %#ok<AGROW>
                        
                        dRot3rd_WfromOF = objState.obj3rdTargetPose_W(idB).rotation();

                        if not(isempty(dRot3rd_WfromOF))
                            % Store attitude
                            if isempty(cellAdditionalBodiesDCM_TBfromW{idB})
                                cellAdditionalBodiesDCM_TBfromW{idB} = zeros(3, 3, ui32NumStates); %#ok<AGROW>
                            end
                            cellAdditionalBodiesDCM_TBfromW{idB}(:, :, idT) = transpose(dRot3rd_WfromOF); %#ok<AGROW>
                        end
                    end
                end

            end

            % Additional information
            if isempty(kwargs.dRelativeTimestamps)
                dRelativeTimestampsOut = dRelativeTimestamps;
                
                if all(dRelativeTimestampsOut == 0)
                    dRelativeTimestampsOut = dTimestamps - dTimestamps(1);
                end
            
            else
                dRelativeTimestampsOut = kwargs.dRelativeTimestamps;
            end

            if isempty(kwargs.enumWorldFrame)
                enumWorldFrame = objSimStatesArray(1).enumWorldFrame;
                if isempty(enumWorldFrame)
                    enumWorldFrame = EnumFrameName.IN;
                end
            else
                enumWorldFrame = kwargs.enumWorldFrame;
            end

            if isempty(kwargs.dEarthPosition_W)
                dEarthPosition_W = zeros(3, ui32NumStates);
            else
                dEarthPosition_W = kwargs.dEarthPosition_W;
            end

            % Assemble instance of object
            objDataset = SReferenceMissionDesign(enumWorldFrame, ...
                                                dTimestamps, ...
                                                [dPosSC_W; dVelSC_W], ...
                                                dDCM_TBfromW, ...
                                                dTargetPosition_W, ...
                                                dSunPosition_W, ...
                                                dEarthPosition_W, ...
                                                "dManoeuvresTimegrids", kwargs.dManoeuvresTimegrids, ...
                                                "dManoeuvresStartTimestamps", kwargs.dManoeuvresStartTimestamps, ...
                                                "dManoeuvresDeltaV_SC", kwargs.dManoeuvresDeltaV_W, ...
                                                "dRelativeTimestamps", dRelativeTimestampsOut, ...
                                                "dDCM_SCfromW", dDCM_SCfromW);

            % Assign data
            objDataset.cellAdditionalBodiesPos_W       = cellAdditionalBodiesPos_W;
            objDataset.cellAdditionalBodiesDCM_TBfromW = cellAdditionalBodiesDCM_TBfromW;

            if isempty(kwargs.cellAdditionalBodiesTags) && not(isempty(cellAdditionalBodiesPos_W))
                objDataset.cellAdditionalBodiesTags = arrayfun(@(idB) sprintf("Body%d", idB), 1:length(cellAdditionalBodiesPos_W), 'UniformOutput', false);
            else
                objDataset.cellAdditionalBodiesTags = kwargs.cellAdditionalBodiesTags;
            end

            objDataset.cellAdditionalTargetFrames = kwargs.cellAdditionalTargetFrames;
            objDataset.charLengthUnits            = kwargs.charLengthUnits;
        end

        function [objSimStatesArray, dManoeuvresStartTimestamps, dManoeuvresDeltaV_W, dManoeuvresTimegrids] = toSimulationStates(objDataset)
            arguments
                objDataset (1,1) {mustBeA(objDataset, "SReferenceMissionDesign")}
            end
            % Method to convert from dataset object to simulation states array

            if isempty(which('CSimulationState'))
                error('SReferenceMissionDesign:MissingDependency', 'CSimulationState is not on the MATLAB path. Add nav-backend/matlab/src/datastructs to use this conversion.');
            end
            if isempty(which('SNavState'))
                error('SReferenceMissionDesign:MissingDependency', 'SNavState is not on the MATLAB path. Add nav-backend/matlab/src/datastructs to use this conversion.');
            end

            % Get number of states
            ui32NumStates = length(objDataset.dTimestamps);

            % Get manoeuvres information
            if ui32NumStates == 0
                objSimStatesArray = CSimulationState.empty(1,0);
                dManoeuvresStartTimestamps = objDataset.dManoeuvresStartTimestamps;
                dManoeuvresDeltaV_W       = objDataset.dManoeuvresDeltaV_SC;
                dManoeuvresTimegrids       = objDataset.dManoeuvresTimegrids;
                return
            end

            % Loop over timestamps to build array
            cellStates = cell(1, ui32NumStates);
            for idT = 1:ui32NumStates

                % Get camera data
                dTimestamp     = objDataset.dTimestamps(idT);
                dPos_W         = objDataset.dPosSC_W(:, idT);
                dVel_W         = objDataset.dVelSC_W(:, idT);

                if isempty(objDataset.dDCM_SCfromW)
                    dDCM_SCfromW = eye(3);
                else
                    dDCM_SCfromW = objDataset.dDCM_SCfromW(:, :, idT);
                end

                % Get target data
                if isempty(objDataset.dDCM_TBfromW)
                    dDCM_TBfromW = eye(3);
                else
                    dDCM_TBfromW = objDataset.dDCM_TBfromW(:, :, idT);
                end

                % Initial camera and target instances in world frame
                objCameraPose_W = SNavState(dTimestamp, dPos_W, dVel_W, dDCM_SCfromW');
                objTargetPose_W = SPose3(objDataset.dTargetPosition_W(:, idT), dDCM_TBfromW');

                % Initial camera and target instances in target frame
                dPos_TB = dDCM_TBfromW * dPos_W;
                dVel_TB = dDCM_TBfromW * dVel_W;
                dDCM_SCfromTB = dDCM_TBfromW * dDCM_SCfromW;
                objCameraPose_TB = SNavState(dTimestamp, dPos_TB, dVel_TB, dDCM_SCfromTB');

                % Assemble simulation state entry
                objState = CSimulationState(objCameraPose_W, objTargetPose_W, objCameraPose_TB);
                
                % Assign state data
                objState.enumWorldFrame = objDataset.enumWorldFrame;
                objState.ui32TimestampID = uint32(idT);
                objState.dTimestamp = dTimestamp;

                if isempty(objDataset.dRelativeTimestamps)
                    objState.dRelativeTimestamp = dTimestamp - objDataset.dTimestamps(1);
                else
                    objState.dRelativeTimestamp = objDataset.dRelativeTimestamps(idT);
                end

                if not(isempty(objDataset.dSunPosition_W))
                    objState.dSunPosition_W = objDataset.dSunPosition_W(:, idT);
                end

                objState.objCameraPose_W.dTimestamp  = dTimestamp;
                objState.objCameraPose_TB.dTimestamp = dTimestamp;

                % Assign 3rd bodies if present
                if not(isempty(objDataset.cellAdditionalBodiesPos_W))

                    for idB = 1:length(objDataset.cellAdditionalBodiesPos_W)
                        dPosBody_W = objDataset.cellAdditionalBodiesPos_W{idB}(:, idT);
                        dDCM_BodyFromW = eye(3);

                        
                        if not(isempty(objDataset.cellAdditionalBodiesDCM_TBfromW)) && ...
                            length(objDataset.cellAdditionalBodiesDCM_TBfromW) >= idB && ...
                             not(isempty(objDataset.cellAdditionalBodiesDCM_TBfromW{idB}))
                        
                            dDCM_BodyFromW = objDataset.cellAdditionalBodiesDCM_TBfromW{idB}(:, :, idT);
                        end

                        objState.obj3rdTargetPose_W(idB) = SPose3(dPosBody_W, dDCM_BodyFromW');

                        dPosCamFromBody = dDCM_BodyFromW * (dPos_W - dPosBody_W);
                        dDCM_CamFromBody = dDCM_BodyFromW * dDCM_SCfromW;
                        objState.obj3rdCameraPose_TB(idB) = SPose3(dPosCamFromBody, dDCM_CamFromBody');
                    end

                end

                cellStates{idT} = objState;
            end

            % Flatten cell to array
            objSimStatesArray = [cellStates{:}];
            dManoeuvresStartTimestamps = objDataset.dManoeuvresStartTimestamps;
            dManoeuvresDeltaV_W        = objDataset.dManoeuvresDeltaV_SC;
            dManoeuvresTimegrids       = objDataset.dManoeuvresTimegrids;
        end

        function [self, dCameraAttDCM_WfromOF, dRot3Param, dOffPointingAngles] = generateAttitudePointing(self, kwargs)
            arguments
                self (1,1) {mustBeA(self, "SReferenceMissionDesign")}
            end
            arguments
                kwargs.placeholder = [];
            end
            % TODO add kwargs and doc

            % Call static method
            [dCameraAttDCM_WfromOF, dRot3Param, dOffPointingAngles] = SReferenceMissionDesign.GenerateAttitudePointingStatic(self, kwargs);

            % Assign camera attitude
            self.dDCM_SCfromW = pagetranspose(dCameraAttDCM_WfromOF);
        end

        function [dCameraAttDCM_WfromOF, dRot3Param, dOffPointingAngles] = GenerateAttitudePointingStatic(objDataset, kwargs)
            arguments
                objDataset (1,1) {mustBeA(objDataset, "SReferenceMissionDesign")}
            end
            arguments
                kwargs.bShowAttitudePointingPlot       (1,1) logical {islogical, isscalar} = false
                kwargs.dSigmaBoresightRollAngle        (1,1) double {mustBeNumeric} = 0.0;
                kwargs.dSigmaOffPointingDegAngle       (1,1) double {mustBeNumeric} = 0.0;
                kwargs.enumConstraintType              (1,:) string {mustBeMember(kwargs.enumConstraintType, ["YorthogonalSun", "trackLVLH", "auxiliaryAxis"])} = "YorthogonalSun"
                kwargs.enumOutRot3Param                (1,1) EnumRotParams {mustBeA(kwargs.enumOutRot3Param, ["EnumRotParams", "string"])} = EnumRotParams.DCM
                kwargs.dDCM_displacedPoseFromPose      (3,3,:) double {mustBeNumeric} = zeros(3,3) % Custom rotation to apply to the rotation
                kwargs.enumOffPointingMode             (1,1) string {mustBeMember(kwargs.enumOffPointingMode, ["randomAxis", "refAxisOutOfPlane", "refAxisInPlane"])} = "randomAxis";
                kwargs.dReferenceAxis_Frame            (3,:) double {mustBeNumeric} = zeros(3,0)
                kwargs.enumDisplaceDistribution           (1,:) string {mustBeMember(kwargs.enumDisplaceDistribution, ...
                    ["uniform", "gaussian", "time_correlation", "gaussian_same_on_batch", "uniform_same_on_batch"])} = "gaussian";
            end
            
            % TODO add kwargs
            objPointingGenerator = CAttitudePointingGenerator(objDataset.dPosSC_W, ...
                                                                objDataset.dTargetPosition_W, ...
                                                                objDataset.dSunPosition_W, ...
                                                                "bShowAttitudePointingPlot", kwargs.bShowAttitudePointingPlot);


            [objPointingGenerator, dRot3Param, dCameraAttDCM_WfromOF, dOffPointingAngles] = objPointingGenerator.pointToTarget(...
                                                                                    "dSigmaDegRotAboutBoresight", kwargs.dSigmaBoresightRollAngle, ...
                                                                                    "enumOffPointingMode", kwargs.enumOffPointingMode, ...
                                                                                    "dSigmaOffPointingDegAngle", kwargs.dSigmaOffPointingDegAngle, ...
                                                                                    "enumDisplaceDistribution", kwargs.enumDisplaceDistribution, ...
                                                                                    "dReferenceAxis_Frame", kwargs.dReferenceAxis_Frame, ...
                                                                                    "enumConstraintType", kwargs.enumConstraintType, ...
                                                                                    "enumOutRot3Param", kwargs.enumOutRot3Param, ...
                                                                                    "dDCM_displacedPoseFromPose", kwargs.dDCM_displacedPoseFromPose); %#ok<*UNRCH>
        end
  

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

        function [objFig] = PlotDatasetPositionVelocityStatic(objDataset)
            arguments
                objDataset (1,1) {mustBeA(objDataset, "SReferenceMissionDesign")}
            end

            % Plot position and velocity in world frame
            objFig = PlotDatasetPositionVelocity(objDataset);

        end

        function [objSceneFig] = VisualizeTrajectory3dSceneAndPosesStatic(objDataset)
            arguments
                objDataset (1,1) {mustBeA(objDataset, "SReferenceMissionDesign")}
            end
            % Plot 3D trajectory in target fixed frame
            [objSceneFig] = VisualizeTrajectory3dSceneAndPoses(objDataset);

        end


    end

end
