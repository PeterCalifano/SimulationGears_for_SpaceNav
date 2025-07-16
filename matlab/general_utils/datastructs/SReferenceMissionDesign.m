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
    % 15-07-2025    Pietro Califano     Update class with new conveniency methods to produce plots and 
    %                                   attitude data directly from dataset
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
        enumTimeScale                   (1, :) char {ischar, isstring, mustBeMember(enumTimeScale, ...
                                                     ["TAI", "TDB", "TDT", "TT", "ET", "JDTDB", "JDTDT", "JED", "GPS", "N/D"])} = "N/D";
        
        ui32TargetPointingID              (1,:) uint32 {isnumeric} = []
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

        % METHODS
        function [dStateSC_W, self] = dStateSC_W(self, dStateSC_W)
            arguments
                self
                dStateSC_W {ismatrix, isnumeric} = [];
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
                                            dTimeGridTarget, ...
                                            dInterpPosSC_W, ...
                                            dInterpVelSC_W, ...
                                            dInterpSunPosition_W, ...
                                            dInterpEarthPosition_W, ...
                                            dInterpTargetPosition_W, ...
                                            dInterpDCM_TBfromW)
            arguments
                self
                dTimeGridOriginal           double {mustBeVector}
                dTimeGridTarget             double {mustBeVector}
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
                    sprintf('Compare – %s', cellstrVectorNames{ui32VecIdx}), ...
                    'NumberTitle','off');

                if contains(cellstrVectorNames{ui32VecIdx}, 'DCM')
                    % Convert DCM to quaternion sequence
                    dTmpOrig    = DCM2quatSeq(dTmpOrig, false);
                    dTmpInterp  = DCM2quatSeq(dTmpInterp, false);
                end

                ui32NumStates   = size(dTmpOrig,1);

                for ui32StateIdx = uint32(1):ui32NumStates

                    subplot(ui32NumStates, 1, double(ui32StateIdx));
                    plot(dTimeGridTarget,    dTmpInterp(ui32StateIdx,:), ...
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
                dTimeGridTarget = kwargs.dTargetTimegrid;

            elseif not(isempty(kwargs.dDeltaTimeMultiplier))
                % Dataset timegrid definition
                dNewDeltaTime = kwargs.dDeltaTimeMultiplier * (self.dTimestamps(2) - self.dTimestamps(1));
                dTimeGridTarget = objDataset.dTimestamps(1):dNewDeltaTime:objDataset.dTimestamps(end);
            else
                error('Invalid inputs: either dTargetTimegrid or dDeltaTimeMultiplier must be provided.')
            end

            dRelTimeGridTarget = dTimeGridTarget - dTimeGridTarget(1);

            % Interpolate position/velocity data
            dTimeGridOriginal = self.dTimestamps;
                objSplinerHandle = @(dInputData) spline(dTimeGridOriginal, dInputData, dTimeGridTarget);

                dInterpPosSC_W          = objSplinerHandle( self.dPosSC_W );
                dInterpVelSC_W          = objSplinerHandle( self.dVelSC_W );
                dInterpSunPosition_W    = objSplinerHandle( self.dSunPosition_W );
                dInterpEarthPosition_W  = objSplinerHandle( self.dEarthPosition_W );
                dInterpTargetPosition_W = objSplinerHandle( self.dTargetPosition_W );

                % Interpolate target attitude
                dInterpDCM_TBfromW = zeros(3,3, length(dTimeGridTarget));
                ui32InterpGridCheckID_  = zeros(1, length(dTimeGridTarget), 'uint32');

                ui32InterPtr = uint32(1);

                for idQ = 1:length(dTimeGridOriginal)-1

                    % Get idQth interval limits
                    dTime0_ = dTimeGridOriginal(idQ);
                    dTime1_ = dTimeGridOriginal(idQ + 1);

                    % Get all times in the target timegrid within it idQ interval
                    bInitMask = dTimeGridTarget >= dTime0_;
                    bEndMask  = dTimeGridTarget < dTime1_;
                    bExtractionIdx = bInitMask & bEndMask;

                    dInterpTargetTimesInInterval_ = dTimeGridTarget(bExtractionIdx);
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
                    dInterpDCM_TBfromW(1:3, 1:3, ui32InterPtr:ui32InterPtr+ui32NumNewInterpEntries-1) = QuatSeq2DCM(dInterpTargetBodyQuat_TBfromW', false);
                    ui32InterpGridCheckID_(ui32InterPtr:ui32InterPtr+ui32NumNewInterpEntries-1) = ui32TargetIndices;

                    % Update ptr
                    ui32InterPtr = ui32InterPtr + ui32NumNewInterpEntries;
                end


                % TODO interpolate dInterpDCM_SCfromW
                dInterpDCM_SCfromW = [];

                % Add last time instant
                dInterpDCM_TBfromW(:,:,end) = self.dDCM_TBfromW(:,:, end);
                assert(length(unique(ui32InterpGridCheckID_)) == length(dTimeGridTarget), 'ERROR: duplicated indices found in interpolation index. Please check allocation logic!');

                if kwargs.bDisplayComparison
                    % Plot interpolated vs original data
                    self.plotInterpDatasetComparison(dTimeGridOriginal, ...
                                                dTimeGridTarget         , ...
                                                dInterpPosSC_W          , ...
                                                dInterpVelSC_W          , ...
                                                dInterpSunPosition_W    , ...
                                                dInterpEarthPosition_W  , ...
                                                dInterpTargetPosition_W, ...
                                                dInterpDCM_TBfromW);
                end


                % Update fields
                objInterpDataset.dTimestamps       = dTimeGridTarget;
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


        function [objFig_PosVel, objFig_PhaseAngle] = plotDatasetData(self, ui32TargetIndex)
            arguments
                self            (1,1) {mustBeA(self, "SReferenceMissionDesign")}
                ui32TargetIndex (1,1) uint32 {isscalar} = 0
            end
            % DEVNOTE: basic version of method to plot visualizations of data contained in the dataset
            % object. May be expanded with many functionalities and options

            % Plot position and velocity in world frame
            objFig_PosVel = PlotDatasetPositionVelocity(self);

            % Plot phase angle evolution in time
            objFig_PhaseAngle = [];
            if ui32TargetIndex == 0
                dTarget_W = self.dTargetPosition_W;
            else
                try
                    dTarget_W = self.cellAdditionalBodiesPos_W{ui32TargetIndex};
                catch ME
                    warning('Error in getting position of target body ID%d: %s. Are you sure that the dataset contain any additional body?', ...
                        ui32TargetIndex, string(ME.message));
                    return
                end
            end

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
            return


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
        % NONE
    end

    methods (Access = public, Static)

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
                kwargs.placeholder = [];
            end
            
            % TODO add kwargs
            objPointingGenerator = CAttitudePointingGenerator(objDataset.dPosSC_W, ...
                                                                objDataset.dTargetPosition_W, ...
                                                                objDataset.dSunPosition_W, ...
                                                                "bShowAttitudePointingPlot", false);

            dSigmaBoresightRollAngle = 0.0;
            dSigmaOffPointingDegAngle = 0.0;

            [objPointingGenerator, dRot3Param, dCameraAttDCM_WfromOF, dOffPointingAngles] = objPointingGenerator.pointToTarget(...
                                                                                    "dSigmaDegRotAboutBoresight", dSigmaBoresightRollAngle, ...
                                                                                    "enumOffPointingMode", "randomAxis", ...
                                                                                    "dSigmaOffPointingDegAngle", dSigmaOffPointingDegAngle, ...
                                                                                    "enumDisplaceDistribution", "gaussian"); %#ok<*UNRCH>
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
