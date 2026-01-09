classdef SNavState < SPose3 
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 19-02-2025    Pietro Califano     Implemented to store data on navigation state  
    % 11-09-2025    Pietro Califano     Update implementation for new nav-system version
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

    properties ( Access = public)
        % enumFrame TBD, may be too heavy to carry along
        dVelocity_Frame
        dTimestamp
        dStateCovariance % Order is [r, v, attitude]
        bTriangularCov = false

        bRequireTimeUpdate = false;


        % Storage for GTSAM objects
        % TODO move to subclass!
        objMaxAPostPose3_NavFrame   {mustBeA(objMaxAPostPose3_NavFrame, ["gtsam.Pose3", "gtsam.Point3", "double"])} = [];
        objRot3_NavFromPose         {mustBeA(objRot3_NavFromPose, ["gtsam.Rot3", "double"])} = [];
        dMaxAPostVelocity_NavFrame  {mustBeA(dMaxAPostVelocity_NavFrame, ["gtsam.Point3", "double"])} = [];

        dJointMarginalPosVel_PoseFrame  (6,6) {ismatrix, mustBeNumeric} = zeros(6,6)
        dMarginalPoseCov_PoseFrame      (6,6) {ismatrix, mustBeNumeric} = zeros(6,6);
        dMarginalVelCov_NavFrame        (3,3) {ismatrix, mustBeNumeric} = zeros(3,3);
        dMarginalPoseCov_NavFrame       (6,6) {ismatrix, mustBeNumeric} = zeros(6,6);
    end
    
    methods (Access = public)
        function self = SNavState(dTimestamp, dPosition_Frame, dVelocity_Frame, dDCM_FrameFromPoseFrame, dStateCovariance)
            arguments
                dTimestamp                  (1,1) double {isscalar, mustBeNumeric} = 0.0
                dPosition_Frame             (3,1) double {isvector, mustBeNumeric} = zeros(3,1);
                dVelocity_Frame             (3,1) double {isvector, mustBeNumeric} = zeros(3,1);
                dDCM_FrameFromPoseFrame     (3,3) double {ismatrix, mustBeNumeric} = eye(3); 
                dStateCovariance            (9,9) double {ismatrix, mustBeNumeric} = zeros(9,9);
            end
            
            % Store data
            self = self@SPose3(dPosition_Frame, dDCM_FrameFromPoseFrame);
            self.dVelocity_Frame = dVelocity_Frame;
            self.dTimestamp = dTimestamp;
            self.dStateCovariance = dStateCovariance;
             
            self.bTriangularCov = all(abs(dStateCovariance) > eps, 'all') && ...
                (istriu(dStateCovariance) || istril(dStateCovariance));

            if nargin > 0
                self.bDefaultConstructed = false;
            end
        end
        

        function self = changeReferenceFrame(self, dDCM_NewFrameFromFrame)
            arguments
                self
                dDCM_NewFrameFromFrame (3,3) {ismatrix, mustBeNumeric}
            end

            self.dVelocity_Frame         = dDCM_NewFrameFromFrame * self.dVelocity_Frame;
            self = changeReferenceFrame@SPose3(self, dDCM_NewFrameFromFrame);

            % TODO add code for covariance!
        end

        % SETTERS
        function self = setCovariance(self, dSubStateCovariance, ui32IdVecStates)
            arguments
                self
                dSubStateCovariance (:,:) double
                ui32IdVecStates     (1,:) uint32 = [1,2,3]; % Must be 1 to 36
            end
            % TODO, should set entire matrix, or subblocks according to which inputs are given
            % Get entries of ui32IdVecStates and construct allocation ptrs
            assert(all(ui32IdVecStates > 0) && all(ui32IdVecStates <= 3), 'Invalid input indices. Must be any combination of 1: Position, 2: Velocity, 3: Attitude.');
            ui32AllocPtrs = self.determineStatesPtrs(ui32IdVecStates);

        end

        % GETTERS
        % Just methods for syntactic sugar
        function dVelocity_Frame = velocity(self)
            dVelocity_Frame = self.dVelocity_Frame;
        end

        function composeRightSide(self, objOtherNavState)
            arguments
                self
                objOtherNavState {mustBeA(objOtherNavState, "SNavState")}
            end

            % TODO
            error('Not implemented yet')

        end
    
        function dPosVel = getPosVelState(self)
            dPosVel = [self.translation; self.velocity];
        end

        function dSubStateCovariance = getCovariance(self, ui32IdVecStates)
            arguments
                self
                ui32IdVecStates     (1,:) uint32 = [1,2,3]; % Must be 1 to 36
            end
            % TODO, should set entire matrix, or subblocks according to which inputs are given
            % Get entries of ui32IdVecStates and construct allocation ptrs
            assert(all(ui32IdVecStates > 0) && all(ui32IdVecStates <= 3), 'Invalid input indices. Must be any combination of 1: Position, 2: Velocity, 3: Attitude.');
            ui32AllocPtrs = self.determineStatesPtrs(ui32IdVecStates);
            
            % Index data
            dSubStateCovariance = zeros( size(3 * length(ui32IdVecStates) ));

            for idPtr = 1:(length(ui32IdVecStates))
    
                if idPtr > length(ui32IdVecStates)

                    % Off-diagonal terms
                    dSubStateCovariance(ui32AllocPtrs(1,:), ui32AllocPtrs(1,:)) = self.dStateCovariance(ui32AllocPtrs(1,:), ui32AllocPtrs(1,:));
                
                else
                    % Main diagonal subblocks
                    dSubStateCovariance(ui32AllocPtrs(idPtr,:), ui32AllocPtrs(idPtr,:)) = self.dStateCovariance(ui32AllocPtrs(idPtr,:), ui32AllocPtrs(idPtr,:));
                end
            end
        end
    end

    methods (Access = protected)
        % TODO complete
        function [ui32MainAllocPtrs, ui32OffDiagAllocPtr] = determineStatesPtrs(~, ui32IdVecStates)

            error('BUG: Method to fix')
            % Logic: multiply by 3 the index, and go back of -3, then expand
            ui32NumOfIndices = length(ui32IdVecStates);
            ui32NumOfOffDiagIndices = ui32NumOfIndices * (ui32NumOfIndices - 1) / 2; % TODO combinations!

            ui32MainAllocPtrs = zeros(ui32NumOfIndices, 3);
            ui32OffDiagAllocPtr = zeros(ui32NumOfOffDiagIndices, 6);

            % Build main diagonal allocation indices
            for idAPtr = 1:ui32NumOfIndices
                ui32TmpLastEntry = 3*ui32IdVecStates(idAPtr);
                ui32MainAllocPtrs(idAPtr, :) = ui32TmpLastEntry-3:1:ui32TmpLastEntry;
            end

            % Logic: If index is just one, nothing to do. Else build combinations
            if ui32NumOfOffDiagIndices > 0

                % Build combinations for correlation terms
                for idAPtr = 1:ui32NumOfOffDiagIndices
                    % TODO not completed
                    ui32TmpLastEntry = 3*ui32IdVecStates(idAPtr);
                    ui32MainAllocPtrs(ui32NumOfIndices + idAPtr, :) = ui32TmpLastEntry-3:1:ui32TmpLastEntry;

                end

            end

        end
    end


end
