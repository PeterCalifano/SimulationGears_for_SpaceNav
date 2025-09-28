classdef SPose3 < CBaseDatastruct
    %% DESCRIPTION
    % Class repredenting a 6D pose object
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 12-02-2025    Pietro Califano     Simple storage class to represent a 6D pose object
    % 14-02-2025    Pietro Califano     Modify to allow default construction
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties ( Access = public)
        % enumFrame TBD, may be too heavy to carry along
        dPosition_Frame
        dDCM_FrameFromPoseFrame
    end
    
    methods (Access = public)
        function self = SPose3(dPosition_Frame, dDCM_FrameFromPoseFrame)
            arguments
                dPosition_Frame             (3,1) double {isvector, isnumeric} = zeros(3,1);
                dDCM_FrameFromPoseFrame     (3,3) double {ismatrix, isnumeric} = eye(3); 
            end

            self.dPosition_Frame            = dPosition_Frame;
            self.dDCM_FrameFromPoseFrame    = dDCM_FrameFromPoseFrame;

            if nargin > 0
                self.bDefaultConstructed = false;
            end
        end

        function self = changeReferenceFrame(self, dDCM_NewFrameFromFrame)
            arguments
                self
                dDCM_NewFrameFromFrame (3,3) {ismatrix, isnumeric}
            end

            self.dPosition_Frame         = dDCM_NewFrameFromFrame * self.dPosition_Frame;
            self.dDCM_FrameFromPoseFrame = dDCM_NewFrameFromFrame * self.dDCM_FrameFromPoseFrame;
        end

        function [objNewPose] = composeRightSide(self, objOtherPose3)
            arguments
                self
                objOtherPose3 {mustBeA(objOtherPose3, "SPose3")}
            end
            
            % Compute new pose3
            dPositionFromOther_Frame        = self.dPosition_Frame - objOtherPose3.dPosition_Frame;
            dDCM_OtherFrameFromPoseFrame    = transpose(objOtherPose3.dDCM_FrameFromPoseFrame) * self.dDCM_FrameFromPoseFrame;

            objNewPose = SPose3(dPositionFromOther_Frame, dDCM_OtherFrameFromPoseFrame);
        end

        % GETTERS
        % Just methods for syntactic sugar
        function dPosition_Frame = translation(self)
            dPosition_Frame = self.dPosition_Frame;
        end

        function dDCM_FrameFromPoseFrame = rotation(self)
            dDCM_FrameFromPoseFrame = self.dDCM_FrameFromPoseFrame;
        end
        
    end
end

