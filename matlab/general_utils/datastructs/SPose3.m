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

