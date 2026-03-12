classdef CSimulationState < CBaseDatastruct
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 13-02-2025    Pietro Califano     First implementation of class to store and manage simulation state
    %                                   for nav-system unified simulation framework.
    % 21-07-2025    Pietro Califano     Add code to handle multiple bodies and poses
    % 29-12-2025    Pietro Califano     Move from nav-backend to SimulationGears_for_SpaceNav
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    properties (SetAccess = public, GetAccess = public)
        enumWorldFrame
        objCameraPose_W
        objTargetPose_W

        % Camera pose wrt a Target body fixed frame
        objCameraPose_TB

        % Sun position
        dSunPosition_W

        % Timestamps
        dTimestamp         (1,1) double = 0.0
        dRelativeTimestamp (1,1) double = 0.0
        ui32TimestampID = uint32(1);

        % Ground truth index
        i32GraphFeaturesIDs_GT

        % Additional targets and camera poses
        obj3rdTargetPose_W  = SPose3()
        obj3rdCameraPose_TB = SPose3()

    end

    properties (Dependent)
        ui32Num3rdBodies
    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = CSimulationState(objCameraPose_W, objTargetPose_W, objCameraPose_TB)
            arguments
                objCameraPose_W  (1,1) {mustBeA(objCameraPose_W, "SPose3")}  = SNavState()
                objTargetPose_W  (1,1) {mustBeA(objTargetPose_W, "SPose3")}  = SPose3()
                objCameraPose_TB (1,1) {mustBeA(objCameraPose_TB, "SPose3")} = SNavState()
            end

            % Store data
            self.objCameraPose_W = objCameraPose_W;
            self.objTargetPose_W = objTargetPose_W;
            self.objCameraPose_TB = objCameraPose_TB;

            if nargin > 0
                self.bDefaultConstructed = false;
            end

        end

        % GETTERS
        function [dPos_W] = getCamTranslation(self)
            dPos_W = self.objCameraPose_W.translation();
        end

        function [dRot3_W] = getCamRotation(self)
            dRot3_W = self.objCameraPose_W.rotation();
        end

        function [dPos_W] = getTargetTranslation(self)
            dPos_W = self.objTargetPose_W.translation();
        end

        function [dRot3_W] = getTargetRotation(self)
            dRot3_W = self.objTargetPose_W.rotation();
        end


        % SETTERS

        % METHODS
    end


    % DEPENDENT properties getters
    methods
        function [ui32Num3rdBodies] = get.ui32Num3rdBodies(self)
            if self.obj3rdTargetPose_W(1).bDefaultConstructed
                ui32Num3rdBodies = uint32(0);
                return
            end
            ui32Num3rdBodies = length(self.obj3rdTargetPose_W);
        end
    end


    methods (Access=protected)


    end

    % methods (Access=private)
    %
    % end

    % methods (Access=public, Static)
    %
    % end

    % methods (Abstract, Access=public)
    % [x,y] = abstract_function_name(args) NOTE: number of args matter.
    % end
end
