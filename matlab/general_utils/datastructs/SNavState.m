classdef SNavState < SPose3
    %% DESCRIPTION
    % Timestamped navigation-state mean containing position, velocity, and
    % attitude in one reference frame. Uncertainty and backend-native graph
    % objects are carried by dedicated estimate classes.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 19-02-2025    Pietro Califano     Implemented to store data on navigation state.
    % 11-09-2025    Pietro Califano     Update implementation for new nav-system version.
    % 11-08-2026    Pietro Califano, Codex     Restrict the class to the kinematic state mean and timestamp.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % SNavState: Construct a timestamped kinematic state.
    % changeReferenceFrame: Rotate position, velocity, and attitude together.
    % composeRightSide: Return the same-epoch relative kinematic state.
    % velocity: Return velocity in the current frame.
    % getPosVelState: Return the stacked position/velocity mean.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % dVelocity_Frame: (3,1) double velocity in the current reference frame.
    % dTimestamp: (1,1) double state epoch in seconds.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % SPose3.
    % -------------------------------------------------------------------------------------------------------------

    properties (Access = public)
        dVelocity_Frame (3,1) double = zeros(3,1)
        dTimestamp (1,1) double = 0.0
    end

    methods (Access = public)
        function self = SNavState(dTimestamp, dPosition_Frame, dVelocity_Frame, dDCM_FrameFromPoseFrame)
            %% SIGNATURE
            % self = SNavState(dTimestamp, dPosition_Frame, dVelocity_Frame, dDCM_FrameFromPoseFrame)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Construct one navigation-state mean at a single timestamp.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % dTimestamp                  (1,1) double state epoch in seconds.
            % dPosition_Frame             (3,1) double position in the current frame.
            % dVelocity_Frame             (3,1) double velocity in the current frame.
            % dDCM_FrameFromPoseFrame     (3,3) double attitude direction-cosine matrix.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self                        Constructed navigation-state value.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-02-2025    Pietro Califano     First implementation.
            % 11-08-2026    Pietro Califano, Codex     Remove uncertainty and graph-backend payloads.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SPose3.
            % -----------------------------------------------------------------------------------------------------
            arguments
                dTimestamp (1,1) double {mustBeNumeric} = 0.0
                dPosition_Frame (3,1) double {mustBeNumeric} = zeros(3,1)
                dVelocity_Frame (3,1) double {mustBeNumeric} = zeros(3,1)
                dDCM_FrameFromPoseFrame (3,3) double {mustBeNumeric} = eye(3)
            end

            self = self@SPose3(dPosition_Frame, dDCM_FrameFromPoseFrame);
            self.dVelocity_Frame = dVelocity_Frame;
            self.dTimestamp = dTimestamp;
            if nargin > 0
                self.bDefaultConstructed = false;
            end
        end

        function self = changeReferenceFrame(self, dDCM_NewFrameFromFrame)
            %% SIGNATURE
            % self = self.changeReferenceFrame(dDCM_NewFrameFromFrame)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Rotate the kinematic mean into a new reference frame without changing its epoch.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                       Navigation-state value.
            % dDCM_NewFrameFromFrame     (3,3) double old-to-new frame rotation.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self                       Navigation state in the new frame.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-02-2025    Pietro Califano     First implementation.
            % 11-08-2026    Pietro Califano, Codex     Document state-only frame transformation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SPose3.changeReferenceFrame.
            % -----------------------------------------------------------------------------------------------------
            arguments
                self (1,1) SNavState
                dDCM_NewFrameFromFrame (3,3) double {mustBeNumeric}
            end

            self.dVelocity_Frame = dDCM_NewFrameFromFrame * self.dVelocity_Frame;
            self = changeReferenceFrame@SPose3(self, dDCM_NewFrameFromFrame);
        end

        function dVelocity_Frame = velocity(self)
            %% SIGNATURE
            % dVelocity_Frame = self.velocity()
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return velocity expressed in the current reference frame.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                  Navigation-state value.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dVelocity_Frame       (3,1) double velocity in the current frame.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-02-2025    Pietro Califano     First implementation.
            % 11-08-2026    Pietro Califano, Codex     Complete public API documentation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) SNavState
            end
            arguments (Output)
                dVelocity_Frame (3,1) double
            end
            dVelocity_Frame = self.dVelocity_Frame;
        end

        function objRelativeNavState = composeRightSide(self, objOtherNavState)
            %% SIGNATURE
            % objRelativeNavState = self.composeRightSide(objOtherNavState)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return the relative kinematic state of self with respect to a
            % same-epoch reference state, following SPose3 composition semantics.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                  Navigation state being expressed relatively.
            % objOtherNavState      Same-epoch reference navigation state.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objRelativeNavState   Relative position, velocity, and attitude at self's epoch.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-02-2025    Pietro Califano     First declaration.
            % 11-08-2026    Pietro Califano, Codex     Implement same-epoch relative-state composition.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SPose3.composeRightSide.
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) SNavState
                objOtherNavState (1,1) SNavState
            end
            arguments (Output)
                objRelativeNavState (1,1) SNavState
            end

            % Relative-state composition is defined only for estimates of the
            % same epoch; it must never act as implicit state propagation.
            dTimestampTolerance = 1.0e-6;
            dTimestampDifference = self.dTimestamp - objOtherNavState.dTimestamp;
            if ~isfinite(dTimestampDifference) || abs(dTimestampDifference) > dTimestampTolerance
                error("SNavState:CompositionTimestampMismatch", ...
                    "Navigation states must share a timestamp within 1.0e-6 seconds.");
            end

            % Reuse the base relative-pose convention and extend it only with
            % velocity differencing while preserving the left state epoch.
            objRelativePose = composeRightSide@SPose3(self, objOtherNavState);
            dRelativeVelocity_Frame = self.dVelocity_Frame - objOtherNavState.dVelocity_Frame;
            objRelativeNavState = SNavState(self.dTimestamp, objRelativePose.dPosition_Frame, ...
                dRelativeVelocity_Frame, objRelativePose.dDCM_FrameFromPoseFrame);
        end

        function dPosVel = getPosVelState(self)
            %% SIGNATURE
            % dPosVel = self.getPosVelState()
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return the stacked [position; velocity] mean.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                  Navigation-state value.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dPosVel               (6,1) double stacked position and velocity.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-02-2025    Pietro Califano     First implementation.
            % 11-08-2026    Pietro Califano, Codex     Complete public API documentation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SPose3.translation, SNavState.velocity.
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) SNavState
            end
            arguments (Output)
                dPosVel (6,1) double
            end
            dPosVel = [self.translation; self.velocity];
        end
    end
end
