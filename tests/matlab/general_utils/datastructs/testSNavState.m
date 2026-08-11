classdef testSNavState < matlab.unittest.TestCase
    %% DESCRIPTION
    % Behavioral contract tests for the uncertainty-free navigation state value.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-08-2026  Pietro Califano, Codex     Cover the pure kinematic-state boundary.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % SNavState.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testStoresOnlyTimestampedKinematicState(self)
            objNavState = SNavState(7.5, [1.0; 2.0; 3.0], [0.1; 0.2; 0.3], eye(3));
            cellLegacyProperties = {"dStateCovariance", "bTriangularCov", "bRequireTimeUpdate", ...
                "objMaxAPostPose3_NavFrame", "objRot3_NavFromPose", "dMaxAPostVelocity_NavFrame", ...
                "dJointMarginalPosVel_PoseFrame", "dMarginalPoseCov_PoseFrame", ...
                "dMarginalVelCov_NavFrame", "dMarginalPoseCov_NavFrame"};

            self.verifyEqual(objNavState.dTimestamp, 7.5, "AbsTol", 0.0);
            self.verifyEqual(objNavState.getPosVelState(), [1.0; 2.0; 3.0; 0.1; 0.2; 0.3], "AbsTol", 0.0);
            for ui32PropertyIdx = 1:numel(cellLegacyProperties)
                self.verifyFalse(isprop(objNavState, cellLegacyProperties{ui32PropertyIdx}));
            end
        end

        function testValueCopyAndFrameChangePreserveEpoch(self)
            dDCM_NewFromOld = [0.0, -1.0, 0.0; 1.0, 0.0, 0.0; 0.0, 0.0, 1.0];
            objOriginalState = SNavState(12.0, [2.0; -1.0; 4.0], [0.5; -0.2; 0.1], eye(3));
            objChangedState = objOriginalState.changeReferenceFrame(dDCM_NewFromOld);

            self.verifyEqual(objOriginalState.dPosition_Frame, [2.0; -1.0; 4.0], "AbsTol", 0.0);
            self.verifyEqual(objOriginalState.dVelocity_Frame, [0.5; -0.2; 0.1], "AbsTol", 0.0);
            self.verifyEqual(objOriginalState.dDCM_FrameFromPoseFrame, eye(3), "AbsTol", 0.0);
            self.verifyEqual(objChangedState.dPosition_Frame, dDCM_NewFromOld * [2.0; -1.0; 4.0], "AbsTol", 0.0);
            self.verifyEqual(objChangedState.dVelocity_Frame, dDCM_NewFromOld * [0.5; -0.2; 0.1], "AbsTol", 0.0);
            self.verifyEqual(objChangedState.dDCM_FrameFromPoseFrame, dDCM_NewFromOld, "AbsTol", 0.0);
            self.verifyEqual(objChangedState.dTimestamp, 12.0, "AbsTol", 0.0);
        end

        function testComposeRightSideReturnsSameEpochRelativeState(self)
            % A co-temporal composition must extend the SPose3 relative-pose
            % contract to velocity without mutating either input value.
            dLeftTimestamp = 42.0 + 0.4e-6;
            dRightTimestamp = 42.0;
            dLeftDCM = [0.0, -1.0, 0.0; 1.0, 0.0, 0.0; 0.0, 0.0, 1.0];
            dRightDCM = [0.0, 1.0, 0.0; -1.0, 0.0, 0.0; 0.0, 0.0, 1.0];
            objLeftState = SNavState(dLeftTimestamp, [5.0; -1.0; 3.0], [0.8; -0.4; 0.2], dLeftDCM);
            objRightState = SNavState(dRightTimestamp, [2.0; 4.0; -1.0], [0.1; 0.3; -0.2], dRightDCM);

            objRelativeState = objLeftState.composeRightSide(objRightState);

            self.verifyClass(objRelativeState, "SNavState");
            self.verifyEqual(objRelativeState.dTimestamp, dLeftTimestamp, "AbsTol", 0.0);
            self.verifyEqual(objRelativeState.dPosition_Frame, [3.0; -5.0; 4.0], "AbsTol", 0.0);
            self.verifyEqual(objRelativeState.dVelocity_Frame, [0.7; -0.7; 0.4], "AbsTol", 10.0 * eps);
            self.verifyEqual(objRelativeState.dDCM_FrameFromPoseFrame, ...
                [-1.0, 0.0, 0.0; 0.0, -1.0, 0.0; 0.0, 0.0, 1.0], "AbsTol", 0.0);
            self.verifyEqual(objLeftState.dPosition_Frame, [5.0; -1.0; 3.0], "AbsTol", 0.0);
            self.verifyEqual(objRightState.dPosition_Frame, [2.0; 4.0; -1.0], "AbsTol", 0.0);
        end

        function testComposeRightSideRejectsTimestampMismatch(self)
            objLeftState = SNavState(42.0, [1.0; 2.0; 3.0], [0.1; 0.2; 0.3], eye(3));
            objRightState = SNavState(42.0 + 2.0e-6, zeros(3,1), zeros(3,1), eye(3));

            self.verifyError(@() objLeftState.composeRightSide(objRightState), ...
                "SNavState:CompositionTimestampMismatch");
        end

        function testRejectsLegacyCovarianceConstructorArgument(self)
            self.verifyError(@() SNavState(1.0, zeros(3,1), zeros(3,1), eye(3), eye(9)), ...
                "MATLAB:TooManyInputs");
        end
    end
end
