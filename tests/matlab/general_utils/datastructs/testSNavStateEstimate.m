classdef testSNavStateEstimate < matlab.unittest.TestCase
    %% DESCRIPTION
    % Behavioral contract tests for SNavStateEstimate.
    % The suite verifies explicit uncertainty metadata, supported state-error
    % layouts, value semantics, stable uncertainty conversion, validation,
    % reference-frame transformation, and delayed-epoch association.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-08-2026  Pietro Califano, Codex     First navigation-state estimate contract tests.
    % 11-08-2026  Pietro Califano, Codex     Cover trusted and strict construction policies.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % SNavStateEstimate, SNavState, uncertainty/layout/convention enums.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant, Access = private)
        dConversionAbsTol = 5.0e-12
        dConversionRelTol = 5.0e-12
        dIllConditionedAbsTol = 5.0e-8
        dIllConditionedRelTol = 5.0e-8
        dFrameAbsTol = 5.0e-11
        dFrameRelTol = 5.0e-11
    end

    methods (TestClassSetup)
        function addSimulationGearsPaths(~)
            charTestDirectory = fileparts(mfilename("fullpath"));
            charRepositoryRoot = fileparts(fileparts(fileparts(fileparts(charTestDirectory))));
            run(fullfile(charRepositoryRoot, "matlab", "SetupSimGears.m"));
        end
    end

    methods (Test)
        function testConstructsPositionVelocityEstimate(self)
            % A six-state estimate must retain its state, covariance, and
            % explicit no-attitude layout metadata without normalization.
            objNavState = CreateNavState_();
            dCovariance = diag([4.0, 9.0, 16.0, 0.04, 0.09, 0.16]);

            objEstimate = SNavStateEstimate(objNavState, dCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

            self.verifyEqual(objEstimate.objNavState, objNavState);
            self.verifyEqual(objEstimate.dUncertaintyMatrix, dCovariance);
            self.verifyEqual(objEstimate.enumStateLayout, EnumNavStateUncertaintyLayout.POSITION_VELOCITY);
            self.verifyEqual(objEstimate.enumAttitudeErrorConvention, EnumAttitudeErrorConvention.NONE);
        end

        function testConstructsAttitudeErrorEstimate(self)
            % A nine-state estimate must retain the declared attitude-error
            % convention together with the position/velocity/attitude layout.
            objNavState = CreateNavState_();
            dCovariance = diag([4.0, 9.0, 16.0, 0.04, 0.09, 0.16, ...
                1.0e-4, 2.0e-4, 3.0e-4]);

            objEstimate = SNavStateEstimate(objNavState, dCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.LEFT_FRAME);

            self.verifyEqual(objEstimate.objNavState, objNavState);
            self.verifyEqual(objEstimate.dUncertaintyMatrix, dCovariance);
            self.verifyEqual(objEstimate.enumStateLayout, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR);
            self.verifyEqual(objEstimate.enumAttitudeErrorConvention, EnumAttitudeErrorConvention.LEFT_FRAME);
        end

        function testRepresentationIsNotInferredFromTriangularContents(self)
            % Identical triangular values can represent covariance or
            % information roots; only the enum distinguishes the contracts.
            objNavState = CreateNavState_();
            dUpperFactor = diag([2.0, 3.0, 4.0, 0.2, 0.3, 0.4]);

            objCovarianceRootEstimate = SNavStateEstimate(objNavState, dUpperFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);
            objInformationRootEstimate = SNavStateEstimate(objNavState, dUpperFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

            self.verifyEqual(objCovarianceRootEstimate.dUncertaintyMatrix, objInformationRootEstimate.dUncertaintyMatrix);
            self.verifyNotEqual(objCovarianceRootEstimate.enumUncertaintyRepresentation, ...
                objInformationRootEstimate.enumUncertaintyRepresentation);
        end

        function testAttitudeConventionIsNotInferredFromMatrixContents(self)
            % The same nine-state covariance supports distinct left-frame and
            % right-pose perturbation contracts selected only by metadata.
            objNavState = CreateNavState_();
            dCovariance = diag(1.0:9.0);

            objLeftEstimate = SNavStateEstimate(objNavState, dCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.LEFT_FRAME);
            objRightEstimate = SNavStateEstimate(objNavState, dCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.RIGHT_POSE);

            self.verifyEqual(objLeftEstimate.dUncertaintyMatrix, objRightEstimate.dUncertaintyMatrix);
            self.verifyNotEqual(objLeftEstimate.enumAttitudeErrorConvention, ...
                objRightEstimate.enumAttitudeErrorConvention);
        end

        function testPositionVelocityRejectsAttitudeErrorConvention(self)
            % A six-state uncertainty has no attitude-error block, so any
            % attitude convention other than NONE is a contract violation.
            objNavState = CreateNavState_();

            self.verifyError(@() SNavStateEstimate(objNavState, eye(6), ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.LEFT_FRAME), ...
                "SNavStateEstimate:InvalidAttitudeErrorConvention");
        end

        function testAttitudeErrorLayoutRejectsNoneConvention(self)
            % A nine-state uncertainty cannot leave the attitude-error
            % perturbation side/frame unspecified.
            objNavState = CreateNavState_();

            self.verifyError(@() SNavStateEstimate(objNavState, eye(9), ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.NONE), ...
                "SNavStateEstimate:InvalidAttitudeErrorConvention");
        end

        function testRejectsLayoutDimensionMismatch(self)
            % Matrix dimensions must follow declared layout metadata rather
            % than being accepted as an implicit alternative layout.
            objNavState = CreateNavState_();

            self.verifyError(@() SNavStateEstimate(objNavState, eye(9), ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE), ...
                "SNavStateEstimate:InvalidMatrixSize");
            self.verifyError(@() SNavStateEstimate(objNavState, eye(6), ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.LEFT_FRAME), ...
                "SNavStateEstimate:InvalidMatrixSize");
        end

        function testDefaultConstructionSkipsStrictNumericalValidation(self)
            % Trusted hot paths retain structurally valid matrices without
            % paying for repeated finiteness, symmetry, or Cholesky checks.
            objNavState = CreateNavState_();
            dUncheckedCovariance = diag([1.0, 2.0, 3.0, 4.0, 5.0, 0.0]);

            objEstimate = SNavStateEstimate(objNavState, dUncheckedCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

            self.verifyEqual(objEstimate.dUncertaintyMatrix, dUncheckedCovariance, "AbsTol", 0.0);
            self.verifyEqual(objEstimate.enumUncertaintyRepresentation, ...
                EnumStateUncertaintyRepresentation.COVARIANCE);
        end

        function testRejectsNonFiniteUncertainty(self)
            % Non-finite values are invalid before representation-specific
            % symmetry, definiteness, or triangularity checks run.
            objNavState = CreateNavState_();
            dNonFiniteCovariance = eye(6);
            dNonFiniteCovariance(2, 2) = NaN;
            dNonFiniteFactor = eye(6);
            dNonFiniteFactor(4, 4) = Inf;

            self.verifyError(@() SNavStateEstimate(objNavState, dNonFiniteCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                "SNavStateEstimate:NonFiniteUncertaintyMatrix");
            self.verifyError(@() SNavStateEstimate(objNavState, dNonFiniteFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                "SNavStateEstimate:NonFiniteUncertaintyMatrix");
        end

        function testRejectsAsymmetricCovarianceAndInformation(self)
            % Both direct matrix representations require symmetry before
            % positive-definiteness is meaningful.
            objNavState = CreateNavState_();
            dAsymmetricMatrix = eye(6);
            dAsymmetricMatrix(1, 2) = 0.25;
            enumDirectRepresentations = [ ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumStateUncertaintyRepresentation.INFORMATION];

            for ui32RepresentationIndex = uint32(1):uint32(numel(enumDirectRepresentations))
                self.verifyError(@() SNavStateEstimate(objNavState, dAsymmetricMatrix, ...
                    enumDirectRepresentations(double(ui32RepresentationIndex)), ...
                    EnumNavStateUncertaintyLayout.POSITION_VELOCITY, ...
                    EnumAttitudeErrorConvention.NONE, true), ...
                    "SNavStateEstimate:AsymmetricUncertaintyMatrix");
            end
        end

        function testRejectsSingularAndIndefiniteDirectMatrices(self)
            % Cholesky validation must reject both a zero eigenvalue and a
            % negative eigenvalue for covariance and information matrices.
            objNavState = CreateNavState_();
            dSingularMatrix = diag([1.0, 2.0, 3.0, 4.0, 5.0, 0.0]);
            dIndefiniteMatrix = diag([1.0, 2.0, 3.0, 4.0, 5.0, -1.0]);
            enumDirectRepresentations = [ ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumStateUncertaintyRepresentation.INFORMATION];

            for ui32RepresentationIndex = uint32(1):uint32(numel(enumDirectRepresentations))
                enumRepresentation = enumDirectRepresentations(double(ui32RepresentationIndex));
                self.verifyError(@() SNavStateEstimate(objNavState, dSingularMatrix, enumRepresentation, ...
                    EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                    "SNavStateEstimate:UncertaintyMatrixNotPositiveDefinite");
                self.verifyError(@() SNavStateEstimate(objNavState, dIndefiniteMatrix, enumRepresentation, ...
                    EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                    "SNavStateEstimate:UncertaintyMatrixNotPositiveDefinite");
            end
        end

        function testRejectsIncorrectFactorOrientation(self)
            % An upper declaration cannot accept a lower factor, and the
            % corresponding lower declaration cannot accept an upper factor.
            objNavState = CreateNavState_();
            dLowerFactor = eye(6);
            dLowerFactor(2, 1) = 0.2;
            dUpperFactor = eye(6);
            dUpperFactor(1, 2) = 0.2;

            self.verifyError(@() SNavStateEstimate(objNavState, dLowerFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                "SNavStateEstimate:UncertaintyFactorNotUpperTriangular");
            self.verifyError(@() SNavStateEstimate(objNavState, dUpperFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_INFORMATION_LOWER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                "SNavStateEstimate:UncertaintyFactorNotLowerTriangular");
        end

        function testRejectsNonpositiveFactorDiagonal(self)
            % A triangular factor with a zero or negative diagonal is
            % singular or violates the canonical Cholesky sign convention.
            objNavState = CreateNavState_();
            dZeroDiagonalFactor = eye(6);
            dZeroDiagonalFactor(3, 3) = 0.0;
            dNegativeDiagonalFactor = eye(6);
            dNegativeDiagonalFactor(5, 5) = -1.0;

            self.verifyError(@() SNavStateEstimate(objNavState, dZeroDiagonalFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                "SNavStateEstimate:UncertaintyFactorNonPositiveDiagonal");
            self.verifyError(@() SNavStateEstimate(objNavState, dNegativeDiagonalFactor, ...
                EnumStateUncertaintyRepresentation.SQRT_INFORMATION_LOWER, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE, true), ...
                "SNavStateEstimate:UncertaintyFactorNonPositiveDiagonal");
        end

        function testEstimateAndComposedStateUseValueSemantics(self)
            % Copying the estimate or mutating the constructor input must not
            % alias the stored estimate value.
            objInputNavState = CreateNavState_();
            dOriginalPosition = objInputNavState.dPosition_Frame;
            objEstimate = SNavStateEstimate(objInputNavState, eye(6), ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

            objEstimateCopy = objEstimate;
            objEstimateCopy.bDefaultConstructed = true;
            objInputNavState.dPosition_Frame = 2.0 * objInputNavState.dPosition_Frame;
            objInputNavState.dTimestamp = -1.0;

            self.verifyFalse(objEstimate.bDefaultConstructed);
            self.verifyTrue(objEstimateCopy.bDefaultConstructed);
            self.verifyEqual(objEstimate.objNavState.dPosition_Frame, dOriginalPosition);
            self.verifyEqual(objEstimate.objNavState.dTimestamp, 42.5);
        end

        function testAllRepresentationsConvertToSameCovarianceAndInformation(self)
            % Every representation must implement its declared factor
            % orientation and converge on the same covariance/information pair.
            objNavState = CreateNavState_();
            [dExpectedCovariance, dExpectedInformation, enumRepresentations, cellStoredMatrices] = ...
                CreateSixStateUncertaintyFixture_();

            for ui32RepresentationIndex = uint32(1):uint32(numel(enumRepresentations))
                enumRepresentation = enumRepresentations(double(ui32RepresentationIndex));
                dStoredMatrix = cellStoredMatrices{double(ui32RepresentationIndex)};
                objEstimate = SNavStateEstimate(objNavState, dStoredMatrix, enumRepresentation, ...
                    EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

                dActualCovariance = objEstimate.getCovariance();
                dActualInformation = objEstimate.getInformation();

                self.verifyEqual(dActualCovariance, dExpectedCovariance, ...
                    "AbsTol", self.dConversionAbsTol, "RelTol", self.dConversionRelTol);
                self.verifyEqual(dActualInformation, dExpectedInformation, ...
                    "AbsTol", self.dConversionAbsTol, "RelTol", self.dConversionRelTol);
                self.verifyEqual(objEstimate.dUncertaintyMatrix, dStoredMatrix, "AbsTol", 0.0);
                self.verifyEqual(objEstimate.enumUncertaintyRepresentation, enumRepresentation);
            end
        end

        function testIllConditionedSpdConversionsRemainStable(self)
            % A Hilbert covariance exercises stable Cholesky/triangular solves
            % without accepting singular or indefinite inputs.
            objNavState = CreateNavState_();
            dExpectedCovariance = hilb(6);
            dExpectedInformation = dExpectedCovariance \ eye(6);
            dExpectedInformation = 0.5 * (dExpectedInformation + dExpectedInformation');
            objCovarianceEstimate = SNavStateEstimate(objNavState, dExpectedCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);
            objInformationEstimate = SNavStateEstimate(objNavState, dExpectedInformation, ...
                EnumStateUncertaintyRepresentation.INFORMATION, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

            self.verifyGreaterThan(cond(dExpectedCovariance), 1.0e6);
            self.verifyEqual(objCovarianceEstimate.getInformation(), dExpectedInformation, ...
                "AbsTol", self.dIllConditionedAbsTol, "RelTol", self.dIllConditionedRelTol);
            self.verifyEqual(objInformationEstimate.getCovariance(), dExpectedCovariance, ...
                "AbsTol", self.dIllConditionedAbsTol, "RelTol", self.dIllConditionedRelTol);
        end

        function testFrameChangePreservesEveryRepresentationAndRoundTrips(self)
            % Six-state frame changes must transform nonzero cross-covariance,
            % preserve storage metadata, and recover the source through R'.
            objNavState = CreateNavState_();
            [dOriginalCovariance, dOriginalInformation, enumRepresentations, cellOriginalMatrices] = ...
                CreateSixStateUncertaintyFixture_();
            dDCM_NewFrameFromFrame = CreateFrameRotation_();
            dJacobian = blkdiag(dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame);
            dExpectedCovariance = dJacobian * dOriginalCovariance * dJacobian';
            dExpectedCovariance = 0.5 * (dExpectedCovariance + dExpectedCovariance');
            dExpectedInformation = dJacobian' \ (dOriginalInformation / dJacobian);
            dExpectedInformation = 0.5 * (dExpectedInformation + dExpectedInformation');
            cellExpectedMatrices = {dExpectedCovariance, chol(dExpectedCovariance), ...
                chol(dExpectedCovariance, "lower"), dExpectedInformation, ...
                chol(dExpectedInformation), chol(dExpectedInformation, "lower")};

            for ui32RepresentationIndex = uint32(1):uint32(numel(enumRepresentations))
                enumRepresentation = enumRepresentations(double(ui32RepresentationIndex));
                dOriginalMatrix = cellOriginalMatrices{double(ui32RepresentationIndex)};
                dExpectedMatrix = cellExpectedMatrices{double(ui32RepresentationIndex)};
                objEstimate = SNavStateEstimate(objNavState, dOriginalMatrix, enumRepresentation, ...
                    EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

                objTransformedEstimate = objEstimate.changeReferenceFrame(dDCM_NewFrameFromFrame);

                self.verifyEqual(objTransformedEstimate.objNavState.dPosition_Frame, ...
                    dDCM_NewFrameFromFrame * objNavState.dPosition_Frame, "AbsTol", self.dFrameAbsTol);
                self.verifyEqual(objTransformedEstimate.objNavState.dVelocity_Frame, ...
                    dDCM_NewFrameFromFrame * objNavState.dVelocity_Frame, "AbsTol", self.dFrameAbsTol);
                self.verifyEqual(objTransformedEstimate.objNavState.dDCM_FrameFromPoseFrame, ...
                    dDCM_NewFrameFromFrame * objNavState.dDCM_FrameFromPoseFrame, "AbsTol", self.dFrameAbsTol);
                self.verifyEqual(objTransformedEstimate.objNavState.dTimestamp, objNavState.dTimestamp);
                self.verifyEqual(objTransformedEstimate.dUncertaintyMatrix, dExpectedMatrix, ...
                    "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
                self.verifyEqual(objTransformedEstimate.enumUncertaintyRepresentation, enumRepresentation);
                self.verifyEqual(objEstimate.dUncertaintyMatrix, dOriginalMatrix, "AbsTol", 0.0);

                objRoundTripEstimate = objTransformedEstimate.changeReferenceFrame(dDCM_NewFrameFromFrame');
                self.verifyEqual(objRoundTripEstimate.objNavState.dPosition_Frame, objNavState.dPosition_Frame, ...
                    "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
                self.verifyEqual(objRoundTripEstimate.objNavState.dVelocity_Frame, objNavState.dVelocity_Frame, ...
                    "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
                self.verifyEqual(objRoundTripEstimate.objNavState.dDCM_FrameFromPoseFrame, ...
                    objNavState.dDCM_FrameFromPoseFrame, "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
                self.verifyEqual(objRoundTripEstimate.dUncertaintyMatrix, dOriginalMatrix, ...
                    "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
            end
        end

        function testAttitudeConventionSelectsFrameJacobian(self)
            % Cross-covariance with attitude exposes the distinct LEFT_FRAME
            % and RIGHT_POSE third Jacobian blocks under a nonidentity rotation.
            objNavState = CreateNavState_();
            dLowerFactor = diag([2.0, 1.8, 1.6, 0.9, 0.8, 0.7, 0.3, 0.25, 0.2]);
            dLowerFactor(4, 1) = 0.2;
            dLowerFactor(6, 3) = -0.15;
            dLowerFactor(7, 1) = 0.12;
            dLowerFactor(8, 5) = -0.08;
            dLowerFactor(9, 2) = 0.09;
            dLowerFactor(9, 6) = 0.06;
            dOriginalCovariance = dLowerFactor * dLowerFactor';
            dDCM_NewFrameFromFrame = CreateFrameRotation_();
            dLeftJacobian = blkdiag(dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame);
            dRightJacobian = blkdiag(dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame, eye(3));
            dExpectedLeftCovariance = dLeftJacobian * dOriginalCovariance * dLeftJacobian';
            dExpectedRightCovariance = dRightJacobian * dOriginalCovariance * dRightJacobian';
            objLeftEstimate = SNavStateEstimate(objNavState, dOriginalCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.LEFT_FRAME);
            objRightEstimate = SNavStateEstimate(objNavState, dOriginalCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY_ATTITUDE_ERROR, ...
                EnumAttitudeErrorConvention.RIGHT_POSE);

            objTransformedLeft = objLeftEstimate.changeReferenceFrame(dDCM_NewFrameFromFrame);
            objTransformedRight = objRightEstimate.changeReferenceFrame(dDCM_NewFrameFromFrame);

            self.verifyGreaterThan(norm(dOriginalCovariance(1:6, 7:9), "fro"), 0.0);
            self.verifyEqual(objTransformedLeft.getCovariance(), dExpectedLeftCovariance, ...
                "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
            self.verifyEqual(objTransformedRight.getCovariance(), dExpectedRightCovariance, ...
                "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
            self.verifyGreaterThan(norm(objTransformedLeft.getCovariance() - ...
                objTransformedRight.getCovariance(), "fro"), 1.0e-3);
            self.verifyEqual(objTransformedLeft.objNavState, objTransformedRight.objNavState);

            objLeftRoundTrip = objTransformedLeft.changeReferenceFrame(dDCM_NewFrameFromFrame');
            objRightRoundTrip = objTransformedRight.changeReferenceFrame(dDCM_NewFrameFromFrame');
            self.verifyEqual(objLeftRoundTrip.getCovariance(), dOriginalCovariance, ...
                "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
            self.verifyEqual(objRightRoundTrip.getCovariance(), dOriginalCovariance, ...
                "AbsTol", self.dFrameAbsTol, "RelTol", self.dFrameRelTol);
        end

        function testDelayedEpochRemainsAssociatedWithUncertainty(self)
            % Conversion and frame change may reinterpret or rotate an estimate,
            % but must not propagate its nonzero-velocity mean to another epoch.
            dDelayedTimestamp = 17.25;
            dDelayedPosition = [8.0; -3.0; 1.5];
            dDelayedVelocity = [0.4; -0.2; 0.1];
            objDelayedNavState = SNavState(dDelayedTimestamp, dDelayedPosition, dDelayedVelocity, eye(3));
            dDelayedCovariance = diag([2.0, 3.0, 4.0, 0.2, 0.3, 0.4]);
            objDelayedEstimate = SNavStateEstimate(objDelayedNavState, dDelayedCovariance, ...
                EnumStateUncertaintyRepresentation.COVARIANCE, ...
                EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);

            objDelayedEstimate.getInformation();
            objDelayedEstimate.getCovariance();
            dDCM_NewFrameFromFrame = CreateFrameRotation_();
            objRotatedEstimate = objDelayedEstimate.changeReferenceFrame(dDCM_NewFrameFromFrame);

            self.verifyEqual(objDelayedEstimate.objNavState.dTimestamp, dDelayedTimestamp);
            self.verifyEqual(objDelayedEstimate.objNavState.dPosition_Frame, dDelayedPosition);
            self.verifyEqual(objDelayedEstimate.objNavState.dVelocity_Frame, dDelayedVelocity);
            self.verifyEqual(objDelayedEstimate.dUncertaintyMatrix, dDelayedCovariance);
            self.verifyEqual(objRotatedEstimate.objNavState.dTimestamp, dDelayedTimestamp);
            self.verifyEqual(objRotatedEstimate.objNavState.dPosition_Frame, ...
                dDCM_NewFrameFromFrame * dDelayedPosition, "AbsTol", self.dFrameAbsTol);
            self.verifyEqual(objRotatedEstimate.objNavState.dVelocity_Frame, ...
                dDCM_NewFrameFromFrame * dDelayedVelocity, "AbsTol", self.dFrameAbsTol);
        end
    end
end

function objNavState = CreateNavState_()
%% SIGNATURE
% objNavState = CreateNavState_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Create a non-default navigation state shared by estimate contract tests.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% [-]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objNavState    Navigation state with a nonzero timestamp and mean.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-08-2026  Pietro Califano, Codex     First test-state fixture.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SNavState
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    objNavState (1,1) SNavState
end

objNavState = SNavState(42.5, [10.0; -4.0; 2.0], ...
    [0.2; -0.1; 0.05], eye(3));
end

function [dCovariance, dInformation, enumRepresentations, cellStoredMatrices] = CreateSixStateUncertaintyFixture_()
%% SIGNATURE
% [dCovariance, dInformation, enumRepresentations, cellStoredMatrices] = CreateSixStateUncertaintyFixture_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build one non-diagonal SPD uncertainty and all six declared storage forms.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% [-]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dCovariance            Reference covariance.
% dInformation           Reference information matrix.
% enumRepresentations    Representation enum in storage-fixture order.
% cellStoredMatrices     Matrices for all six representations.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-08-2026  Pietro Califano, Codex     First shared uncertainty fixture.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% chol
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    dCovariance (6,6) double
    dInformation (6,6) double
    enumRepresentations (1,6) EnumStateUncertaintyRepresentation
    cellStoredMatrices (1,6) cell
end

dCovarianceUpperFactor = [ ...
    2.0, 0.3, -0.2, 0.1, 0.0, 0.4; 0.0, 1.5, 0.1, -0.3, 0.2, 0.0; ...
    0.0, 0.0, 1.2, 0.0, 0.1, -0.2; 0.0, 0.0, 0.0, 0.8, 0.2, 0.1; ...
    0.0, 0.0, 0.0, 0.0, 0.6, -0.1; 0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
dCovariance = dCovarianceUpperFactor' * dCovarianceUpperFactor;
dInformation = dCovariance \ eye(6);
dInformation = 0.5 * (dInformation + dInformation');
enumRepresentations = [EnumStateUncertaintyRepresentation.COVARIANCE, ...
    EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER, ...
    EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_LOWER, ...
    EnumStateUncertaintyRepresentation.INFORMATION, ...
    EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER, ...
    EnumStateUncertaintyRepresentation.SQRT_INFORMATION_LOWER];
cellStoredMatrices = {dCovariance, dCovarianceUpperFactor, chol(dCovariance, "lower"), ...
    dInformation, chol(dInformation), chol(dInformation, "lower")};
end

function dDCM_NewFrameFromFrame = CreateFrameRotation_()
%% SIGNATURE
% dDCM_NewFrameFromFrame = CreateFrameRotation_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build a fixed nonidentity proper rotation for frame-change tests.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% [-]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDCM_NewFrameFromFrame    Direction-cosine matrix from old to new frame.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-08-2026  Pietro Califano, Codex     First frame-rotation fixture.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    dDCM_NewFrameFromFrame (3,3) double
end

dAngleZ = 0.37;
dAngleX = -0.21;
dRotationZ = [cos(dAngleZ), -sin(dAngleZ), 0.0; sin(dAngleZ), cos(dAngleZ), 0.0; 0.0, 0.0, 1.0];
dRotationX = [1.0, 0.0, 0.0; 0.0, cos(dAngleX), -sin(dAngleX); 0.0, sin(dAngleX), cos(dAngleX)];
dDCM_NewFrameFromFrame = dRotationZ * dRotationX;
end
