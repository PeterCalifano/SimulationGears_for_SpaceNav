classdef SNavStateEstimate < CBaseDatastruct
    %% DESCRIPTION
    % Value carrier that associates one SNavState with uncertainty at the
    % state's unchanged timestamp. The class owns uncertainty metadata and
    % never propagates, interpolates, or substitutes the composed state.
    %
    % Example:
    %   objNavState = SNavState(12.0, [1; 2; 3], [0.1; 0.2; 0.3], eye(3));
    %   objEstimate = SNavStateEstimate(objNavState, eye(6), ...
    %       EnumStateUncertaintyRepresentation.COVARIANCE, ...
    %       EnumNavStateUncertaintyLayout.POSITION_VELOCITY, EnumAttitudeErrorConvention.NONE);
    %   disp(objEstimate.objNavState.dTimestamp)
    %
    % Output:
    %   12
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-08-2026  Pietro Califano, Codex     First generic navigation-state estimate contract.
    % 11-08-2026  Pietro Califano, Codex     Make numerical validation an explicit strict-mode policy.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % SNavStateEstimate: Construct a state/uncertainty value at one epoch.
    % getCovariance: Return covariance without changing stored representation.
    % getInformation: Return information without changing stored representation.
    % changeReferenceFrame: Rotate the state mean and uncertainty together.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % objNavState: composed navigation-state mean and timestamp.
    % dUncertaintyMatrix: uncertainty in the explicitly selected representation.
    % enumUncertaintyRepresentation: covariance/information/factor representation.
    % enumStateLayout: six-state or nine-state error layout.
    % enumAttitudeErrorConvention: attitude perturbation convention, or NONE.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % SNavState, EnumStateUncertaintyRepresentation,
    % EnumNavStateUncertaintyLayout, EnumAttitudeErrorConvention.
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = private, GetAccess = public)
        objNavState (1,1) SNavState = SNavState()
        dUncertaintyMatrix (:,:) double = zeros(0, 0)
        enumUncertaintyRepresentation (1,1) EnumStateUncertaintyRepresentation = EnumStateUncertaintyRepresentation.COVARIANCE
        enumStateLayout (1,1) EnumNavStateUncertaintyLayout = EnumNavStateUncertaintyLayout.POSITION_VELOCITY
        enumAttitudeErrorConvention (1,1) EnumAttitudeErrorConvention = EnumAttitudeErrorConvention.NONE
    end

    methods (Access = public)
        function self = SNavStateEstimate(objNavState, dUncertaintyMatrix, enumUncertaintyRepresentation, ...
                                         enumStateLayout, enumAttitudeErrorConvention, bStrictValidation)
            %% SIGNATURE
            % self = SNavStateEstimate(objNavState, dUncertaintyMatrix, ...
            %     enumUncertaintyRepresentation, enumStateLayout, ...
            %     enumAttitudeErrorConvention, bStrictValidation)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Associate a navigation state with explicitly described uncertainty
            % at the state's existing timestamp. Matrix shape is always checked;
            % strict numerical validation is optional and disabled by default.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % objNavState                       Navigation-state mean and epoch.
            % dUncertaintyMatrix                Matrix in the declared representation.
            % enumUncertaintyRepresentation     Covariance/information representation.
            % enumStateLayout                   Error-state component layout.
            % enumAttitudeErrorConvention       Attitude perturbation convention.
            % bStrictValidation                 Enable finite/SPD/factor validation.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self                              Constructed estimate value.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 11-08-2026  Pietro Califano, Codex     First implementation.
            % 11-08-2026  Pietro Califano, Codex     Add opt-in strict numerical validation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -----------------------------------------------------------------------------------------------------
            arguments
                objNavState (1,1) SNavState
                dUncertaintyMatrix (:,:) double
                enumUncertaintyRepresentation (1,1) EnumStateUncertaintyRepresentation
                enumStateLayout (1,1) EnumNavStateUncertaintyLayout
                enumAttitudeErrorConvention (1,1) EnumAttitudeErrorConvention
                bStrictValidation (1,1) logical = false
            end

            % Couple each layout to the only meaningful attitude metadata so
            % callers cannot leave the perturbation contract ambiguous.
            bPositionVelocityLayout = enumStateLayout == EnumNavStateUncertaintyLayout.POSITION_VELOCITY;
            bNoAttitudeConvention = enumAttitudeErrorConvention == EnumAttitudeErrorConvention.NONE;
            if bPositionVelocityLayout ~= bNoAttitudeConvention
                error("SNavStateEstimate:InvalidAttitudeErrorConvention", ...
                    "POSITION_VELOCITY requires NONE, while " + ...
                    "POSITION_VELOCITY_ATTITUDE_ERROR requires LEFT_FRAME or RIGHT_POSE.");
            end

            % Matrix size remains a cheap structural invariant in both modes.
            if bPositionVelocityLayout
                ui32ExpectedDimension = uint32(6);
            else
                ui32ExpectedDimension = uint32(9);
            end
            if size(dUncertaintyMatrix, 1) ~= double(ui32ExpectedDimension) || ...
                    size(dUncertaintyMatrix, 2) ~= double(ui32ExpectedDimension)
                error("SNavStateEstimate:InvalidMatrixSize", ...
                    "The declared layout requires an exact %d-by-%d uncertainty matrix.", ...
                    ui32ExpectedDimension, ui32ExpectedDimension);
            end

            % Invoke the matrix-wide scans and factorization only when the
            % caller explicitly requests strict numerical validation.
            if bStrictValidation
                SNavStateEstimate.ValidateUncertainty_(dUncertaintyMatrix, enumUncertaintyRepresentation);
            end

            % Store the explicitly declared value without interpreting matrix
            % shape or contents as metadata.
            self.objNavState = objNavState;
            self.dUncertaintyMatrix = dUncertaintyMatrix;
            self.enumUncertaintyRepresentation = enumUncertaintyRepresentation;
            self.enumStateLayout = enumStateLayout;
            self.enumAttitudeErrorConvention = enumAttitudeErrorConvention;
            self.bDefaultConstructed = false;
        end

        function dCovariance = getCovariance(self)
            %% SIGNATURE
            % dCovariance = self.getCovariance()
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return covariance implied by the stored representation using
            % factor products or triangular solves without mutating self.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self              Navigation-state estimate value.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dCovariance       Symmetric positive-definite covariance matrix.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 11-08-2026  Pietro Califano, Codex     First implementation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % chol
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) SNavStateEstimate
            end
            arguments (Output)
                dCovariance (:,:) double
            end

            enumRepresentation = self.enumUncertaintyRepresentation;
            dStoredMatrix = self.dUncertaintyMatrix;

            % Return covariance representations directly or reconstruct their
            % declared Gram matrix without a factorization.
            if enumRepresentation == EnumStateUncertaintyRepresentation.COVARIANCE
                dCovariance = dStoredMatrix;
                return
            elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER
                dCovariance = SNavStateEstimate.Symmetrize_(dStoredMatrix' * dStoredMatrix);
                return
            elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_LOWER
                dCovariance = SNavStateEstimate.Symmetrize_(dStoredMatrix * dStoredMatrix');
                return
            end

            % Invert information representations through their Cholesky or
            % already-declared triangular factor.
            if enumRepresentation == EnumStateUncertaintyRepresentation.INFORMATION
                dUpperFactor = chol(dStoredMatrix);
                dCovariance = SNavStateEstimate.InvertUpperFactor_(dUpperFactor);
            elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER
                dCovariance = SNavStateEstimate.InvertUpperFactor_(dStoredMatrix);
            else
                dCovariance = SNavStateEstimate.InvertLowerFactor_(dStoredMatrix);
            end
        end

        function dInformation = getInformation(self)
            %% SIGNATURE
            % dInformation = self.getInformation()
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return information implied by the stored representation using
            % factor products or triangular solves without mutating self.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self              Navigation-state estimate value.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dInformation      Symmetric positive-definite information matrix.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 11-08-2026  Pietro Califano, Codex     First implementation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % chol
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) SNavStateEstimate
            end
            arguments (Output)
                dInformation (:,:) double
            end

            enumRepresentation = self.enumUncertaintyRepresentation;
            dStoredMatrix = self.dUncertaintyMatrix;

            % Return information representations directly or reconstruct
            % their declared Gram matrix without a factorization.
            if enumRepresentation == EnumStateUncertaintyRepresentation.INFORMATION
                dInformation = dStoredMatrix;
                return
            elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER
                dInformation = SNavStateEstimate.Symmetrize_(dStoredMatrix' * dStoredMatrix);
                return
            elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_INFORMATION_LOWER
                dInformation = SNavStateEstimate.Symmetrize_(dStoredMatrix * dStoredMatrix');
                return
            end

            % Invert covariance representations through their Cholesky or
            % already-declared triangular factor.
            if enumRepresentation == EnumStateUncertaintyRepresentation.COVARIANCE
                dUpperFactor = chol(dStoredMatrix);
                dInformation = SNavStateEstimate.InvertUpperFactor_(dUpperFactor);
            elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER
                dInformation = SNavStateEstimate.InvertUpperFactor_(dStoredMatrix);
            else
                dInformation = SNavStateEstimate.InvertLowerFactor_(dStoredMatrix);
            end
        end

        function self = changeReferenceFrame(self, dDCM_NewFrameFromFrame)
            %% SIGNATURE
            % self = self.changeReferenceFrame(dDCM_NewFrameFromFrame)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Rotate the composed navigation-state mean and its error-state
            % uncertainty together while preserving timestamp and storage form.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                          Navigation-state estimate value.
            % dDCM_NewFrameFromFrame        Direction-cosine matrix from old to new frame.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self                          Estimate expressed in the new frame.
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 11-08-2026  Pietro Califano, Codex     First implementation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SNavState.changeReferenceFrame, blkdiag, chol
            % -----------------------------------------------------------------------------------------------------
            arguments
                self (1,1) SNavStateEstimate
                dDCM_NewFrameFromFrame (3,3) double {mustBeFinite}
            end

            % Matrix size is immutable and already validated against layout;
            % selecting the fixed-size branch also preserves Coder inference.
            if size(self.dUncertaintyMatrix, 1) == 6
                dFrameJacobian = blkdiag(dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame);
            elseif self.enumAttitudeErrorConvention == EnumAttitudeErrorConvention.LEFT_FRAME
                dFrameJacobian = blkdiag(dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame);
            else
                dFrameJacobian = blkdiag(dDCM_NewFrameFromFrame, dDCM_NewFrameFromFrame, eye(3));
            end

            enumRepresentation = self.enumUncertaintyRepresentation;
            bCovarianceRepresentation = enumRepresentation == EnumStateUncertaintyRepresentation.COVARIANCE || ...
                enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER || ...
                enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_LOWER;

            % Transform the native covariance or information quantity, then
            % restore the caller-selected representation.
            if bCovarianceRepresentation
                dTransformedCovariance = dFrameJacobian * self.getCovariance() * dFrameJacobian';
                dTransformedCovariance = SNavStateEstimate.Symmetrize_(dTransformedCovariance);
                if enumRepresentation == EnumStateUncertaintyRepresentation.COVARIANCE
                    self.dUncertaintyMatrix = dTransformedCovariance;
                elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER
                    self.dUncertaintyMatrix = chol(dTransformedCovariance);
                else
                    self.dUncertaintyMatrix = chol(dTransformedCovariance, "lower");
                end
            else
                dInformation = self.getInformation();
                dTransformedInformation = dFrameJacobian' \ (dInformation / dFrameJacobian);
                dTransformedInformation = SNavStateEstimate.Symmetrize_(dTransformedInformation);
                if enumRepresentation == EnumStateUncertaintyRepresentation.INFORMATION
                    self.dUncertaintyMatrix = dTransformedInformation;
                elseif enumRepresentation == EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER
                    self.dUncertaintyMatrix = chol(dTransformedInformation);
                else
                    self.dUncertaintyMatrix = chol(dTransformedInformation, "lower");
                end
            end

            % Delegate mean rotation to the existing state-only operation; it
            % leaves the delayed-state timestamp unchanged.
            self.objNavState = self.objNavState.changeReferenceFrame(dDCM_NewFrameFromFrame);
        end
    end

    methods (Static, Access = private)
        function dInverseMatrix = InvertUpperFactor_(dUpperFactor)
            %% DESCRIPTION
            % Invert M = R' * R through two triangular solves.
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                dUpperFactor (:,:) double
            end
            arguments (Output)
                dInverseMatrix (:,:) double
            end

            dIdentityMatrix = eye(size(dUpperFactor, 1));
            dInverseMatrix = dUpperFactor \ (dUpperFactor' \ dIdentityMatrix);
            dInverseMatrix = SNavStateEstimate.Symmetrize_(dInverseMatrix);
        end

        function dInverseMatrix = InvertLowerFactor_(dLowerFactor)
            %% DESCRIPTION
            % Invert M = L * L' through two triangular solves.
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                dLowerFactor (:,:) double
            end
            arguments (Output)
                dInverseMatrix (:,:) double
            end

            dIdentityMatrix = eye(size(dLowerFactor, 1));
            dInverseMatrix = dLowerFactor' \ (dLowerFactor \ dIdentityMatrix);
            dInverseMatrix = SNavStateEstimate.Symmetrize_(dInverseMatrix);
        end

        function dSymmetricMatrix = Symmetrize_(dMatrix)
            %% DESCRIPTION
            % Remove solve/product roundoff asymmetry from an SPD result.
            % -----------------------------------------------------------------------------------------------------
            arguments (Input)
                dMatrix (:,:) double
            end
            arguments (Output)
                dSymmetricMatrix (:,:) double
            end

            dSymmetricMatrix = 0.5 * (dMatrix + dMatrix');
        end

        function ValidateUncertainty_(dUncertaintyMatrix, enumUncertaintyRepresentation)
            %% DESCRIPTION
            % Validate representation-specific numerical structure.
            % -----------------------------------------------------------------------------------------------------
            arguments
                dUncertaintyMatrix (:,:) double
                enumUncertaintyRepresentation (1,1) EnumStateUncertaintyRepresentation
            end

            % Reject non-finite inputs before structural or factorization
            % checks so diagnostics remain deterministic.
            if any(~isfinite(dUncertaintyMatrix), "all")
                error("SNavStateEstimate:NonFiniteUncertaintyMatrix", ...
                    "The uncertainty matrix must contain only finite values.");
            end

            bDirectMatrix = enumUncertaintyRepresentation == EnumStateUncertaintyRepresentation.COVARIANCE || ...
                enumUncertaintyRepresentation == EnumStateUncertaintyRepresentation.INFORMATION;
            bUpperFactor = enumUncertaintyRepresentation == ...
                EnumStateUncertaintyRepresentation.SQRT_COVARIANCE_UPPER || ...
                enumUncertaintyRepresentation == EnumStateUncertaintyRepresentation.SQRT_INFORMATION_UPPER;

            % Direct covariance and information matrices share the symmetric
            % positive-definite contract.
            if bDirectMatrix
                if ~issymmetric(dUncertaintyMatrix)
                    error("SNavStateEstimate:AsymmetricUncertaintyMatrix", ...
                        "Direct covariance and information matrices must be symmetric.");
                end
                [~, dCholeskyFlag] = chol(dUncertaintyMatrix);
                if dCholeskyFlag ~= 0.0
                    error("SNavStateEstimate:UncertaintyMatrixNotPositiveDefinite", ...
                        "Direct covariance and information matrices must be positive definite.");
                end
                return
            end

            % Square-root representations must honor their declared exact
            % orientation before the common Cholesky-sign convention check.
            if bUpperFactor && ~istriu(dUncertaintyMatrix)
                error("SNavStateEstimate:UncertaintyFactorNotUpperTriangular", ...
                    "An upper square-root representation requires an upper-triangular factor.");
            elseif ~bUpperFactor && ~istril(dUncertaintyMatrix)
                error("SNavStateEstimate:UncertaintyFactorNotLowerTriangular", ...
                    "A lower square-root representation requires a lower-triangular factor.");
            end
            if any(diag(dUncertaintyMatrix) <= 0.0)
                error("SNavStateEstimate:UncertaintyFactorNonPositiveDiagonal", ...
                    "A square-root uncertainty factor requires a strictly positive diagonal.");
            end
        end
    end
end
