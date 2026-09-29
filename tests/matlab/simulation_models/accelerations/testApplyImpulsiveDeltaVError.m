classdef testApplyImpulsiveDeltaVError < matlab.unittest.TestCase
    %% DESCRIPTION
    % Validate the stochastic impulse-error model independently of scenarios.
    % Check edge cases, output shape, unbiasedness and the fractional magnitude
    % and angular dispersion contracts. Restore the caller's RNG after every test.
    % -------------------------------------------------------------------------------------------------------------

    %% CHANGELOG
    % 29-09-2026  Pietro Califano, Codex gpt-6  Generalize impulse scaling regressions.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
    % -------------------------------------------------------------------------------------------------------------

    methods (TestMethodSetup)
        function preserveCallerRng(self)
            %% SIGNATURE
            % preserveCallerRng(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Restore the caller's RNG state after each stochastic test.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; register a test teardown callback.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Isolate test randomness.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % rng, matlab.unittest.TestCase.addTeardown.
            % ---------------------------------------------------------------------------------------------------------

            strCallerRngState = rng;
            self.addTeardown(@() rng(strCallerRngState));
        end
    end

    methods (Test)

        function testOutputDimension(self)
            %% SIGNATURE
            % testOutputDimension(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Verify the three-component output contract.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Check the fixed output shape for a non-axis-aligned impulse.
            dNominalDeltaV = [0.1; -0.05; 0.2];
            dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.01, deg2rad(0.5));
            self.verifySize(dRealizedDeltaV, [3, 1]);
        end

        function testZeroSigmaReturnsNominal(self)
            %% SIGNATURE
            % testZeroSigmaReturnsNominal(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve arbitrary nominal impulses when both dispersions are zero.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Preserve the input with both error components disabled.
            rng('default');
            for dTrialIdx = 1:10
                dNominalDeltaV = randn(3, 1);
                dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.0, 0.0);
                self.verifyEqual(dRealizedDeltaV, dNominalDeltaV, 'AbsTol', 1e-15, ...
                    sprintf('Zero sigma must return nominal DV (trial %d)', dTrialIdx));
            end
        end

        function testNearZeroDVReturnsUnchanged(self)
            %% SIGNATURE
            % testNearZeroDVReturnsUnchanged(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve the zero-impulse guard before direction normalization.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Return a zero impulse unchanged before normalization.
            dZeroDeltaV = zeros(3, 1);
            dRealizedDeltaV = ApplyImpulsiveDeltaVError(dZeroDeltaV, 0.1, deg2rad(1));
            self.verifyEqual(dRealizedDeltaV, dZeroDeltaV, 'AbsTol', 1e-15, ...
                'Near-zero DV must be returned unchanged');
        end

        function testStatisticalUnbiasedness(self)
            %% SIGNATURE
            % testStatisticalUnbiasedness(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check the ensemble mean against the nominal impulse.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Compare the ensemble mean with the zero-mean error contract.
            rng('default');
            dNumDraws = 10000;
            dNominalDeltaV = [1.0; 0.0; 0.0]; % Unit impulse along x.
            dSigmaMagnitude = 0.05; % Fractional magnitude sigma.
            dSigmaDirection = deg2rad(1.0); % Angular sigma in radians.

            dImpulseSamples = zeros(3, dNumDraws);
            for dDrawIdx = 1:dNumDraws
                dImpulseSamples(:, dDrawIdx) = ApplyImpulsiveDeltaVError( ...
                    dNominalDeltaV, dSigmaMagnitude, dSigmaDirection);
            end

            dMeanDeltaV = mean(dImpulseSamples, 2);
            dRelativeBias = norm(dMeanDeltaV - dNominalDeltaV) / norm(dNominalDeltaV);

            self.verifyLessThan(dRelativeBias, 0.01, ...
                sprintf('Mean DV bias too large (relBias=%.4f)', dRelativeBias));
        end

        function testMagnitudeErrorScalesWithSigma(self)
            %% SIGNATURE
            % testMagnitudeErrorScalesWithSigma(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check fractional magnitude dispersion with angular error disabled.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Isolate the fractional parallel error from angular effects.
            rng(42);
            dNumDraws = 8000;
            dNominalDeltaV = [2.0; 1.0; -0.5];
            dNominalMagnitude = norm(dNominalDeltaV);
            dSigmaMagnitude = 0.1;   % 10%

            dMagnitudes = zeros(1, dNumDraws);
            for dDrawIdx = 1:dNumDraws
                dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, dSigmaMagnitude, 0.0);
                dMagnitudes(dDrawIdx) = norm(dRealizedDeltaV);
            end

            dEstimatedSigmaFrac = std(dMagnitudes) / dNominalMagnitude;
            self.verifyEqual(dEstimatedSigmaFrac, dSigmaMagnitude, 'RelTol', 0.1, ...
                sprintf('Magnitude error std differs from sigma (got %.4f, expected %.4f)', ...
                    dEstimatedSigmaFrac, dSigmaMagnitude));
        end

        function testDirectionErrorOrthogonalInMean(self)
            %% SIGNATURE
            % testDirectionErrorOrthogonalInMean(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Keep angular perturbations perpendicular to the nominal impulse.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Isolate angular error and measure its parallel component.
            rng(7);
            dNumDraws = 8000;
            dNominalDeltaV = [0; 0; 1.5];   % Along z
            dUnitImpulse = dNominalDeltaV / norm(dNominalDeltaV);
            dSigmaDirection = deg2rad(2.0);

            dParallelErrors = zeros(1, dNumDraws);
            for dDrawIdx = 1:dNumDraws
                dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.0, dSigmaDirection);
                dImpulseError = dRealizedDeltaV - dNominalDeltaV;
                dParallelErrors(dDrawIdx) = dot(dImpulseError, dUnitImpulse);
            end

            % Verify the tangent perturbation has no parallel component.
            dMeanParallelError = mean(dParallelErrors);
            self.verifyEqual(dMeanParallelError, 0.0, 'AbsTol', 0.02, ...
                sprintf('Direction error must be zero-mean along nominal direction (got %.4f)', ...
                    dMeanParallelError));
        end

        function testImpulseErrorIsScaleInvariant(self)
            %% SIGNATURE
            % testImpulseErrorIsScaleInvariant(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve fractional and angular errors when rescaling velocity
            % units. Replay the same draws across several impulse magnitudes.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert identical normalized realizations.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Check velocity-unit invariance.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, rng.
            % ---------------------------------------------------------------------------------------------------------

            % Replay common draws so the comparison isolates unit rescaling.
            dNominalDeltaV = [0.3; -0.5; 0.8];
            dImpulseScales = [1e-7, 1e-3, 1.0, 10.0];
            rng(43);
            dReferenceDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.05, 0.01);
            for dImpulseScale = dImpulseScales
                rng(43);
                dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                    dImpulseScale * dNominalDeltaV, 0.05, 0.01);
                self.verifyEqual(dRealizedDeltaV / dImpulseScale, ...
                    dReferenceDeltaV, 'AbsTol', 5e-15);
            end
        end

        function testDirectionSigmaAcrossMagnitudes(self)
            %% SIGNATURE
            % testDirectionSigmaAcrossMagnitudes(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check angular RMS across eight decades of nominal impulse
            % magnitude with fractional magnitude error disabled. Use generic
            % numeric inputs, independent of mission profiles and units.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self  MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; check angular RMS against the small-angle input sigma.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  Cover small-burn direction scale.
            % 29-09-2026  Pietro Califano, Codex gpt-6  Generalize impulse magnitudes.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError.
            % ---------------------------------------------------------------------------------------------------------

            % Span small and large impulses without depending on campaign data.
            rng(20260927);
            dBurnMagnitudes = logspace(-7, 1, 5);
            dSigmaDirection = 0.01; % rad
            ui32NumDraws = uint32(1200);

            for ui32BurnIdx = uint32(1):uint32(numel(dBurnMagnitudes))
                dNominalDeltaV = [dBurnMagnitudes(ui32BurnIdx); 0.0; 0.0];
                dAngularErrors = zeros(1, double(ui32NumDraws));
                for ui32DrawIdx = uint32(1):ui32NumDraws
                    dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                        dNominalDeltaV, 0.0, dSigmaDirection);
                    dAngularErrors(ui32DrawIdx) = atan2( ...
                        norm(cross(dNominalDeltaV, dRealizedDeltaV)), ...
                        dot(dNominalDeltaV, dRealizedDeltaV));
                end

                dAngularRms = sqrt(mean(dAngularErrors.^2));
                self.verifyEqual(dAngularRms, dSigmaDirection, 'RelTol', 0.15, ...
                    sprintf('Direction sigma is wrong for impulse magnitude %.3g.', ...
                        dBurnMagnitudes(ui32BurnIdx)));
            end
        end

        function testOutputMagnitudeWithOnlyMagnitudeError(self)
            %% SIGNATURE
            % testOutputMagnitudeWithOnlyMagnitudeError(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve impulse direction when angular dispersion is zero.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Check direction preservation with only fractional magnitude error.
            rng(13);
            dNominalDeltaV = [0.3; -0.5; 0.8];
            dUnitNominalImpulse = dNominalDeltaV / norm(dNominalDeltaV);

            for dTrialIdx = 1:20
                dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.05, 0.0);
                dUnitRealizedImpulse = dRealizedDeltaV / norm(dRealizedDeltaV);

                dAngleError = acos(min(1, abs(dot(dUnitRealizedImpulse, dUnitNominalImpulse))));
                % Allow acos precision loss close to unit alignment.
                self.verifyLessThan(dAngleError, 1e-7, ...
                    sprintf('Direction must be unchanged when sigma_dir=0 (trial %d)', dTrialIdx));
            end
        end

        function testRandomDVsProduceFiniteOutputs(self)
            %% SIGNATURE
            % testRandomDVsProduceFiniteOutputs(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check finite three-component outputs across randomized valid inputs.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Sweep valid random impulses and dispersions for finite output.
            rng('default');
            for dTrialIdx = 1:50
                dNominalDeltaV = randn(3, 1) * (0.001 + 5 * rand());
                dSigmaMagnitude = 0.2 * rand();
                dSigmaDirection = deg2rad(5 * rand());

                dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                    dNominalDeltaV, dSigmaMagnitude, dSigmaDirection);

                self.verifySize(dRealizedDeltaV, [3, 1]);
                self.verifyTrue(all(isfinite(dRealizedDeltaV)), ...
                    sprintf('Output must be finite at trial %d', dTrialIdx));
            end
        end

        function testSmallSigmaPreservesNominalApproximately(self)
            %% SIGNATURE
            % testSmallSigmaPreservesNominalApproximately(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Bound relative changes for small magnitude and angular dispersions.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the documented impulse-error behavior.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Document the behavioral contract.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Compare small-dispersion realizations with the nominal impulse.
            rng(21);
            dNominalDeltaV = [1.0; -0.3; 0.7];
            dNominalMagnitude = norm(dNominalDeltaV);
            dSigmaMagnitude = 1e-4;
            dSigmaDirection = deg2rad(0.01);

            for dTrialIdx = 1:20
                dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                    dNominalDeltaV, dSigmaMagnitude, dSigmaDirection);
                dRelativeChange = norm(dRealizedDeltaV - dNominalDeltaV) / dNominalMagnitude;
                self.verifyLessThan(dRelativeChange, 0.01, ...
                    sprintf('Tiny sigma must keep output near nominal (trial %d, relDiff=%.4e)', ...
                        dTrialIdx, dRelativeChange));
            end
        end

    end
end
