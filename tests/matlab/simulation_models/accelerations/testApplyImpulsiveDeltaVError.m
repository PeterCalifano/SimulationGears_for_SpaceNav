classdef testApplyImpulsiveDeltaVError < matlab.unittest.TestCase
    %% DESCRIPTION
    % Validate the stochastic impulse-error model independently of scenarios.
    % Check exact rotation, Gaussian moments, signed magnitude scaling and the
    % RNG contract. Restore the caller's RNG after every test.
    % -------------------------------------------------------------------------------------------------------------

    %% CHANGELOG
    % 29-09-2026  Pietro Califano, Codex gpt-6  Generalize impulse scaling regressions.
    % 29-09-2026  Pietro Califano, Codex gpt-6  Cover exact finite-angle rotation.
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

        function testGaussianAngleEnsembleMean(self)
            %% SIGNATURE
            % testGaussianAngleEnsembleMean(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check the mean and its sampling uncertainty for independent Gaussian
            % magnitude and signed-angle draws at finite angular dispersion.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the finite-angle impulse contract.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Use exact Gaussian cosine moments, including signed magnitude noise.
            rng('default');
            dNumDraws = 10000;
            dNominalDeltaV = [1.0; 0.0; 0.0];
            dSigmaMagnitude = 0.05;
            dSigmaDirection = 0.8;
            dImpulseSamples = zeros(3, dNumDraws);
            for dDrawIdx = 1:dNumDraws
                dImpulseSamples(:, dDrawIdx) = ApplyImpulsiveDeltaVError( ...
                    dNominalDeltaV, dSigmaMagnitude, dSigmaDirection);
            end

            % Account for the longitudinal mean contraction of rotated vectors.
            dMeanCosine = exp(-0.5 * dSigmaDirection^2);
            dSecondCosMoment = 0.5 * (1.0 + exp(-2.0 * dSigmaDirection^2));
            dSecondScaleMoment = 1.0 + dSigmaMagnitude^2;
            dParallelVariance = dSecondScaleMoment * dSecondCosMoment - dMeanCosine^2;
            dTransverseVariance = 0.5 * dSecondScaleMoment * (1.0 - dSecondCosMoment);
            dMeanStd = sqrt([dParallelVariance; ...
                dTransverseVariance; dTransverseVariance] / dNumDraws);
            dExpectedMean = dMeanCosine * dNominalDeltaV;
            self.verifyLessThan(abs(mean(dImpulseSamples, 2) - dExpectedMean), 6.0 * dMeanStd);
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

        function testDirectionErrorPreservesMagnitude(self)
            %% SIGNATURE
            % testDirectionErrorPreservesMagnitude(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve the nominal impulse norm with direction-only noise across
            % small, finite and wrapped angular dispersions.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert the finite-angle impulse contract.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, MATLAB unit-test framework.
            % ---------------------------------------------------------------------------------------------------------

            % Sweep angular dispersions on a non-axis-aligned impulse.
            rng(7);
            dNominalDeltaV = [-0.3; 2.0; 1.5];
            dAngularSigmas = [0.01, 0.6, 1.7];
            for dSigmaDirection = dAngularSigmas
                for dDrawIdx = 1:100
                    dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                        dNominalDeltaV, 0.0, dSigmaDirection);
                    self.verifyEqual(norm(dRealizedDeltaV), norm(dNominalDeltaV), ...
                        'RelTol', 1e-14);
                end
            end
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
            % Keep magnitude-only impulses collinear with the nominal impulse.
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

            % Check collinearity while allowing the signed magnitude factor.
            rng(13);
            dNominalDeltaV = [0.3; -0.5; 0.8];
            dUnitNominalImpulse = dNominalDeltaV / norm(dNominalDeltaV);

            for dTrialIdx = 1:20
                dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.05, 0.0);
                dUnitRealizedImpulse = dRealizedDeltaV / norm(dRealizedDeltaV);

                dAngleError = acos(min(1, abs(dot(dUnitRealizedImpulse, dUnitNominalImpulse))));
                % Allow acos precision loss close to unit alignment.
                self.verifyLessThan(dAngleError, 1e-7, ...
                    sprintf('Impulse must remain collinear when sigma_dir=0 (trial %d)', dTrialIdx));
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

        function testFiniteAngleMatchesTrigOracle(self)
            %% SIGNATURE
            % testFiniteAngleMatchesTrigOracle(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare the actual sampled impulse with an independent sine/cosine
            % oracle near zero, at finite angles and beyond one full rotation.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; check vector orientation and magnitude against independent geometry.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, ReadImpulseDraws_, rng.
            % ---------------------------------------------------------------------------------------------------------

            dNominalDeltaV = [0.3; -0.5; 0.8];
            dTargetAngles = [1e-7, 1e-5, pi / 6, pi / 2, pi, 2.0 * pi + 0.4];
            dMagnitudeSigmas = [0.0, 0.1];

            % Replay the existing draws while selecting each deterministic angle.
            for dSigmaMagnitude = dMagnitudeSigmas
                for dTargetAngle = dTargetAngles
                    rng(43);
                    strReplayRng = rng;
                    [dMagnitudeDraw, dUnitAxis, dAngularDraw] = ...
                        ReadImpulseDraws_(dNominalDeltaV);
                    dSigmaDirection = dTargetAngle / abs(dAngularDraw);
                    dSampleAngle = dSigmaDirection * dAngularDraw;
                    dSignedScale = 1.0 + dSigmaMagnitude * dMagnitudeDraw;
                    dExpectedDeltaV = dSignedScale * (cos(dSampleAngle) * dNominalDeltaV + ...
                        sin(dSampleAngle) * cross(dUnitAxis, dNominalDeltaV));

                    rng(strReplayRng);
                    dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                        dNominalDeltaV, dSigmaMagnitude, dSigmaDirection);
                    self.verifyEqual(dRealizedDeltaV, dExpectedDeltaV, ...
                        'AbsTol', 64.0 * eps * norm(dNominalDeltaV));
                end
            end
        end

        function testPrincipalAngleWrapsAtPi(self)
            %% SIGNATURE
            % testPrincipalAngleWrapsAtPi(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Match the shortest pointing angle after finite signed rotations,
            % including draws larger than pi and two pi.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; verify principal-angle wrapping without reinterpreting input sigma.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, ReadImpulseDraws_, rng.
            % ---------------------------------------------------------------------------------------------------------

            dNominalDeltaV = [-0.7; 0.2; 1.0];
            dTargetAngles = [pi / 2, 1.25 * pi, 2.0 * pi + 0.7, 3.0 * pi];

            % Compare geodesic angles with the folded signed draw.
            for dTargetAngle = dTargetAngles
                rng(51);
                strReplayRng = rng;
                [~, ~, dAngularDraw] = ReadImpulseDraws_(dNominalDeltaV);
                dSigmaDirection = dTargetAngle / abs(dAngularDraw);
                rng(strReplayRng);
                dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                    dNominalDeltaV, 0.0, dSigmaDirection);
                dPrincipalAngle = atan2(norm(cross(dNominalDeltaV, dRealizedDeltaV)), ...
                    dot(dNominalDeltaV, dRealizedDeltaV));
                dExpectedAngle = acos(cos(dSigmaDirection * dAngularDraw));
                self.verifyEqual(dPrincipalAngle, dExpectedAngle, 'AbsTol', 1e-12);
                self.verifyGreaterThanOrEqual(dPrincipalAngle, 0.0);
                self.verifyLessThanOrEqual(dPrincipalAngle, pi);
            end
        end

        function testSmallAngleConvergesToTangent(self)
            %% SIGNATURE
            % testSmallAngleConvergesToTangent(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Recover the first-order tangent model with quadratic error as the
            % sampled angular error decreases.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; assert second-order convergence to the existing tangent approximation.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, ReadImpulseDraws_, rng.
            % ---------------------------------------------------------------------------------------------------------

            dNominalDeltaV = [0.3; -0.5; 0.8];
            dTargetAngles = [0.04, 0.02, 0.01];
            dRelativeErrors = zeros(size(dTargetAngles));

            % Hold the axis and Gaussian draws fixed while reducing the angle.
            for dAngleIdx = 1:numel(dTargetAngles)
                rng(43);
                strReplayRng = rng;
                [~, dUnitAxis, dAngularDraw] = ReadImpulseDraws_(dNominalDeltaV);
                dSigmaDirection = dTargetAngles(dAngleIdx) / abs(dAngularDraw);
                dSampleAngle = dSigmaDirection * dAngularDraw;
                dFirstOrderDeltaV = dNominalDeltaV + ...
                    dSampleAngle * cross(dUnitAxis, dNominalDeltaV);
                rng(strReplayRng);
                dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                    dNominalDeltaV, 0.0, dSigmaDirection);
                dRelativeErrors(dAngleIdx) = ...
                    norm(dRealizedDeltaV - dFirstOrderDeltaV) / norm(dNominalDeltaV);
            end

            % Check the expected second-order remainder rather than bitwise equality.
            self.verifyGreaterThan(dRelativeErrors, zeros(size(dRelativeErrors)));
            self.verifyLessThan(dRelativeErrors, 0.51 * dTargetAngles.^2);
            self.verifyEqual(dRelativeErrors(1:2) ./ dRelativeErrors(2:3), ...
                [4.0, 4.0], 'RelTol', 0.01);
        end

        function testSignedMagnitudeScale(self)
            %% SIGNATURE
            % testSignedMagnitudeScale(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Keep the Gaussian magnitude factor signed, including negative
            % realizations; preserve its absolute magnitude without clipping.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; verify signed Gaussian scaling and exact rotated magnitude.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, ReadImpulseDraws_, rng.
            % ---------------------------------------------------------------------------------------------------------

            dNominalDeltaV = [0.3; -0.5; 0.8];

            % Select a deterministic negative draw without pinning a Gaussian value.
            for dSeed = 1:64
                rng(dSeed);
                strReplayRng = rng;
                [dMagnitudeDraw, dUnitAxis, dAngularDraw] = ...
                    ReadImpulseDraws_(dNominalDeltaV);
                if dMagnitudeDraw < 0.0
                    break
                end
            end
            self.assertLessThan(dMagnitudeDraw, 0.0);
            dSigmaMagnitude = 2.0 / abs(dMagnitudeDraw);
            dSigmaDirection = 0.6;
            dSignedScale = 1.0 + dSigmaMagnitude * dMagnitudeDraw;
            dSampleAngle = dSigmaDirection * dAngularDraw;
            dExpectedDeltaV = dSignedScale * (cos(dSampleAngle) * dNominalDeltaV + ...
                sin(dSampleAngle) * cross(dUnitAxis, dNominalDeltaV));

            rng(strReplayRng);
            dRealizedDeltaV = ApplyImpulsiveDeltaVError( ...
                dNominalDeltaV, dSigmaMagnitude, dSigmaDirection);
            self.verifyEqual(dRealizedDeltaV, dExpectedDeltaV, ...
                'AbsTol', 64.0 * eps * norm(dNominalDeltaV));
            self.verifyEqual(norm(dRealizedDeltaV), ...
                abs(dSignedScale) * norm(dNominalDeltaV), 'RelTol', 1e-14);
        end

        function testRngDrawOrderIsPreserved(self)
            %% SIGNATURE
            % testRngDrawOrderIsPreserved(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Consume the existing magnitude, axis and angle draws without changing
            % caller RNG order for enabled and disabled direction noise.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; compare complete RNG states after the expected number of draws.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, rng, randn.
            % ---------------------------------------------------------------------------------------------------------

            dNominalDeltaV = [0.3; -0.5; 0.8];
            dSigmaPairs = [0.0, 0.0; 0.1, 0.0; 0.0, 0.6; 0.1, 0.6];

            % Compute only the expected draw sequence, independent of rotation math.
            for dPairIdx = 1:size(dSigmaPairs, 1)
                rng(63);
                strReplayRng = rng;
                randn(1, 1);
                if dSigmaPairs(dPairIdx, 2) > 0.0
                    randn(3, 1);
                    randn(1, 1);
                end
                strExpectedRng = rng;
                rng(strReplayRng);
                ApplyImpulsiveDeltaVError(dNominalDeltaV, ...
                    dSigmaPairs(dPairIdx, 1), dSigmaPairs(dPairIdx, 2));
                self.verifyEqual(rng, strExpectedRng);
            end
        end

        function testNegligibleImpulseConsumesNoRng(self)
            %% SIGNATURE
            % testNegligibleImpulseConsumesNoRng(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve the existing sub-epsilon impulse guard before consuming any
            % Gaussian realization.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; check the unchanged impulse and RNG state at the negligible boundary.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6  Validate finite-angle impulse errors.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ApplyImpulsiveDeltaVError, rng.
            % ---------------------------------------------------------------------------------------------------------

            dNominalDeltaV = [eps / 2.0; 0.0; 0.0];
            rng(79);
            strCallerRng = rng;
            dRealizedDeltaV = ApplyImpulsiveDeltaVError(dNominalDeltaV, 0.1, 1.0);
            self.verifyEqual(dRealizedDeltaV, dNominalDeltaV);
            self.verifyEqual(rng, strCallerRng);
        end

    end
end

function [dMagnitudeDraw, dUnitAxis, dAngularDraw] = ReadImpulseDraws_(dNominalDeltaV)
%% SIGNATURE
% [dMagnitudeDraw, dUnitAxis, dAngularDraw] = ReadImpulseDraws_(dNominalDeltaV)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Read the existing Gaussian realization for deterministic physical oracles.
% Project the axis through the vector triple-product identity; keep exact
% rotation mathematics independent of the MathCore implementation under test.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dNominalDeltaV    Nonzero nominal impulse in caller velocity units.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMagnitudeDraw   Standard Gaussian magnitude realization.
% dUnitAxis        Unit axis perpendicular to the nominal impulse.
% dAngularDraw     Standard Gaussian signed-angle realization.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-09-2026  Pietro Califano, Codex gpt-6  Replay noise for independent rotation oracles.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% randn, cross.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dNominalDeltaV (3, 1) double {mustBeReal, mustBeFinite}
end

arguments (Output)
    dMagnitudeDraw (1, 1) double
    dUnitAxis (3, 1) double
    dAngularDraw (1, 1) double
end

% Read the public sampler's specified sequence.
dMagnitudeDraw = randn(1, 1);
dAxisDraw = randn(3, 1);
dAngularDraw = randn(1, 1);

% Project through an independent identity without calling the rotation provider.
dUnitImpulse = dNominalDeltaV / norm(dNominalDeltaV);
dProjectedAxis = cross(dUnitImpulse, cross(dAxisDraw, dUnitImpulse));
assert(norm(dProjectedAxis) > eps, 'The oracle fixture needs a nondegenerate axis.');
dUnitAxis = dProjectedAxis / norm(dProjectedAxis);

end
