classdef testApplyImpulsiveDeltaVError < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for ApplyImpulsiveDeltaVError (stochastic magnitude and direction DeltaV error model).
    % Tests deterministic edge cases, output shape, statistical unbiasedness, magnitude/direction
    % error scaling, and robustness to near-zero inputs.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)

        function testOutputDimension(testCase)
            % Output must always be [3x1]
            dDV = [0.1; -0.05; 0.2];
            dResult = ApplyImpulsiveDeltaVError(dDV, 0.01, deg2rad(0.5));
            testCase.verifySize(dResult, [3, 1]);
        end

        function testZeroSigmaReturnsNominal(testCase)
            % With both sigmas = 0, the output must equal the nominal DV exactly
            rng('default');
            for idTrial = 1:10
                dDV = randn(3, 1);
                dResult = ApplyImpulsiveDeltaVError(dDV, 0.0, 0.0);
                testCase.verifyEqual(dResult, dDV, 'AbsTol', 1e-15, ...
                    sprintf('Zero sigma must return nominal DV (trial %d)', idTrial));
            end
        end

        function testNearZeroDVReturnsUnchanged(testCase)
            % DV below machine epsilon must be returned unchanged (early-return guard)
            dDV_zero = zeros(3, 1);
            dResult  = ApplyImpulsiveDeltaVError(dDV_zero, 0.1, deg2rad(1));
            testCase.verifyEqual(dResult, dDV_zero, 'AbsTol', 1e-15, ...
                'Near-zero DV must be returned unchanged');
        end

        function testStatisticalUnbiasedness(testCase)
            % Over a large ensemble, the mean of the perturbed DV must equal the nominal
            % (error model is zero-mean Gaussian)
            rng('default');
            dNsamples = 10000;
            dDV_nominal = [1.0; 0.0; 0.0];   % 1 km/s along x
            dSigmaMag   = 0.05;               % 5% magnitude error
            dSigmaDir   = deg2rad(1.0);       % 1 deg direction error

            dDV_samples = zeros(3, dNsamples);
            for idS = 1:dNsamples
                dDV_samples(:, idS) = ApplyImpulsiveDeltaVError(dDV_nominal, dSigmaMag, dSigmaDir);
            end

            dMeanDV = mean(dDV_samples, 2);
            dRelBias = norm(dMeanDV - dDV_nominal) / norm(dDV_nominal);

            testCase.verifyLessThan(dRelBias, 0.01, ...
                sprintf('Mean DV bias too large (relBias=%.4f)', dRelBias));
        end

        function testMagnitudeErrorScalesWithSigma(testCase)
            % std(|DV_out|) / |DV_nominal| should approximate sigma_mag
            rng(42);
            dNsamples    = 8000;
            dDV_nominal  = [2.0; 1.0; -0.5];
            dNomMag      = norm(dDV_nominal);
            dSigmaMag    = 0.1;   % 10%

            dMagnitudes = zeros(1, dNsamples);
            for idS = 1:dNsamples
                dOut = ApplyImpulsiveDeltaVError(dDV_nominal, dSigmaMag, 0.0);
                dMagnitudes(idS) = norm(dOut);
            end

            dEstimatedSigmaFrac = std(dMagnitudes) / dNomMag;
            testCase.verifyEqual(dEstimatedSigmaFrac, dSigmaMag, 'RelTol', 0.1, ...
                sprintf('Magnitude error std should be ~sigma_mag (got %.4f, expected %.4f)', ...
                        dEstimatedSigmaFrac, dSigmaMag));
        end

        function testDirectionErrorOrthogonalInMean(testCase)
            % Direction error adds a tangential perturbation orthogonal to the nominal DV.
            % The component of the error along the nominal direction should be zero-mean.
            rng(7);
            dNsamples   = 8000;
            dDV_nominal = [0; 0; 1.5];   % Along z
            dUnitDV     = dDV_nominal / norm(dDV_nominal);
            dSigmaDir   = deg2rad(2.0);

            dParallelErrors = zeros(1, dNsamples);
            for idS = 1:dNsamples
                dOut = ApplyImpulsiveDeltaVError(dDV_nominal, 0.0, dSigmaDir);
                dErr = dOut - dDV_nominal;
                dParallelErrors(idS) = dot(dErr, dUnitDV);
            end

            % The direction error term is purely tangential, so its projection on the
            % nominal direction should have zero mean (it only has a magnitude component with sigma=0)
            dMeanParallel = mean(dParallelErrors);
            testCase.verifyEqual(dMeanParallel, 0.0, 'AbsTol', 0.02, ...
                sprintf('Direction error must be zero-mean along nominal direction (got %.4f)', dMeanParallel));
        end

        function testOutputMagnitudeWithOnlyMagnitudeError(testCase)
            % With direction sigma = 0, the DV direction must be preserved exactly
            rng(13);
            dDV_nominal = [0.3; -0.5; 0.8];
            dUnitNominal = dDV_nominal / norm(dDV_nominal);

            for idTrial = 1:20
                dOut = ApplyImpulsiveDeltaVError(dDV_nominal, 0.05, 0.0);
                dUnitOut = dOut / norm(dOut);

                dAngleError = acos(min(1, abs(dot(dUnitOut, dUnitNominal))));
                % Tolerance is sqrt(eps) ~ 1.5e-8: acos near 1 has limited precision
                testCase.verifyLessThan(dAngleError, 1e-7, ...
                    sprintf('Direction must be unchanged when sigma_dir=0 (trial %d)', idTrial));
            end
        end

        function testRandomDVsProduceFiniteOutputs(testCase)
            % For a range of random DV magnitudes and sigma values, outputs must be finite
            rng('default');
            for idTrial = 1:50
                dDV        = randn(3, 1) * (0.001 + 5 * rand());
                dSigmaMag  = 0.2 * rand();
                dSigmaDir  = deg2rad(5 * rand());

                dOut = ApplyImpulsiveDeltaVError(dDV, dSigmaMag, dSigmaDir);

                testCase.verifySize(dOut, [3, 1]);
                testCase.verifyTrue(all(isfinite(dOut)), ...
                    sprintf('Output must be finite at trial %d', idTrial));
            end
        end

        function testSmallSigmaPreservesNominalApproximately(testCase)
            % With very small sigma values, the perturbed DV must stay close to nominal
            rng(21);
            dDV_nominal = [1.0; -0.3; 0.7];
            dNomMag     = norm(dDV_nominal);
            dSigmaMag   = 1e-4;
            dSigmaDir   = deg2rad(0.01);

            for idTrial = 1:20
                dOut     = ApplyImpulsiveDeltaVError(dDV_nominal, dSigmaMag, dSigmaDir);
                dRelDiff = norm(dOut - dDV_nominal) / dNomMag;
                testCase.verifyLessThan(dRelDiff, 0.01, ...
                    sprintf('Tiny sigma must keep output near nominal (trial %d, relDiff=%.4e)', ...
                            idTrial, dRelDiff));
            end
        end

    end
end
