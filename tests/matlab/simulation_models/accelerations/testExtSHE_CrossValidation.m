classdef testExtSHE_CrossValidation < matlab.unittest.TestCase
    %% DESCRIPTION
    % Cross-validate exterior spherical-harmonic potential, acceleration and
    % analytical position Jacobians. Use raw acceleration finite differences,
    % independent polar limits, unit scaling and source/MEX comparisons.
    % Keep central gravity separate from the tested harmonic perturbation.
    % Example: objResults = runtests('testExtSHE_CrossValidation');
    % Output: Test results whose Passed flags are true when all contracts hold.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 07-10-2026  Pietro Califano  Extend analytical gravity and polar validation.
    % 10-10-2026  Pietro Califano  Document validation contracts and clarify build calls.
    % 11-10-2026  Pietro Califano  Exercise private harmonic tables through the public basis.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % matlab.unittest, MATLAB Coder, SimulationGears gravity models and legacy reference.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth = 398600.4418;   % [km^3/s^2]
        dRadiusEarth = 6378.137;  % [km]
        dJ2Earth = 1.08263e-3;    % [-]
    end

    properties (TestParameter)
        dTestPosition = struct( ...
            'equatorial_LEO', [6800; 0; 0], ...
            'polar_LEO', [0; 0; 6800], ...
            'near_north_pole', [1e-8; -2e-8; 6800], ...
            'near_south_pole', [1e-8; 2e-8; -6800], ...
            'inclined_LEO', [4000; 3000; 4500], ...
            'GEO', [42164; 0; 0], ...
            'inclined_MEO', [15000; 10000; 8000], ...
            'high_lat', [1000; 500; 6500]);
    end

    methods (Static)
        function charRepoRoot = GetRepoRoot()
            charRepoRoot = fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
        end

        function charLegacyDir = GetLegacySHEDir()
            charLegacyDir = fullfile(testExtSHE_CrossValidation.GetRepoRoot(), ...
                'matlab', 'simulation_models', 'accelerations', '.deprecated');
        end

        function EnsureLegacySHEOnPath()
            charLegacyDir = testExtSHE_CrossValidation.GetLegacySHEDir();
            if isempty(which('SphericalHarmonics'))
                addpath(charLegacyDir);
            end
        end

        function ui32Count = GetCoeffCount(ui32MaxDeg)
            ui32Count = uint32((double(ui32MaxDeg + 1) * double(ui32MaxDeg + 2)) / 2 - 2);
        end

        function [dCmat, dSmat] = BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, dScale, dSeed)
            rng(dSeed);

            dCmat = zeros(double(ui32MaxDeg) + 1);
            dSmat = zeros(double(ui32MaxDeg) + 1);

            for l = 2:double(ui32MaxDeg)
                for m = 0:l
                    dCmat(l + 1, m + 1) = dScale * randn();
                    dSmat(l + 1, m + 1) = dScale * randn();
                end
            end
        end

        function dCSlmCols = ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg)
            dCSlmCols = zeros(double(testExtSHE_CrossValidation.GetCoeffCount(ui32MaxDeg)), 2);

            idPair = 1;
            for l = 1:double(ui32MaxDeg)
                for m = 0:l
                    if l == 1 && m == 0
                        continue;
                    end

                    if m == 0
                        dScaleFactor = sqrt(factorial(l + m) / (factorial(l - m) * (2 * l + 1)));
                    else
                        dScaleFactor = sqrt(factorial(l + m) / (factorial(l - m) * 2 * (2 * l + 1)));
                    end

                    dCSlmCols(idPair, 1) = dCmat(l + 1, m + 1) / dScaleFactor;
                    dCSlmCols(idPair, 2) = dSmat(l + 1, m + 1) / dScaleFactor;
                    idPair = idPair + 1;
                end
            end
        end

        function [dCmat, dSmat] = ConvertColPairsToMatrix(dCSlmCols, ui32MaxDeg)
            dCmat = zeros(double(ui32MaxDeg) + 1);
            dSmat = zeros(double(ui32MaxDeg) + 1);

            idPair = 1;
            for l = 1:double(ui32MaxDeg)
                for m = 0:l
                    if l == 1 && m == 0
                        continue;
                    end

                    if m == 0
                        dScaleFactor = sqrt(factorial(l + m) / (factorial(l - m) * (2 * l + 1)));
                    else
                        dScaleFactor = sqrt(factorial(l + m) / (factorial(l - m) * 2 * (2 * l + 1)));
                    end

                    dCmat(l + 1, m + 1) = dCSlmCols(idPair, 1) * dScaleFactor;
                    dSmat(l + 1, m + 1) = dCSlmCols(idPair, 2) * dScaleFactor;
                    idPair = idPair + 1;
                end
            end
        end

        function dAccPert = EvaluateLegacyPerturbation(dPos, ui32MaxDeg, dMu, dRadius, dCmat, dSmat)
            testExtSHE_CrossValidation.EnsureLegacySHEOnPath();
            [~, dAccTotal] = SphericalHarmonics(dPos.', double(ui32MaxDeg), double(ui32MaxDeg), ...
                dMu, dRadius, dCmat, dSmat);
            dAccPert = dAccTotal.' + dMu * dPos / norm(dPos)^3;
        end

        function dJac = FiniteDifferenceJacobian(dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius)
            %% SIGNATURE
            % dJac = FiniteDifferenceJacobian(dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compute a central-difference reference from perturbed acceleration queries.
            % Retain the raw matrix: never enforce symmetry or remove its trace.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % dPos          Target-frame position [LU].
            % ui32MaxDeg    Active harmonic degree.
            % dCSlmCols     Unnormalized coefficient pairs.
            % dMu           Gravitational parameter [LU^3/s^2].
            % dRadius       Reference radius [LU].
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dJac          Raw acceleration position Jacobian [s^-2].
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EvalExtSphHarmExpInTargetFrame.
            % ----------------------------------------------------------------------------------------------------

            dStep = 1.0e-6 * max(norm(dPos), dRadius);
            dJac = zeros(3, 3);

            for idxAxis = 1:3
                dPerturb = zeros(3, 1);
                dPerturb(idxAxis) = dStep;

                [~, dAccPlus] = EvalExtSphHarmExpInTargetFrame( ...
                    dPos + dPerturb, ui32MaxDeg, dCSlmCols, dMu, dRadius);
                [~, dAccMinus] = EvalExtSphHarmExpInTargetFrame( ...
                    dPos - dPerturb, ui32MaxDeg, dCSlmCols, dMu, dRadius);

                dJac(:, idxAxis) = (dAccPlus - dAccMinus) / (2.0 * dStep);
            end

            % Retain the raw reference: projection could hide a derivative defect.
        end

        function [dAcceleration, dHessian] = EvaluatePolePartials(dPos, ui32Degree, ...
                dCoefficients, dMu, dRadius)
            %% SIGNATURE
            % [dAcceleration, dHessian] = EvaluatePolePartials(dPos, ui32Degree, ...
            % dCoefficients, dMu, dRadius)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Evaluate independent closed-form Legendre limits on the polar axis.
            % Only orders zero and one affect force; order two also affects the Hessian.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % dPos           Nonzero position on the target-frame polar axis [LU].
            % ui32Degree     Active harmonic degree.
            % dCoefficients  Unnormalized coefficient pairs.
            % dMu            Gravitational parameter [LU^3/s^2].
            % dRadius        Reference radius [LU].
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dAcceleration  Polar perturbing acceleration [LU/s^2].
            % dHessian       Polar acceleration position Jacobian [s^-2].
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None; closed-form polar Legendre limits.
            % ----------------------------------------------------------------------------------------------------

            % Independent Legendre limits: only m<=1 affects force and m<=2 the Hessian.
            dAcceleration = zeros(3, 1);
            dHessian = zeros(3);
            dRange = norm(dPos);
            dSign = sign(dPos(3));

            for dDegree = 2:double(ui32Degree)
                ui32Row = dDegree * (dDegree + 1) / 2 - 1;
                dFactor = dMu * dRadius^dDegree / dRange^(dDegree + 2);
                dParity = dSign^dDegree;

                % Zonal (m = 0): axial force and axisymmetric curvature.
                dZonal = dFactor * dCoefficients(ui32Row, 1) * dParity;
                dAcceleration(3) = dAcceleration(3) - dSign * (dDegree + 1) * dZonal;
                dCurvature = (dDegree + 1) * (dDegree + 2) * dZonal / dRange;
                dHessian = dHessian + diag([-0.5, -0.5, 1]) * dCurvature;

                % Tesseral m = 1: transverse force and mixed axial-transverse Hessian terms.
                dTransverse = dFactor * dDegree * (dDegree + 1) / 2 * dSign^(dDegree + 1) * ...
                    dCoefficients(ui32Row + 1, :).';
                dAcceleration(1:2) = dAcceleration(1:2) + dTransverse;
                dMixed = -dSign * (dDegree + 2) * dTransverse / dRange;
                dHessian(1:2, 3) = dHessian(1:2, 3) + dMixed;
                dHessian(3, 1:2) = dHessian(3, 1:2) + dMixed.';

                % Order m = 2: trace-free transverse Hessian block only.
                dSecond = dFactor / dRange * dParity * ...
                    (dDegree - 1) * dDegree * (dDegree + 1) * (dDegree + 2) / 4;
                dPair = dCoefficients(ui32Row + 2, :);
                dHessian(1:2, 1:2) = dHessian(1:2, 1:2) + ...
                    dSecond * [dPair(1), dPair(2); dPair(2), -dPair(1)];
            end
        end

        function [dPotentialPert, dAccPertTB] = EvaluateCanonicalTargetFrame(dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius)
            [dPotentialPert, dAccPertTB] = EvalExtSphHarmExpInTargetFrame( ...
                dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius);
        end

        function dAccPertTB = EvaluateCanonicalAcceleration(dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius)
            [~, dAccPertTB] = testExtSHE_CrossValidation.EvaluateCanonicalTargetFrame( ...
                dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius);
        end

        function dAccPertTB = EvaluateCanonicalAccelerationMex(fcnMex, dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius)
            [~, dAccPertTB] = fcnMex(dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius);
        end

        function sMexInfo = BuildMexTargets()
            %% SIGNATURE
            % sMexInfo = BuildMexTargets()
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Build and cache the gravity MEX interfaces once per MATLAB session.
            % Exercise fixed degree-eight force inputs and fixed/bounded degree-sixteen
            % Jacobian storage; retain runtime active-degree selection.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % None.
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % sMexInfo       Generated function names and temporary build directory.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % MATLAB Coder, SetupSimGears, canonical and legacy gravity evaluators.
            % ----------------------------------------------------------------------------------------------------

            persistent sCachedInfo

            if ~isempty(sCachedInfo)
                sMexInfo = sCachedInfo;
                return;
            end

            charRepoRoot = testExtSHE_CrossValidation.GetRepoRoot();
            addpath(fullfile(charRepoRoot, 'matlab'));
            SetupSimGears();
            testExtSHE_CrossValidation.EnsureLegacySHEOnPath();

            cfg = coder.config('mex');
            charBuildDir = fullfile(tempdir, 'simgears_she_codegen');
            if ~exist(charBuildDir, 'dir')
                mkdir(charBuildDir);
            end
            addpath(charBuildDir);

            ui32MaxDeg = uint32(8);
            ui32Count = testExtSHE_CrossValidation.GetCoeffCount(ui32MaxDeg);
            dSampleRadius = 7000.0;
            dSampleMu = 3.986004418e5;

            codegen('-config', cfg, '-d', charBuildDir, 'EvalExtSphHarmExpInTargetFrame', ...
                '-args', {[dSampleRadius; 0.0; 0.0], ui32MaxDeg, zeros(double(ui32Count), 2), dSampleMu, dSampleRadius});
            codegen('-config', cfg, '-d', charBuildDir, 'EvalExtSphericalHarmExpCore', ...
                '-args', {dSampleRadius, 0.0, 0.0, ui32MaxDeg, zeros(double(ui32Count), 2), dSampleMu, dSampleRadius});
            codegen('-config', cfg, '-d', charBuildDir, 'EvalJac_ExtSphHarmExpInTargetFrame', ...
                '-args', {[dSampleRadius; 0; 0], uint32(16), ...
                          zeros(double(testExtSHE_CrossValidation.GetCoeffCount(uint32(16))), 2), ...
                          dSampleMu, dSampleRadius});

            % Match the bounded optional coefficient slice used by the truth RHS.
            codegen('-config', cfg, '-d', charBuildDir, '-o', 'BoundedSHJac_mex', ...
                'EvalJac_ExtSphHarmExpInTargetFrame', '-args', ...
                {[dSampleRadius; 0; 0], uint32(16), ...
                 coder.typeof(0, ...
                     [double(testExtSHE_CrossValidation.GetCoeffCount(uint32(16))), 2], ...
                     [true, false]), ...
                 dSampleMu, dSampleRadius});
            codegen('-config', cfg, '-d', charBuildDir, 'SphericalHarmonics', ...
                '-args', {[dSampleRadius, 0.0, 0.0], double(ui32MaxDeg), double(ui32MaxDeg), dSampleMu, dSampleRadius, ...
                          zeros(double(ui32MaxDeg) + 1), zeros(double(ui32MaxDeg) + 1)});

            sCachedInfo = struct( ...
                'canonical_target', 'EvalExtSphHarmExpInTargetFrame_mex', ...
                'canonical_core', 'EvalExtSphericalHarmExpCore_mex', ...
                'canonical_jacobian', 'EvalJac_ExtSphHarmExpInTargetFrame_mex', ...
                'bounded_jacobian', 'BoundedSHJac_mex', ...
                'legacy', 'SphericalHarmonics_mex', ...
                'build_dir', charBuildDir);

            sMexInfo = sCachedInfo;
        end
    end

    methods (Test)
        function testBasisMatchesLegendreReference(testCase)
            %% SIGNATURE
            % testBasisMatchesLegendreReference(testCase)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check every active basis column through degree ten against MATLAB
            % Legendre values and direct longitude trigonometry. Keep internal
            % lookup builders private; exercise their public consumer instead.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase    MATLAB unit-test instance.
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Verify potential, radial derivative and longitude derivative.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 11-10-2026  Pietro Califano  Replace direct lookup-table access with a basis oracle.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % BuildExtSphHarmExpBasis, legendre.
            % ----------------------------------------------------------------------------------------------------

            ui32MaxDegree = uint32(10);
            dLatitude = 0.43;
            dLongitude = -1.27;
            dRadius = 2.0;

            [dPotentialBasis, dGradientBasis] = BuildExtSphHarmExpBasis( ...
                dRadius, dLatitude, dLongitude, ui32MaxDegree, 1.0, 1.0);
            testCase.verifySize(dPotentialBasis, [1, 117]);
            testCase.verifySize(dGradientBasis, [3, 117]);

            % Remove MATLAB's Condon-Shortley phase to match repository coefficients.
            dBasisColumn = 1;
            for dDegree = 2:double(ui32MaxDegree)
                dLegendreValues = legendre(dDegree, sin(dLatitude), 'unnorm');

                for dOrder = 0:dDegree
                    dAmplitude = (-1)^dOrder * dLegendreValues(dOrder + 1) / ...
                        dRadius^(dDegree + 1);
                    dAngle = dOrder * dLongitude;
                    dLongitudeFactors = [cos(dAngle), sin(dAngle)];
                    dLongitudeDerivatives = dOrder * [-sin(dAngle), cos(dAngle)];

                    % The zonal term has only a cosine coefficient; other orders have both.
                    for dCoefficientColumn = 1:(1 + (dOrder > 0))
                        dExpectedPotential = dAmplitude * dLongitudeFactors(dCoefficientColumn);
                        dExpectedGradient = [-(dDegree + 1) / dRadius * dExpectedPotential; ...
                            dAmplitude * dLongitudeDerivatives(dCoefficientColumn)];

                        testCase.verifyEqual(dPotentialBasis(dBasisColumn), dExpectedPotential, ...
                            'RelTol', 1e-12, 'AbsTol', 1e-13);
                        testCase.verifyEqual(dGradientBasis([1, 3], dBasisColumn), dExpectedGradient, ...
                            'RelTol', 1e-12, 'AbsTol', 1e-13);
                        dBasisColumn = dBasisColumn + 1;
                    end
                end
            end
        end

        function testJ2OnlyAcceleration(testCase, dTestPosition)
            ui32MaxDeg = uint32(2);
            dCSlmCols = zeros(double(testExtSHE_CrossValidation.GetCoeffCount(ui32MaxDeg)), 2);
            dCSlmCols(2, 1) = -testCase.dJ2Earth; % Unnormalized C20

            [~, dAccSHE] = testExtSHE_CrossValidation.EvaluateCanonicalTargetFrame( ...
                dTestPosition, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            dR = norm(dTestPosition);
            dR2 = dR^2;
            dZ2 = dTestPosition(3)^2;
            dFactor = 1.5 * testCase.dJ2Earth * testCase.dMuEarth * testCase.dRadiusEarth^2 / dR^5;

            dAccAnalytic = zeros(3, 1);
            dAccAnalytic(1) = -dFactor * dTestPosition(1) * (1 - 5 * dZ2 / dR2);
            dAccAnalytic(2) = -dFactor * dTestPosition(2) * (1 - 5 * dZ2 / dR2);
            dAccAnalytic(3) = -dFactor * dTestPosition(3) * (3 - 5 * dZ2 / dR2);

            testCase.verifyEqual(dAccSHE, dAccAnalytic, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testAccelerationMatchesLegacyMATLAB(testCase, dTestPosition)
            %% SIGNATURE
            % testAccelerationMatchesLegacyMATLAB(testCase, dTestPosition)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare canonical and legacy perturbing acceleration away from the poles.
            % Use independent polar limits where the legacy evaluator loses transverse force.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % dTestPosition  Parameterized target-frame position [km].
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EvaluateCanonicalTargetFrame, EvaluateLegacyPerturbation, EvaluatePolePartials.
            % ----------------------------------------------------------------------------------------------------

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 42);
            dCmat(3, 1) = -testCase.dJ2Earth / sqrt(5);

            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            [~, dAccExt] = testExtSHE_CrossValidation.EvaluateCanonicalTargetFrame( ...
                dTestPosition, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            dAccLegacy = testExtSHE_CrossValidation.EvaluateLegacyPerturbation( ...
                dTestPosition, ui32MaxDeg, testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);

            if norm(dTestPosition(1:2)) <= 1e-9 * norm(dTestPosition)
                % The legacy spherical-coordinate evaluator drops transverse polar force.
                dAccLegacy = testExtSHE_CrossValidation.EvaluatePolePartials( ...
                    dTestPosition, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            end

            testCase.verifyEqual(dAccExt, dAccLegacy, 'RelTol', 3e-3, 'AbsTol', 1e-13);
        end

        function testCorePotentialMatchesLegacyMATLAB(testCase, dTestPosition)
            %% SIGNATURE
            % testCorePotentialMatchesLegacyMATLAB(testCase, dTestPosition)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare the spherical core potential with Cartesian and legacy evaluators.
            % Retain near-pole latitude information with atan2 for the spherical query.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % dTestPosition  Parameterized target-frame position [km].
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EvalExtSphericalHarmExpCore, EvaluateCanonicalTargetFrame, SphericalHarmonics.
            % ----------------------------------------------------------------------------------------------------

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 5e-7, 7);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            dPosNorm = norm(dTestPosition);
            % atan2 retains the transverse direction when z/r rounds to unity near a pole.
            dLat = atan2(dTestPosition(3), norm(dTestPosition(1:2)));
            dLong = atan2(dTestPosition(2), dTestPosition(1));

            [dPotentialCore, dGradCore, dPlmCore] = EvalExtSphericalHarmExpCore(dPosNorm, dLat, dLong, ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            [dPotentialFromTarget, ~] = testExtSHE_CrossValidation.EvaluateCanonicalTargetFrame( ...
                dTestPosition, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            [dULegacy, ~] = SphericalHarmonics(dTestPosition.', double(ui32MaxDeg), ...
                double(ui32MaxDeg), testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);
            dPotentialLegacyPert = dULegacy - testCase.dMuEarth / dPosNorm;

            testCase.verifyEqual(dPotentialCore, dPotentialFromTarget, 'RelTol', 1e-13, 'AbsTol', 1e-15);
            testCase.verifyEqual(dPotentialCore, dPotentialLegacyPert, 'RelTol', 3e-3, 'AbsTol', 1e-13);
            testCase.verifyGreaterThan(norm(dGradCore), 0.0);
            testCase.verifyGreaterThan(numel(dPlmCore), 0);
        end

        function testEvalJacobianMatchesFiniteDifference(testCase)
            ui32MaxDeg = uint32(6);
            dPos = [4000; 3000; 4500];

            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 9);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            dJac = EvalJac_ExtSphHarmExpInTargetFrame(dPos, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            dJacFD = testExtSHE_CrossValidation.FiniteDifferenceJacobian( ...
                dPos, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            testCase.verifyEqual(dJac, dJac.', 'AbsTol', 1e-15);
            testCase.verifyEqual(trace(dJac), 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac, dJacFD, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testEvalJacobianRejectsZeroPosition(testCase)
            ui32MaxDeg = uint32(4);
            dCSlmCols = zeros(double(testExtSHE_CrossValidation.GetCoeffCount(ui32MaxDeg)), 2);

            testCase.verifyError(@() EvalJac_ExtSphHarmExpInTargetFrame([0; 0; 0], ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth), ...
                'EvalJac_ExtSphHarmExpInTargetFrame:ZeroPosition');
        end

        function testDegree16RawDerivativesAndUnits(testCase, dTestPosition)
            %% SIGNATURE
            % testDegree16RawDerivativesAndUnits(testCase, dTestPosition)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare the analytical degree-sixteen Jacobian with raw central differences.
            % Check symmetry and trace independently, then verify metre/kilometre scaling.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % dTestPosition  Parameterized target-frame position [km].
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EvalJac_ExtSphHarmExpInTargetFrame, FiniteDifferenceJacobian, EvalExtSphHarmExpInTargetFrame.
            % ----------------------------------------------------------------------------------------------------

            % Validate the full active degree with unprojected independent numerical references.
            ui32Degree = uint32(16);
            [dCosine, dSine] = testCase.BuildRandomNormalizedCoeffMatrices(ui32Degree, 1e-6, 27);
            dCoefficients = testCase.ConvertMatrixToColPairs(dCosine, dSine, ui32Degree);

            dJacobian = EvalJac_ExtSphHarmExpInTargetFrame(dTestPosition, ui32Degree, ...
                dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
            dReference = testCase.FiniteDifferenceJacobian(dTestPosition, ui32Degree, ...
                dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
            dJacobianNorm = norm(dJacobian, 'fro');

            % An exterior potential has a symmetric, trace-free Hessian without any projection.
            testCase.verifyLessThan(norm(dJacobian - dReference, 'fro') / dJacobianNorm, 2e-7);
            testCase.verifyLessThan(norm(dJacobian - dJacobian.', 'fro') / dJacobianNorm, 2e-13);
            testCase.verifyLessThan(abs(trace(dJacobian)) / dJacobianNorm, 2e-13);

            % Kilometre-to-metre scaling: force scales by 1e3, the Jacobian is invariant.
            [~, dAcceleration] = EvalExtSphHarmExpInTargetFrame(dTestPosition, ui32Degree, ...
                dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
            [~, dMetreAcceleration] = EvalExtSphHarmExpInTargetFrame(1e3 * dTestPosition, ui32Degree, ...
                dCoefficients, 1e9 * testCase.dMuEarth, 1e3 * testCase.dRadiusEarth);
            dMetreJacobian = EvalJac_ExtSphHarmExpInTargetFrame(1e3 * dTestPosition, ui32Degree, ...
                dCoefficients, 1e9 * testCase.dMuEarth, 1e3 * testCase.dRadiusEarth);

            testCase.verifyEqual(dMetreAcceleration, 1e3 * dAcceleration, 'RelTol', 2e-12, 'AbsTol', 1e-20);
            testCase.verifyEqual(dMetreJacobian, dJacobian, 'RelTol', 2e-12, 'AbsTol', 1e-24);
        end

        function testDegree16PoleLimits(testCase)
            %% SIGNATURE
            % testDegree16PoleLimits(testCase)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare degree-sixteen acceleration and Jacobian with independent exact
            % north/south pole limits, including transverse and mixed components.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EvaluatePolePartials, EvalExtSphHarmExpInTargetFrame, EvalJac_ExtSphHarmExpInTargetFrame.
            % ----------------------------------------------------------------------------------------------------

            % Closed-form north/south limits independently exercise m=0,1,2 and vanishing higher orders.
            ui32Degree = uint32(16);
            [dCosine, dSine] = testCase.BuildRandomNormalizedCoeffMatrices(ui32Degree, 1e-6, 31);
            dCoefficients = testCase.ConvertMatrixToColPairs(dCosine, dSine, ui32Degree);

            for dSign = [-1, 1]
                dPosition = [0; 0; dSign * 6800];
                [dExpected, dExpectedJac] = testCase.EvaluatePolePartials(dPosition, ui32Degree, ...
                    dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);

                [~, dAcceleration] = EvalExtSphHarmExpInTargetFrame(dPosition, ui32Degree, ...
                    dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                dJacobian = EvalJac_ExtSphHarmExpInTargetFrame(dPosition, ui32Degree, ...
                    dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);

                testCase.verifyEqual(dAcceleration, dExpected, 'RelTol', 2e-12, 'AbsTol', 1e-20);
                testCase.verifyEqual(dJacobian, dExpectedJac, 'RelTol', 2e-12, 'AbsTol', 1e-24);
            end
        end

        function testSectorialBasisLatitudeDerivative(testCase)
            %% SIGNATURE
            % testSectorialBasisLatitudeDerivative(testCase)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Isolate sectorial coefficients to detect incorrect P(n,n+1) indexing.
            % Compare Cartesian acceleration and a raw finite difference of the potential.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EvalExtSphericalHarmExpCore, MapSphericalGradientToCartesian, EvalExtSphHarmExpInTargetFrame.
            % ----------------------------------------------------------------------------------------------------

            % Regression: the sectorial latitude factor must use P(n,n+1) = 0, not the next degree's P(n+1,0).
            % Sectorial-only fields isolate that term in the spherical basis used by the core and the fitter.
            ui32Degree = uint32(6);
            dCoefficients = zeros(double(testCase.GetCoeffCount(ui32Degree)), 2);
            rng(43);
            for dDegree = 2:double(ui32Degree)
                dCoefficients(dDegree * (dDegree + 1) / 2 - 1 + dDegree, :) = 1e-5 * randn(1, 2) / dDegree^2;
            end

            for idSample = 1:8
                dLatitude = (rand - 0.5) * 0.9 * pi;
                dLongitude = (rand - 0.5) * 2 * pi;
                dRange = 6800 + 2000 * rand;
                dPosition = dRange * [cos(dLatitude) * cos(dLongitude); ...
                                      cos(dLatitude) * sin(dLongitude); ...
                                      sin(dLatitude)];

                % Spherical core gradient mapped to Cartesian must match the independent Cartesian recurrence.
                [~, dGradCore] = EvalExtSphericalHarmExpCore(dRange, dLatitude, dLongitude, ui32Degree, ...
                    dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                dAccCore = MapSphericalGradientToCartesian(dPosition, dGradCore);
                [~, dAccCartesian] = EvalExtSphHarmExpInTargetFrame(dPosition, ui32Degree, ...
                    dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                testCase.verifyEqual(dAccCore, dAccCartesian, 'RelTol', 1e-10, 'AbsTol', 1e-22);

                % The latitude derivative must also match a central difference of the core potential.
                dStep = 1e-6;
                dPotentialPlus = EvalExtSphericalHarmExpCore(dRange, dLatitude + dStep, dLongitude, ...
                    ui32Degree, dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                dPotentialMinus = EvalExtSphericalHarmExpCore(dRange, dLatitude - dStep, dLongitude, ...
                    ui32Degree, dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                testCase.verifyEqual(dGradCore(2), (dPotentialPlus - dPotentialMinus) / (2 * dStep), ...
                    'RelTol', 1e-7, 'AbsTol', 1e-18);
            end
        end

        function testJacobianCodegenMatchesMATLAB(testCase, dTestPosition)
            %% SIGNATURE
            % testJacobianCodegenMatchesMATLAB(testCase, dTestPosition)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare fixed-capacity generated and source Jacobians while changing
            % the active degree through zero, one, two and sixteen.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % dTestPosition  Parameterized target-frame position [km].
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % BuildMexTargets, EvalJac_ExtSphHarmExpInTargetFrame, MATLAB Coder.
            % ----------------------------------------------------------------------------------------------------

            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1);
            ui32Degree = uint32(16);
            [dCosine, dSine] = testCase.BuildRandomNormalizedCoeffMatrices(ui32Degree, 1e-6, 33);
            dCoefficients = testCase.ConvertMatrixToColPairs(dCosine, dSine, ui32Degree);

            strMex = testCase.BuildMexTargets();
            fnMex = str2func(strMex.canonical_jacobian);

            for ui32ActiveDegree = [uint32(0), uint32(1), uint32(2), ui32Degree]
                dExpected = EvalJac_ExtSphHarmExpInTargetFrame(dTestPosition, ui32ActiveDegree, ...
                    dCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                dActual = fnMex(dTestPosition, ui32ActiveDegree, dCoefficients, ...
                    testCase.dMuEarth, testCase.dRadiusEarth);
                testCase.verifyEqual(dActual, dExpected, 'RelTol', 2e-12, 'AbsTol', 1e-24);
            end
        end

        function testBoundedCoefficientStorageCodegen(testCase)
            %% SIGNATURE
            % testBoundedCoefficientStorageCodegen(testCase)
            % ----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Exercise the bounded coefficient slices passed by truth dynamics.
            % Compare source and generated Jacobians for empty, degree-two and degree-sixteen inputs.
            % ----------------------------------------------------------------------------------------------------
            %% INPUT
            % testCase       MATLAB unit-test instance.
            % ----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; report assertion failures through the test framework.
            % ----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 10-10-2026  Pietro Califano  Document the gravity validation contract.
            % ----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % BuildMexTargets, EvalJac_ExtSphHarmExpInTargetFrame, MATLAB Coder.
            % ----------------------------------------------------------------------------------------------------

            % The real truth RHS passes a bounded optional coefficient slice, not a constant row count.
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1);
            [dCosine, dSine] = testCase.BuildRandomNormalizedCoeffMatrices(uint32(16), 1e-6, 35);
            dCoefficients = testCase.ConvertMatrixToColPairs(dCosine, dSine, uint32(16));

            strMex = testCase.BuildMexTargets();
            fnMex = str2func(strMex.bounded_jacobian);
            dPosition = [4000; 3000; 4500];

            for ui32Degree = [uint32(0), uint32(2), uint32(16)]
                dRows = 0;
                if ui32Degree > 0
                    dRows = double(testCase.GetCoeffCount(ui32Degree));
                end
                dActiveCoefficients = dCoefficients(1:dRows, :);

                dExpected = EvalJac_ExtSphHarmExpInTargetFrame(dPosition, ui32Degree, ...
                    dActiveCoefficients, testCase.dMuEarth, testCase.dRadiusEarth);
                dActual = fnMex(dPosition, ui32Degree, dActiveCoefficients, ...
                    testCase.dMuEarth, testCase.dRadiusEarth);
                testCase.verifyEqual(dActual, dExpected, 'RelTol', 2e-12, 'AbsTol', 1e-24);
            end
        end

        function testEvalJacobianRejectsInsufficientCoefficients(testCase)
            ui32MaxDeg = uint32(4);
            dCSlmCols = zeros(3, 2);
            dPos = [4000; 3000; 4500];

            testCase.verifyError(@() EvalJac_ExtSphHarmExpInTargetFrame(dPos, ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth), ...
                'EvalJac_ExtSphHarmExpInTargetFrame:InsufficientCoefficients');
        end

        function testTargetFrameEvaluatorCodegenMatchesMATLAB(testCase, dTestPosition)
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 21);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            sMexInfo = testExtSHE_CrossValidation.BuildMexTargets();
            fcnMex = str2func(sMexInfo.canonical_target);

            [dPotentialMatlab, dAccMatlab] = testExtSHE_CrossValidation.EvaluateCanonicalTargetFrame( ...
                dTestPosition, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            [dPotentialMex, dAccMex] = fcnMex(dTestPosition, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);

            testCase.verifyEqual(dPotentialMex, dPotentialMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dAccMex, dAccMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testCoreEvaluatorCodegenMatchesMATLAB(testCase, dTestPosition)
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 5e-7, 11);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            dPosNorm = norm(dTestPosition);
            dLat = asin(dTestPosition(3) / dPosNorm);
            dLong = atan2(dTestPosition(2), dTestPosition(1));

            sMexInfo = testExtSHE_CrossValidation.BuildMexTargets();
            fcnMex = str2func(sMexInfo.canonical_core);

            [dPotentialMatlab, dGradMatlab, dPlmMatlab] = EvalExtSphericalHarmExpCore( ...
                dPosNorm, dLat, dLong, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            [dPotentialMex, dGradMex, dPlmMex] = fcnMex( ...
                dPosNorm, dLat, dLong, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            testCase.verifyEqual(dPotentialMex, dPotentialMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dGradMex, dGradMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dPlmMex, dPlmMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testLegacyCodegenMatchesMATLAB(testCase, dTestPosition)
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 13);

            sMexInfo = testExtSHE_CrossValidation.BuildMexTargets();
            fcnMex = str2func(sMexInfo.legacy);

            [dUMatlab, dAccMatlab] = SphericalHarmonics(dTestPosition.', double(ui32MaxDeg), ...
                double(ui32MaxDeg), testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);
            [dUMex, dAccMex] = fcnMex(dTestPosition.', double(ui32MaxDeg), double(ui32MaxDeg), ...
                testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);

            testCase.verifyEqual(dUMex, dUMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dAccMex, dAccMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testPerformanceComparison(testCase)
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');

            ui32MaxDeg = uint32(8);
            dPos = [4000; 3000; 4500];

            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 17);
            dCmat(3, 1) = -testCase.dJ2Earth / sqrt(5);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            sMexInfo = testExtSHE_CrossValidation.BuildMexTargets();
            fcnExtMex = str2func(sMexInfo.canonical_target);
            fcnLegacyMex = str2func(sMexInfo.legacy);

            fcnExtMatlab = @() testExtSHE_CrossValidation.EvaluateCanonicalAcceleration(dPos, ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            fcnExtMexTimed = @() testExtSHE_CrossValidation.EvaluateCanonicalAccelerationMex(fcnExtMex, dPos, ...
                ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            fcnLegacyMatlab = @() SphericalHarmonics(dPos.', double(ui32MaxDeg), double(ui32MaxDeg), ...
                testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);
            fcnLegacyMexTimed = @() fcnLegacyMex(dPos.', double(ui32MaxDeg), double(ui32MaxDeg), ...
                testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);

            fcnExtMatlab();
            fcnExtMexTimed();
            fcnLegacyMatlab();
            fcnLegacyMexTimed();

            dTimeExtMatlab = timeit(fcnExtMatlab);
            dTimeExtMex = timeit(fcnExtMexTimed);
            dTimeLegacyMatlab = timeit(fcnLegacyMatlab);
            dTimeLegacyMex = timeit(fcnLegacyMexTimed);

            fprintf('\n--- SHE timing comparison (degree %d) ---\n', ui32MaxDeg);
            fprintf('EvalExtSphHarmExpInTargetFrame MATLAB  %.3g us/eval\n', 1e6 * dTimeExtMatlab);
            fprintf('EvalExtSphHarmExpInTargetFrame MEX     %.3g us/eval\n', 1e6 * dTimeExtMex);
            fprintf('SphericalHarmonics MATLAB  %.3g us/eval\n', 1e6 * dTimeLegacyMatlab);
            fprintf('SphericalHarmonics MEX     %.3g us/eval\n', 1e6 * dTimeLegacyMex);
            fprintf('Canonical SHE MATLAB vs legacy MATLAB speedup   %.2fx\n', dTimeLegacyMatlab / dTimeExtMatlab);
            fprintf('Canonical SHE MEX vs canonical SHE MATLAB speedup %.2fx\n', dTimeExtMatlab / dTimeExtMex);
        end
    end
end
