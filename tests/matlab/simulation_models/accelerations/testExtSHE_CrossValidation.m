classdef testExtSHE_CrossValidation < matlab.unittest.TestCase
    properties (Constant)
        dMuEarth = 398600.4418;   % [km^3/s^2]
        dRadiusEarth = 6378.137;  % [km]
        dJ2Earth = 1.08263e-3;    % [-]
    end

    properties (TestParameter)
        dTestPosition = struct( ...
            'equatorial_LEO', [6800; 0; 0], ...
            'polar_LEO', [0; 0; 6800], ...
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

        function dParams = BuildLegacyParamStruct(dCSlmCols, dMu, dRadius)
            dParams = struct( ...
                'mu', dMu, ...
                'meanRbody', dRadius, ...
                'Clm_LUT', dCSlmCols(:, 1), ...
                'Slm_LUT', dCSlmCols(:, 2));
        end

        function dJac = FiniteDifferenceJacobian(dPos, ui32MaxDeg, dCSlmCols, dMu, dRadius)
            dStep = 1.0e-6 * max(norm(dPos), dRadius);
            dJac = zeros(3, 3);

            for idxAxis = 1:3
                dPerturb = zeros(3, 1);
                dPerturb(idxAxis) = dStep;

                dAccPlus = ExtSHE_AccTB(dPos + dPerturb, ui32MaxDeg, dCSlmCols, dMu, dRadius);
                dAccMinus = ExtSHE_AccTB(dPos - dPerturb, ui32MaxDeg, dCSlmCols, dMu, dRadius);

                dJac(:, idxAxis) = (dAccPlus - dAccMinus) / (2.0 * dStep);
            end

            dJac = 0.5 * (dJac + dJac.');
            dJac = dJac - trace(dJac) / 3.0 * eye(3);
        end

        function sMexInfo = BuildMexTargets()
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
            dParams = struct( ...
                'mu', 0.0, ...
                'meanRbody', 0.0, ...
                'Clm_LUT', zeros(double(ui32Count), 1), ...
                'Slm_LUT', zeros(double(ui32Count), 1));

            codegen('-config', cfg, '-d', charBuildDir, 'ExtSHE_AccTB', ...
                '-args', {zeros(3, 1), ui32MaxDeg, zeros(double(ui32Count), 2), 0.0, 0.0});
            codegen('-config', cfg, '-d', charBuildDir, 'ExtSHE_GradU', ...
                '-args', {0.0, 0.0, 0.0, uint8(ui32MaxDeg), dParams});
            codegen('-config', cfg, '-d', charBuildDir, 'SphericalHarmonics', ...
                '-args', {zeros(1, 3), double(ui32MaxDeg), double(ui32MaxDeg), 0.0, 0.0, ...
                          zeros(double(ui32MaxDeg) + 1), zeros(double(ui32MaxDeg) + 1)});

            sCachedInfo = struct( ...
                'extshe_acc', 'ExtSHE_AccTB_mex', ...
                'extshe_gradu', 'ExtSHE_GradU_mex', ...
                'legacy', 'SphericalHarmonics_mex', ...
                'build_dir', charBuildDir);

            sMexInfo = sCachedInfo;
        end
    end

    methods (Test)
        function testBuildSHETrigLUTMatchesDirectEvaluation(testCase)
            ui32MaxDeg = uint32(10);
            dLat = 0.43;
            dLong = -1.27;

            dLUT = buildSHEtrigLUT(ui32MaxDeg, dLat, dLong);
            testCase.verifySize(dLUT, [double(ui32MaxDeg) + 2, 3]);

            for m = 0:(double(ui32MaxDeg) + 1)
                idx = m + 1;
                testCase.verifyEqual(dLUT(idx, 1), sin(m * dLong), 'AbsTol', 1e-14);
                testCase.verifyEqual(dLUT(idx, 2), cos(m * dLong), 'AbsTol', 1e-14);
                testCase.verifyEqual(dLUT(idx, 3), m * tan(dLat), 'AbsTol', 1e-14);
            end
        end

        function testJ2OnlyAcceleration(testCase, dTestPosition)
            ui32MaxDeg = uint32(2);
            dCSlmCols = zeros(double(testExtSHE_CrossValidation.GetCoeffCount(ui32MaxDeg)), 2);
            dCSlmCols(2, 1) = -testCase.dJ2Earth; % Unnormalized C20

            dAccSHE = ExtSHE_AccTB(dTestPosition, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);

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
            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 42);
            dCmat(3, 1) = -testCase.dJ2Earth / sqrt(5);

            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            dAccExt = ExtSHE_AccTB(dTestPosition, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);
            dAccLegacy = testExtSHE_CrossValidation.EvaluateLegacyPerturbation( ...
                dTestPosition, ui32MaxDeg, testCase.dMuEarth, testCase.dRadiusEarth, dCmat, dSmat);

            testCase.verifyEqual(dAccExt, dAccLegacy, 'RelTol', 3e-3, 'AbsTol', 1e-13);
        end

        function testStandaloneGradientMatchesSharedCore(testCase, dTestPosition)
            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 5e-7, 7);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            dPosNorm = norm(dTestPosition);
            dLat = asin(dTestPosition(3) / dPosNorm);
            dLong = atan2(dTestPosition(2), dTestPosition(1));
            dParams = testExtSHE_CrossValidation.BuildLegacyParamStruct( ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            [dGradLegacy, dPlmLegacy] = ExtSHE_GradU(dPosNorm, dLat, dLong, uint8(ui32MaxDeg), dParams);
            [dGradCore, dPlmCore] = ExtSHE_GradUCore(dPosNorm, dLat, dLong, ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            testCase.verifyEqual(dGradLegacy, dGradCore, 'RelTol', 1e-13, 'AbsTol', 1e-15);
            testCase.verifyEqual(dPlmLegacy, dPlmCore, 'RelTol', 1e-13, 'AbsTol', 1e-15);
        end

        function testEvalJacobianMatchesFiniteDifference(testCase)
            ui32MaxDeg = uint32(6);
            dPos = [4000; 3000; 4500];

            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 9);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            dJac = EvalJac_ExtSphericalHarmExp(dPos, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);
            dJacFD = testExtSHE_CrossValidation.FiniteDifferenceJacobian( ...
                dPos, ui32MaxDeg, dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            testCase.verifyEqual(dJac, dJac.', 'AbsTol', 1e-15);
            testCase.verifyEqual(trace(dJac), 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac, dJacFD, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testEvalJacobianRejectsZeroPosition(testCase)
            ui32MaxDeg = uint32(4);
            dCSlmCols = zeros(double(testExtSHE_CrossValidation.GetCoeffCount(ui32MaxDeg)), 2);

            testCase.verifyError(@() EvalJac_ExtSphericalHarmExp([0; 0; 0], ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth), ...
                'EvalJac_ExtSphericalHarmExp:ZeroPosition');
        end

        function testEvalJacobianRejectsInsufficientCoefficients(testCase)
            ui32MaxDeg = uint32(4);
            dCSlmCols = zeros(3, 2);
            dPos = [4000; 3000; 4500];

            testCase.verifyError(@() EvalJac_ExtSphericalHarmExp(dPos, ui32MaxDeg, ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth), ...
                'EvalJac_ExtSphericalHarmExp:InsufficientCoefficients');
        end

        function testExtSHEAccCodegenMatchesMATLAB(testCase, dTestPosition)
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 1e-6, 21);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);

            sMexInfo = testExtSHE_CrossValidation.BuildMexTargets();
            fcnMex = str2func(sMexInfo.extshe_acc);

            dAccMatlab = ExtSHE_AccTB(dTestPosition, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);
            dAccMex = fcnMex(dTestPosition, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);

            testCase.verifyEqual(dAccMex, dAccMatlab, 'RelTol', 1e-12, 'AbsTol', 1e-15);
        end

        function testExtSHEGradUCodegenMatchesMATLAB(testCase, dTestPosition)
            testCase.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');

            ui32MaxDeg = uint32(8);
            [dCmat, dSmat] = testExtSHE_CrossValidation.BuildRandomNormalizedCoeffMatrices(ui32MaxDeg, 5e-7, 11);
            dCSlmCols = testExtSHE_CrossValidation.ConvertMatrixToColPairs(dCmat, dSmat, ui32MaxDeg);
            dParams = testExtSHE_CrossValidation.BuildLegacyParamStruct( ...
                dCSlmCols, testCase.dMuEarth, testCase.dRadiusEarth);

            dPosNorm = norm(dTestPosition);
            dLat = asin(dTestPosition(3) / dPosNorm);
            dLong = atan2(dTestPosition(2), dTestPosition(1));

            sMexInfo = testExtSHE_CrossValidation.BuildMexTargets();
            fcnMex = str2func(sMexInfo.extshe_gradu);

            [dGradMatlab, dPlmMatlab] = ExtSHE_GradU(dPosNorm, dLat, dLong, uint8(ui32MaxDeg), dParams);
            [dGradMex, dPlmMex] = fcnMex(dPosNorm, dLat, dLong, uint8(ui32MaxDeg), dParams);

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
            fcnExtMex = str2func(sMexInfo.extshe_acc);
            fcnLegacyMex = str2func(sMexInfo.legacy);

            fcnExtMatlab = @() ExtSHE_AccTB(dPos, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);
            fcnExtMexTimed = @() fcnExtMex(dPos, ui32MaxDeg, dCSlmCols, ...
                testCase.dMuEarth, testCase.dRadiusEarth);
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
            fprintf('ExtSHE_AccTB MATLAB        %.3g us/eval\n', 1e6 * dTimeExtMatlab);
            fprintf('ExtSHE_AccTB MEX           %.3g us/eval\n', 1e6 * dTimeExtMex);
            fprintf('SphericalHarmonics MATLAB  %.3g us/eval\n', 1e6 * dTimeLegacyMatlab);
            fprintf('SphericalHarmonics MEX     %.3g us/eval\n', 1e6 * dTimeLegacyMex);
            fprintf('ExtSHE MATLAB vs legacy MATLAB speedup   %.2fx\n', dTimeLegacyMatlab / dTimeExtMatlab);
            fprintf('ExtSHE MEX vs ExtSHE MATLAB speedup      %.2fx\n', dTimeExtMatlab / dTimeExtMex);
        end
    end
end
