classdef testAttitudePointingGenerator < matlab.unittest.TestCase
    %% DESCRIPTION
    % Verify the public pointing-generator geometry without opening figures
    % or depending on scenario samplers and external datasets.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 22-07-2026    Pietro Califano, Codex     Replace the interactive script with deterministic headless tests.
    % 22-07-2026    Pietro Califano, Codex     Cover the zero-displacement public-method contract.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testDefaultPointingIsHeadlessAndRightHanded(testCase)
            dCameraPosition_Frame = [10.0; 0.0; 0.0];
            dTargetPosition_Frame = zeros(3, 1);
            dSunPosition_Frame = [0.0; 10.0; 2.0];
            ui32FigureCountBefore = uint32(numel(findall(groot, 'Type', 'figure')));

            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);
            [~, dOutRot3, dDCM_FrameFromPose, dOffPointingAngles] = objGenerator.pointToTarget();

            dExpectedBoresight_Frame = (dTargetPosition_Frame - dCameraPosition_Frame) ./ ...
                norm(dTargetPosition_Frame - dCameraPosition_Frame);
            testCase.verifyEqual(dDCM_FrameFromPose(:, 3), dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dOutRot3, dDCM_FrameFromPose, 'AbsTol', 0.0);
            testCase.verifyEqual(dOffPointingAngles, 0.0, 'AbsTol', 0.0);
            testCase.VerifyProperDCMs_(dDCM_FrameFromPose);
            testCase.verifyEqual(uint32(numel(findall(groot, 'Type', 'figure'))), ui32FigureCountBefore);
        end

        function testConstraintModesPreservePointingTriad(testCase)
            dCameraPosition_Frame = [10.0; 0.0; 0.0];
            dTargetPosition_Frame = zeros(3, 1);
            dSunPosition_Frame = [0.0; 10.0; 2.0];
            dVelocity_Frame = [0.0; 2.0; 1.0];
            dAuxiliaryAxis_Frame = [0.0; 2.0; 1.0];
            dExpectedBoresight_Frame = (dTargetPosition_Frame - dCameraPosition_Frame) ./ ...
                norm(dTargetPosition_Frame - dCameraPosition_Frame);

            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);
            [~, ~, dDCMsun] = objGenerator.pointToTarget( ...
                'enumConstraintType', "YorthogonalSun");
            [~, ~, dDCMlvlh] = objGenerator.pointToTarget( ...
                'dVelocity_Frame', dVelocity_Frame, ...
                'enumConstraintType', "trackLVLH");
            [~, ~, dDCMaux] = objGenerator.pointToTarget( ...
                'dAuxiliaryAxis', dAuxiliaryAxis_Frame, ...
                'enumConstraintType', "auxiliaryAxis");

            dCameraFromSun_Frame = dSunPosition_Frame - dCameraPosition_Frame;
            testCase.verifyEqual(dDCMsun(:, 3), dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dot(dDCMsun(:, 2), dCameraFromSun_Frame), 0.0, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dDCMlvlh(:, 3), dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dot(dDCMlvlh(:, 2), dVelocity_Frame), 0.0, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dDCMaux(:, 3), dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dot(dDCMaux(:, 2), dAuxiliaryAxis_Frame), 0.0, 'AbsTol', 1.0e-12);

            testCase.VerifyProperDCMs_(cat(3, dDCMsun, dDCMlvlh, dDCMaux));
        end

        function testUnitSunDirectionIsAccepted(testCase)
            dCameraPosition_Frame = [10.0; 0.0; 0.0];
            dTargetPosition_Frame = zeros(3, 1);
            dSunDirection_Frame = [0.0; 1.0; 0.0];
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunDirection_Frame);

            [~, ~, dDCM_FrameFromPose] = objGenerator.pointToTarget();

            testCase.VerifyProperDCMs_(dDCM_FrameFromPose);
        end

        function testSeededScatteringReportsGeometricOffPointing(testCase)
            dCameraPosition_Frame = [10.0, 0.0, -10.0, 0.0; ...
                                     0.0, 10.0, 0.0, -10.0; ...
                                     2.0, 2.0, 2.0, 2.0];
            dTargetPosition_Frame = zeros(3, 1);
            dSunPosition_Frame = [50.0; 40.0; 30.0];
            dBaseBoresight_Frame = (dTargetPosition_Frame - dCameraPosition_Frame) ./ ...
                vecnorm(dTargetPosition_Frame - dCameraPosition_Frame, 2, 1);

            rng(42, 'twister');
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);
            [~, ~, dDCM_FrameFromPose, dOffPointingAngles] = objGenerator.pointToTarget( ...
                'dSigmaDegRotAboutBoresight', 5.0, ...
                'dSigmaOffPointingDegAngle', 2.0, ...
                'enumOffPointingMode', "randomAxis");

            dScatteredBoresight_Frame = reshape(dDCM_FrameFromPose(:, 3, :), 3, []);
            dBoresightDotProducts = sum(dBaseBoresight_Frame .* dScatteredBoresight_Frame, 1);
            dBoresightDotProducts = min(max(dBoresightDotProducts, -1.0), 1.0);
            dMeasuredOffPointingAngles = transpose(acosd(dBoresightDotProducts));

            testCase.verifyEqual(dOffPointingAngles, dMeasuredOffPointingAngles, 'AbsTol', 1.0e-12);
            testCase.verifyGreaterThan(max(dOffPointingAngles), 0.0);
            testCase.verifyTrue(all(isfinite(dOffPointingAngles)));
            testCase.VerifyProperDCMs_(dDCM_FrameFromPose);
        end

        function testTargetDirectionAndPoseDisplacementBranches(testCase)
            dCameraPosition_Frame = [10.0; 2.0; -1.0];
            dTargetDirectionFromCamera_Frame = [-2.0; 0.0; 0.0];
            dSunPosition_Frame = [0.0; 20.0; 5.0];
            dDCM_DisplacedPoseFromPose = [0.0, -1.0, 0.0; ...
                                          1.0,  0.0, 0.0; ...
                                          0.0,  0.0, 1.0];

            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetDirectionFromCamera_Frame, dSunPosition_Frame);
            [~, ~, dDCM_Base] = objGenerator.pointToTarget( ...
                dCameraPosition_Frame, dTargetDirectionFromCamera_Frame, ...
                'bInputIsTargetDirectionFromCam', true);
            [~, ~, dDCM_Displaced] = objGenerator.pointToTarget( ...
                dCameraPosition_Frame, dTargetDirectionFromCamera_Frame, ...
                'bInputIsTargetDirectionFromCam', true, ...
                'dDCM_displacedPoseFromPose', dDCM_DisplacedPoseFromPose);

            dExpectedBoresight_Frame = dTargetDirectionFromCamera_Frame ./ ...
                norm(dTargetDirectionFromCamera_Frame);
            dExpectedDisplacedDCM = dDCM_Base * dDCM_DisplacedPoseFromPose';
            testCase.verifyEqual(dDCM_Base(:, 3), dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dDCM_Displaced, dExpectedDisplacedDCM, 'AbsTol', 1.0e-12);
            testCase.VerifyProperDCMs_(cat(3, dDCM_Base, dDCM_Displaced));
        end

        function testQuaternionOutputBranchesMatchDCMConversion(testCase)
            dCameraPosition_Frame = [10.0; 0.0; 0.0];
            dTargetPosition_Frame = zeros(3, 1);
            dSunPosition_Frame = [0.0; 10.0; 2.0];
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);

            [~, ~, dDCM_FrameFromPose] = objGenerator.pointToTarget();
            [~, dQuatVectorScalar, ~] = objGenerator.pointToTarget( ...
                'enumOutRot3Param', EnumRotParams.QUAT_VSRPplus);
            [~, dQuatScalarVector, ~] = objGenerator.pointToTarget( ...
                'enumOutRot3Param', EnumRotParams.QUAT_SVRPplus);

            testCase.verifyEqual(dQuatVectorScalar, ...
                DCM2quatSeq(dDCM_FrameFromPose, true), 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dQuatScalarVector, ...
                DCM2quatSeq(dDCM_FrameFromPose, false), 'AbsTol', 1.0e-12);
        end

        function testSameOnBatchBoresightRollUsesOneRotation(testCase)
            ui32NumPoses = uint32(4);
            dCameraPosition_Frame = repmat([10.0; 0.0; 0.0], 1, double(ui32NumPoses));
            dTargetPosition_Frame = zeros(3, 1);
            dSunPosition_Frame = [0.0; 10.0; 2.0];

            rng(21, 'twister');
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);
            [~, ~, dDCM_FrameFromPose] = objGenerator.pointToTarget( ...
                'dSigmaDegRotAboutBoresight', 5.0, ...
                'enumDisplaceDistribution', "gaussian_same_on_batch");

            for ui32PoseIdx = uint32(2):ui32NumPoses
                testCase.verifyEqual(dDCM_FrameFromPose(:, :, double(ui32PoseIdx)), ...
                    dDCM_FrameFromPose(:, :, 1), 'AbsTol', 1.0e-12);
            end
            testCase.VerifyProperDCMs_(dDCM_FrameFromPose);
        end

        function testDeterministicDisplacementModesReturnGeometricOutputs(testCase)
            dLookAtPoint_Frame = [10.0; 0.0; 0.0];
            dReferenceAxis_Frame = [0.0; 0.0; 1.0];
            dLookAtDisplacement = 2.0;
            dRotationAngle = deg2rad(30.0);

            [dLookAtBoresight_Frame, dNewLookAtPoint_Frame] = ...
                CAttitudePointingGenerator.ComputeDisplacedBoresight( ...
                    dLookAtPoint_Frame, dReferenceAxis_Frame, dLookAtDisplacement, ...
                    'enumDisplacementMode', "lookAtPoint");
            [dInPlaneBoresight_Frame, dInPlaneLookAtPoint_Frame] = ...
                CAttitudePointingGenerator.ComputeDisplacedBoresight( ...
                    dLookAtPoint_Frame, dReferenceAxis_Frame, dLookAtDisplacement, ...
                    'enumDisplacementMode', "lookAtPoint", ...
                    'bDisplaceOrthogonalToRefAxisPlane', true);
            [dRotatedBoresight_Frame, dUnusedLookAtPoint_Frame] = ...
                CAttitudePointingGenerator.ComputeDisplacedBoresight( ...
                    dLookAtPoint_Frame, dReferenceAxis_Frame, dRotationAngle, ...
                    'enumDisplacementMode', "rotate3d");

            dExpectedLookAtPoint_Frame = [10.0; -2.0; 0.0];
            dExpectedLookAtBoresight_Frame = dExpectedLookAtPoint_Frame ./ ...
                norm(dExpectedLookAtPoint_Frame);
            dExpectedInPlaneLookAtPoint_Frame = [10.0; 0.0; -2.0];
            dExpectedInPlaneBoresight_Frame = dExpectedInPlaneLookAtPoint_Frame ./ ...
                norm(dExpectedInPlaneLookAtPoint_Frame);
            dActualRotationAngle = acos(dot( ...
                dLookAtPoint_Frame ./ norm(dLookAtPoint_Frame), dRotatedBoresight_Frame));

            testCase.verifyEqual(dNewLookAtPoint_Frame, dExpectedLookAtPoint_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dLookAtBoresight_Frame, dExpectedLookAtBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dInPlaneLookAtPoint_Frame, ...
                dExpectedInPlaneLookAtPoint_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dInPlaneBoresight_Frame, ...
                dExpectedInPlaneBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dActualRotationAngle, dRotationAngle, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dUnusedLookAtPoint_Frame, zeros(3, 1), 'AbsTol', 0.0);
        end

        function testZeroDisplacementReturnsInputGeometry(testCase)
            dLookAtPoint_Frame = [10.0, 0.0; ...
                                  0.0, 3.0; ...
                                  0.0, 4.0];
            dReferenceAxis_Frame = repmat([0.0; 0.0; 1.0], 1, 2);

            [dBoresight_Frame, dNewLookAtPoint_Frame] = ...
                CAttitudePointingGenerator.ComputeDisplacedBoresight( ...
                    dLookAtPoint_Frame, dReferenceAxis_Frame, 0.0);

            dExpectedBoresight_Frame = dLookAtPoint_Frame ./ ...
                vecnorm(dLookAtPoint_Frame, 2, 1);
            testCase.verifyEqual(dBoresight_Frame, ...
                dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dNewLookAtPoint_Frame, ...
                dLookAtPoint_Frame, 'AbsTol', 0.0);
        end

        function testBatchedInPlaneLookAtDisplacementIsVectorized(testCase)
            dLookAtPoint_Frame = [10.0, 0.0; ...
                                  0.0, 8.0; ...
                                  0.0, 0.0];
            dReferenceAxis_Frame = repmat([0.0; 0.0; 1.0], 1, 2);
            dDisplacementNorms = [2.0, 3.0];

            [dBoresight_Frame, dNewLookAtPoint_Frame] = ...
                CAttitudePointingGenerator.ComputeDisplacedBoresight( ...
                    dLookAtPoint_Frame, dReferenceAxis_Frame, dDisplacementNorms, ...
                    'enumDisplacementMode', "lookAtPoint", ...
                    'bDisplaceOrthogonalToRefAxisPlane', true);

            dExpectedLookAtPoint_Frame = dLookAtPoint_Frame - ...
                [0.0, 0.0; 0.0, 0.0; 2.0, 3.0];
            dExpectedBoresight_Frame = dExpectedLookAtPoint_Frame ./ ...
                vecnorm(dExpectedLookAtPoint_Frame, 2, 1);
            testCase.verifyEqual(dNewLookAtPoint_Frame, ...
                dExpectedLookAtPoint_Frame, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dBoresight_Frame, ...
                dExpectedBoresight_Frame, 'AbsTol', 1.0e-12);
        end

        function testReferenceAxisOffPointingModesSelectPlane(testCase)
            dCameraPosition_Frame = zeros(3, 1);
            dTargetPosition_Frame = [10.0; 0.0; 0.0];
            dSunPosition_Frame = [0.0; 10.0; 2.0];
            dReferenceAxis_Frame = [0.0; 0.0; 1.0];
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);

            rng(12, 'twister');
            [~, ~, dDCM_OutOfPlane] = objGenerator.pointToTarget( ...
                'dSigmaOffPointingDegAngle', 2.0, ...
                'enumOffPointingMode', "refAxisOutOfPlane", ...
                'dReferenceAxis_Frame', dReferenceAxis_Frame);
            rng(12, 'twister');
            [~, ~, dDCM_InPlane] = objGenerator.pointToTarget( ...
                'dSigmaOffPointingDegAngle', 2.0, ...
                'enumOffPointingMode', "refAxisInPlane", ...
                'dReferenceAxis_Frame', dReferenceAxis_Frame);

            testCase.verifyEqual(dDCM_OutOfPlane(3, 3), 0.0, 'AbsTol', 1.0e-12);
            testCase.verifyEqual(dDCM_InPlane(2, 3), 0.0, 'AbsTol', 1.0e-12);
            testCase.verifyNotEqual(dDCM_OutOfPlane(:, 3), dDCM_InPlane(:, 3));
            testCase.VerifyProperDCMs_(cat(3, dDCM_OutOfPlane, dDCM_InPlane));
        end

        function testStochasticDisplacementDistributionBranches(testCase)
            ui32NumSamples = uint32(4);
            dLookAtPoint_Frame = repmat([10.0; 0.0; 0.0], 1, double(ui32NumSamples));
            dReferenceAxis_Frame = repmat([0.0; 0.0; 1.0], 1, double(ui32NumSamples));
            cellDistributions = {"gaussian", "uniform", ...
                                 "gaussian_same_on_batch", "uniform_same_on_batch"};

            for ui32DistributionIdx = uint32(1):uint32(numel(cellDistributions))
                charDistribution = cellDistributions{double(ui32DistributionIdx)};
                rng(double(ui32DistributionIdx), 'twister');
                [dBoresight_Frame, ~] = CAttitudePointingGenerator.ComputeDisplacedBoresight( ...
                    dLookAtPoint_Frame, dReferenceAxis_Frame, 0.0, ...
                    'enumDisplacementMode', "rotate3d", ...
                    'dDisplaceSigma', deg2rad(2.0), ...
                    'enumDisplaceDistribution', charDistribution);

                testCase.verifyEqual(vecnorm(dBoresight_Frame, 2, 1), ...
                    ones(1, double(ui32NumSamples)), 'AbsTol', 1.0e-12);
                testCase.verifyTrue(all(isfinite(dBoresight_Frame), 'all'));

                if contains(charDistribution, "same_on_batch")
                    testCase.verifyEqual(dBoresight_Frame, ...
                        repmat(dBoresight_Frame(:, 1), 1, double(ui32NumSamples)), ...
                        'AbsTol', 1.0e-12);
                end
            end
        end

        function testLegacySunConstraintStillReturnsProperDCM(testCase)
            dCameraPosition_Frame = [10.0; 0.0; 0.0];
            dTargetPosition_Frame = zeros(3, 1);
            dSunPosition_Frame = [0.0; 10.0; 2.0];
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame);

            [~, dOutRot3, dDCM_FrameFromPose] = objGenerator.pointToTarget_SunDirConstraint();

            testCase.verifyEqual(dOutRot3, dDCM_FrameFromPose, 'AbsTol', 0.0);
            testCase.VerifyProperDCMs_(dDCM_FrameFromPose);
        end

        function testRejectsZeroConstraintVectors(testCase)
            dCameraPosition_Frame = [10.0; 0.0; 0.0];
            dTargetPosition_Frame = zeros(3, 1);
            objGenerator = CAttitudePointingGenerator( ...
                dCameraPosition_Frame, dTargetPosition_Frame, zeros(3, 1));

            testCase.verifyError(@() objGenerator.pointToTarget(), '');
            testCase.verifyError(@() objGenerator.pointToTarget( ...
                'dVelocity_Frame', zeros(3, 1), ...
                'enumConstraintType', "trackLVLH"), '');
            testCase.verifyError(@() objGenerator.pointToTarget( ...
                'dAuxiliaryAxis', zeros(3, 1), ...
                'enumConstraintType', "auxiliaryAxis"), '');
        end
    end

    methods (Access = private)
        function VerifyProperDCMs_(testCase, dDCM_FrameFromPose)
            for ui32PoseIdx = uint32(1):uint32(size(dDCM_FrameFromPose, 3))
                dDCM = dDCM_FrameFromPose(:, :, double(ui32PoseIdx));
                testCase.verifyEqual(dDCM' * dDCM, eye(3), 'AbsTol', 1.0e-12);
                testCase.verifyEqual(det(dDCM), 1.0, 'AbsTol', 1.0e-12);
            end
        end
    end
end
