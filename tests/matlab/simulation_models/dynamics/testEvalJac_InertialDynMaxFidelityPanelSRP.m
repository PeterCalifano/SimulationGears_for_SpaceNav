classdef testEvalJac_InertialDynMaxFidelityPanelSRP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Verify max-fidelity panel force and position linearization through the
    % actual dynamics entry points. Use independent plate-force/derivative
    % oracles, synthetic occlusion, SI/unit and attitude transformations,
    % model-selection gates and fixed-allocation source/MEX comparisons.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testPanelSRPJacobianMatchesRhsFiniteDifference(testCase)
            %% SIGNATURE
            % testCase.testPanelSRPJacobianMatchesRhsFiniteDifference()
            %% DESCRIPTION
            % Retain the unshadowed legacy caller's frozen-attitude derivative.
            %% INPUT
            % testCase  MATLAB assertion context.
            %% OUTPUT
            % None; assert against independently perturbed public RHS calls.
            %% CHANGELOG
            % 04-10-2026  Pietro Califano, Codex GPT-6  Document the retained contract.
            %% DEPENDENCIES
            % EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity.

            dxState = [1.0; -0.5; 0.3; 0.0; 0.0; 0.0];
            strDynParams = testCase.buildPanelDynParams(dxState(1:3));
            strModelConfigFlags = testCase.buildPanelModelFlags();

            dJacAnalytical = EvalJac_InertialDynMaxFidelity(0.0, ...
                                                            dxState, ...
                                                            strDynParams, ...
                                                            strModelConfigFlags);
            dJacFD = testCase.finiteDifferenceRhsJacobian(dxState, ...
                                                          strDynParams, ...
                                                          strModelConfigFlags);

            testCase.verifyEqual(dJacAnalytical(4:6, 1:3), dJacFD, 'RelTol', 2e-5, 'AbsTol', 2e-10);
        end
        function TestShadowForceAndJacobian(self)
            %% SIGNATURE
            % self.TestShadowForceAndJacobian()
            %% DESCRIPTION
            % Check a quarter-area blocker against independent normal-incidence
            % plate force, torque and held/live-pressure position partials.
            %% INPUT
            % self  MATLAB assertion context.
            %% OUTPUT
            % None; assert physical oracles and local RHS finite differences.
            %% CHANGELOG
            % 04-10-2026  Pietro Califano, Codex GPT-6  Verify prepared truth shadowing.
            %% DEPENDENCIES
            % EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity.

            for bLivePressure = [false, true]
                [dxState, strDynParams] = self.buildShadowDynParams('m');
                strFlags = self.buildPanelModelFlags();
                strFlags.bRecomputeSRPpressureFromDistance = bLivePressure;
                [dxRate, strInfo] = EvalRHS_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
                dJacobian = EvalJac_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);

                % A quarter-sized triangle blocks exactly one quarter of the
                % lower face. The upper face remains fully illuminated.
                strPanel = strDynParams.strSCdata.strSRPpanelData;
                strShadow = strPanel.strShadowData;
                dVisible = ComputePanelSunVisibility([0; 0; 1], strPanel.dQuadsNormals_SCB, ...
                    strShadow.dSamplePoints_SCB, strShadow.dFaceVertices_SCB, strShadow.dRayOffset);
                self.verifyEqual(dVisible, [0.75; 1]);
                dVisibleArea = [1.5; 0.5];
                dNormalResponse = dVisibleArea .* (1 + strPanel.dDiffSpecQuadsCoeffs(:, 2) + ...
                    2 * strPanel.dDiffSpecQuadsCoeffs(:, 1) / 3);
                dPressure = strInfo.dSolarPressure;
                dMass = strDynParams.strSCdata.dSCmass;
                dRange = 1.2e11;
                dExpectedForce = -dPressure * sum(dNormalResponse) * [0; 0; 1];
                dTangentialPartial = dPressure * sum(dVisibleArea .* ...
                    (1 - strPanel.dDiffSpecQuadsCoeffs(:, 2))) / (dMass * dRange);
                dRadialPartial = -double(bLivePressure) * 2 * dPressure * ...
                    sum(dNormalResponse) / (dMass * dRange);
                dExpectedJacobian = diag([dTangentialPartial, dTangentialPartial, dRadialPartial]);

                self.verifyEqual(dxRate(4:6), dExpectedForce / dMass, 'RelTol', 1e-13, 'AbsTol', 1e-22);
                self.verifyEqual(dJacobian(4:6, 1:3), dExpectedJacobian, 'RelTol', 1e-13, 'AbsTol', 1e-28);
                dExpectedTorque = sum(cross(strPanel.dQuadsPressCentre_SCB, ...
                    -dPressure * [0; 0; 1] * dNormalResponse.', 1), 2);
                self.verifyEqual(strInfo.dSRPtorque_SCB, dExpectedTorque, 'RelTol', 1e-13, 'AbsTol', 1e-22);

                % Stay within one visibility branch while perturbing the real
                % RHS; do not compare derivatives across quadrature switches.
                dJacobianFD = self.finiteDifferenceRhsJacobian(dxState, strDynParams, strFlags, 1e5);
                self.verifyEqual(dJacobian(4:6, 1:3), dJacobianFD, 'RelTol', 1e-7, 'AbsTol', 1e-26);
                strUnshadowed = strDynParams;
                strUnshadowed.strSCdata.strSRPpanelData = rmfield(strPanel, 'strShadowData');
                dxUnshadowed = EvalRHS_InertialDynMaxFidelity(0, dxState, strUnshadowed, strFlags);
                self.verifyLessThan(norm(dxRate(4:6)), 0.9 * norm(dxUnshadowed(4:6)));
            end
        end

        function TestShadowUnitsAndAttitude(self)
            %% SIGNATURE
            % self.TestShadowUnitsAndAttitude()
            %% DESCRIPTION
            % Preserve shadow force/partials under dynamics and geometry unit
            % changes, a common attitude rotation and quaternion rescaling.
            %% INPUT
            % self  MATLAB assertion context.
            %% OUTPUT
            % None; assert acceleration/Jacobian transformations.
            %% CHANGELOG
            % 04-10-2026  Pietro Califano, Codex GPT-6  Verify units and body frames.
            %% DEPENDENCIES
            % EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity, Quat2DCM.

            strFlags = self.buildPanelModelFlags();
            [dxMetres, strMetres] = self.buildShadowDynParams('m');
            dxReference = EvalRHS_InertialDynMaxFidelity(0, dxMetres, strMetres, strFlags);
            dJacReference = EvalJac_InertialDynMaxFidelity(0, dxMetres, strMetres, strFlags);

            for charUnit = {'m', 'km'}
                [dxState, strDynParams] = self.buildShadowDynParams(charUnit{1});
                dScale = 1;
                if strcmp(charUnit{1}, 'km')
                    dScale = 1e3;
                end
                dxRate = EvalRHS_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
                dJacobian = EvalJac_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
                self.verifyEqual(dScale * dxRate(4:6), dxReference(4:6), 'RelTol', 1e-13);
                self.verifyEqual(dJacobian, dJacReference, 'RelTol', 1e-13, 'AbsTol', 1e-28);

                % Supply metre geometry to kilometre dynamics as another
                % supported payload; avoid applying unit conversion twice.
                strDynParams.strSCdata.strSRPpanelData = strMetres.strSCdata.strSRPpanelData;
                dxMixed = EvalRHS_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
                dJacMixed = EvalJac_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
                self.verifyEqual(dxMixed, dxRate, 'RelTol', 1e-13, 'AbsTol', 1e-22);
                self.verifyEqual(dJacMixed, dJacobian, 'RelTol', 1e-13, 'AbsTol', 1e-28);
            end

            % Rotate the Sun/attitude together and keep the same body geometry.
            % Scale the quaternion too: the panel law defines normalized attitude.
            dQuaternion = [cos(0.3); sin(0.3) * [1; 1; 1] / sqrt(3)];
            dRotation = Quat2DCM(dQuaternion);
            strRotated = strMetres;
            strRotated.strSCdata.dQuat_INfromSCB = 3 * dQuaternion;
            strRotated.strBody3rdData(1).strOrbitData = self.constantOrbitData( ...
                dxMetres(1:3) + dRotation * [0; 0; 1.2e11]);
            dxRotated = EvalRHS_InertialDynMaxFidelity(0, dxMetres, strRotated, strFlags);
            dJacRotated = EvalJac_InertialDynMaxFidelity(0, dxMetres, strRotated, strFlags);
            self.verifyEqual(dxRotated(4:6), dRotation * dxReference(4:6), 'RelTol', 1e-12);
            self.verifyEqual(dJacRotated(4:6, 1:3), dRotation * ...
                dJacReference(4:6, 1:3) * dRotation.', 'RelTol', 1e-12, 'AbsTol', 1e-28);
        end

        function TestShadowSelectionAndInactiveSrp(self)
            %% SIGNATURE
            % self.TestShadowSelectionAndInactiveSrp()
            %% DESCRIPTION
            % Keep prepared shadow data independent of panel/cannonball
            % selection and suppress panel response for disabled SRP/eclipses.
            %% INPUT
            % self  MATLAB assertion context.
            %% OUTPUT
            % None; assert selected force and position blocks.
            %% CHANGELOG
            % 04-10-2026  Pietro Califano, Codex GPT-6  Verify selection and inactive gates.
            %% DEPENDENCIES
            % EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity.

            [dxState, strDynParams] = self.buildShadowDynParams('m');
            strFlags = self.buildPanelModelFlags();

            % Cover the lower face with an opaque back-facing upper triangle.
            % Its optical side stays dark, but its geometry must still block.
            strCovered = strDynParams;
            strCoveredPanel = strCovered.strSCdata.strSRPpanelData;
            strCoveredPanel.dVerticesPos(4:6, 1:2) = 2 * strCoveredPanel.dVerticesPos(4:6, 1:2);
            strCoveredPanel.dSCquadsArea(2) = 2;
            strCoveredPanel.dQuadsNormals_SCB(:, 2) = [0; 0; -1];
            [dVertices, dSamples] = BuildPanelSrpShadowData(strCoveredPanel, uint32(3));
            strCoveredPanel.strShadowData.dFaceVertices_SCB = dVertices;
            strCoveredPanel.strShadowData.dSamplePoints_SCB = dSamples;
            strCovered.strSCdata.strSRPpanelData = strCoveredPanel;
            dxCovered = EvalRHS_InertialDynMaxFidelity(0, dxState, strCovered, strFlags);
            dJacCovered = EvalJac_InertialDynMaxFidelity(0, dxState, strCovered, strFlags);
            self.verifyEqual(dxCovered(4:6), zeros(3, 1));
            self.verifyEqual(dJacCovered(4:6, 1:3), zeros(3));

            % Move the same blocker clear of every ray and recover the lower
            % plate's analytical normal-incidence response.
            strCoveredPanel.strShadowData.dFaceVertices_SCB(:, :, 2) = dVertices(:, :, 2) + [5; 0; 0];
            strCoveredPanel.strShadowData.dSamplePoints_SCB(:, :, 2) = dSamples(:, :, 2) + [5; 0; 0];
            strCovered.strSCdata.strSRPpanelData = strCoveredPanel;
            [dxClear, strClear] = EvalRHS_InertialDynMaxFidelity(0, dxState, strCovered, strFlags);
            dExpected = -strClear.dSolarPressure * 2 * (1 + 0.1 + 2 * 0.2 / 3) / 7;
            self.verifyEqual(dxClear(4:6), [0; 0; dExpected], 'RelTol', 1e-13, 'AbsTol', 1e-22);

            strFlags.bUsePanelSRP = false;
            strLegacy = strDynParams;
            strLegacy.strSCdata.strSRPpanelData = rmfield( ...
                strLegacy.strSCdata.strSRPpanelData, 'strShadowData');
            dxSelected = EvalRHS_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
            dxLegacy = EvalRHS_InertialDynMaxFidelity(0, dxState, strLegacy, strFlags);
            dJacSelected = EvalJac_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
            dJacLegacy = EvalJac_InertialDynMaxFidelity(0, dxState, strLegacy, strFlags);
            self.verifyEqual(dxSelected, dxLegacy);
            self.verifyEqual(dJacSelected, dJacLegacy);

            % Keep both complete force payloads while switching SRP off.
            strFlags.bUsePanelSRP = true;
            strFlags.bIncludeSRP = false;
            dxInactive = EvalRHS_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
            dJacInactive = EvalJac_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
            self.verifyEqual(dxInactive(4:6), zeros(3, 1));
            self.verifyEqual(dJacInactive(4:6, 1:3), zeros(3));

            % Zero pressure disables force and partials in either pressure mode.
            strFlags.bIncludeSRP = true;
            strZeroPressure = strDynParams;
            strZeroPressure.strSRPdata.dP_SRP0 = 0;
            strZeroPressure.strSRPdata.dP_SRP = 0;
            for bLivePressure = [false, true]
                strFlags.bRecomputeSRPpressureFromDistance = bLivePressure;
                dxZero = EvalRHS_InertialDynMaxFidelity(0, dxState, strZeroPressure, strFlags);
                dJacZero = EvalJac_InertialDynMaxFidelity(0, dxState, strZeroPressure, strFlags);
                self.verifyEqual(dxZero(4:6), zeros(3, 1));
                self.verifyEqual(dJacZero(4:6, 1:3), zeros(3));
            end

            % Put the spacecraft inside the caller-owned target eclipse.
            strFlags.bIncludeEclipse = true;
            strDynParams.strMainData.dRefRadius = 100;
            dxState(1:3) = [-1000; 0; 0];
            strDynParams.strBody3rdData(1).strOrbitData = self.constantOrbitData([1.2e11; 0; 0]);
            dxEclipsed = EvalRHS_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
            dJacEclipsed = EvalJac_InertialDynMaxFidelity(0, dxState, strDynParams, strFlags);
            self.verifyEqual(dxEclipsed(4:6), zeros(3, 1));
            self.verifyEqual(dJacEclipsed(4:6, 1:3), zeros(3));
        end

        function TestGeneratedShadowModels_(self)
            %% SIGNATURE
            % self.TestGeneratedShadowModels_()
            %% DESCRIPTION
            % Generate the actual RHS/Jacobian interfaces with fixed shadow
            % dimensions. Check both unit/pressure modes and panel selection.
            %% INPUT
            % self  MATLAB assertion context; require MATLAB Coder.
            %% OUTPUT
            % None; assert paired source/MEX values. Print the build directory
            % and preserve generated artifacts for inspection after failures.
            %% CHANGELOG
            % 04-10-2026  Pietro Califano, Codex GPT-6  Verify fixed-allocation interfaces.
            %% DEPENDENCIES
            % MATLAB Coder, EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity.

            self.assertTrue(license('test', 'MATLAB_Coder') ~= 0);
            charBuildRoot = tempname;
            mkdir(charBuildRoot);
            fprintf('Panel shadow contract builds: %s\n', charBuildRoot);
            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            addpath(charBuildRoot);
            objConfig = coder.config('mex');
            objConfig.TargetLang = 'C++';
            objConfig.GenerateReport = false;
            objConfig.EnableDynamicMemoryAllocation = false;
            objConfig.EnableVariableSizing = false;

            ui32Build = uint32(0);
            for charUnit = {'m', 'km'}
                [dxState, strDynParams] = self.buildShadowDynParams(charUnit{1});
                for bLivePressure = [false, true]
                    for bPanel = [true, false]
                        % Cover one cannonball specialization with all shadow
                        % data present; its selector must prune the panel path.
                        if ~bPanel && (bLivePressure || strcmp(charUnit{1}, 'km'))
                            continue
                        end
                        strFlags = self.buildPanelModelFlags();
                        strFlags.bUsePanelSRP = bPanel;
                        strFlags.bRecomputeSRPpressureFromDistance = bLivePressure;
                        ui32Build = ui32Build + uint32(1);
                        charRhs = sprintf('PanelShadowRhs_%u_mex', ui32Build);
                        charJac = sprintf('PanelShadowJac_%u_mex', ui32Build);
                        % Reserve degree-three storage for every body while
                        % retaining the active degree-two packed prefix.
                        strBuildParams = strDynParams;
                        for ui32Body = uint32(1):uint32(numel(strBuildParams.strBody3rdData))
                            strBuildParams.strBody3rdData(ui32Body).strOrbitData.dChbvPolycoeffs = ...
                                [strBuildParams.strBody3rdData(ui32Body).strOrbitData.dChbvPolycoeffs; zeros(3, 1)];
                        end
                        cellInputs = {0, dxState, strBuildParams, coder.Constant(strFlags)};
                        codegen('-config', objConfig, 'EvalRHS_InertialDynMaxFidelity', ...
                            '-args', cellInputs, '-nargout', 2, ...
                            '-d', fullfile(charBuildRoot, charRhs), '-o', fullfile(charBuildRoot, charRhs));
                        codegen('-config', objConfig, 'EvalJac_InertialDynMaxFidelity', ...
                            '-args', cellInputs, ...
                            '-d', fullfile(charBuildRoot, charJac), '-o', fullfile(charBuildRoot, charJac));
                        fcnRhs = str2func(charRhs);
                        fcnJac = str2func(charJac);

                        % Preserve the frozen schema while changing attitude,
                        % mass, pressure and geometric visibility at runtime.
                        for ui32Case = uint32(1):uint32(6)
                            strQuery = strBuildParams;
                            if ui32Case == 2
                                strQuery.strSCdata.dQuat_INfromSCB = [cos(0.2); 0; sin(0.2); 0];
                            elseif ui32Case == 3
                                strQuery.strSCdata.dSCmass = 12;
                            elseif ui32Case == 4
                                strQuery.strSRPdata.dP_SRP0 = 0;
                                strQuery.strSRPdata.dP_SRP = 0;
                            elseif ui32Case == 5
                                % Repack another active degree inside the same
                                % coefficient capacity; preserve the Sun state.
                                dSunPosition = strQuery.strBody3rdData(1).strOrbitData.dChbvPolycoeffs([1, 4, 7]);
                                strQuery.strBody3rdData(1).strOrbitData.ui32PolyDeg = uint32(3);
                                strQuery.strBody3rdData(1).strOrbitData.dChbvPolycoeffs = ...
                                    kron(dSunPosition, [1; 0; 0; 0]);
                            elseif ui32Case == 6
                                % Enter the runtime missing-Sun branch without
                                % changing any array size or model selector.
                                strQuery.strBody3rdData(1).strOrbitData.dChbvPolycoeffs(:) = 0;
                            end
                            [dxSource, strSource] = EvalRHS_InertialDynMaxFidelity(0, dxState, strQuery, strFlags);
                            [dxCompiled, strCompiled] = fcnRhs(0, dxState, strQuery, strFlags);
                            dJacSource = EvalJac_InertialDynMaxFidelity(0, dxState, strQuery, strFlags);
                            dJacCompiled = fcnJac(0, dxState, strQuery, strFlags);
                            self.verifyEqual(dxCompiled, dxSource, 'RelTol', 1e-12, 'AbsTol', 1e-22);
                            self.verifyEqual(strCompiled.dSRPtorque_SCB, strSource.dSRPtorque_SCB, ...
                                'RelTol', 1e-12, 'AbsTol', 1e-22);
                            self.verifyEqual(dJacCompiled, dJacSource, 'RelTol', 1e-12, 'AbsTol', 1e-28);
                        end
                    end
                end
            end
        end
    end

    methods (Access = private)
        function [dxState, strDynParams] = buildShadowDynParams(self, charUnit)
            % Prepare two parallel opaque triangles with a quarter-area blocker.
            % Keep physical values identical between SI and kilometre fixtures.
            dxState = [1e4; -5e3; 3e3; 0; 0; 0];
            strDynParams = self.buildPanelDynParams(dxState(1:3));
            strDynParams.strBody3rdData(1).strOrbitData = self.constantOrbitData( ...
                dxState(1:3) + [0; 0; 1.2e11]);
            strDynParams.strSRPdata.dP_SRP0 = 4e-6;
            strDynParams.strSRPdata.dP_SRP = 4e-6;
            strDynParams.strSRPdata.dReferenceDistance = 1.5e11;
            strDynParams.strSCdata.dA_SRP = 2;

            strPanel = struct( ...
                'dVerticesPos', [0, 0, 0; 2, 0, 0; 0, 2, 0; 0, 0, 1; 1, 0, 1; 0, 1, 1], ...
                'ui32FaceVertexIds', uint32([1, 2, 3; 4, 5, 6]), ...
                'dSCquadsArea', [2; 0.5], ...
                'dDiffSpecQuadsCoeffs', [0.2, 0.1; 0.4, 0.25], ...
                'dQuadsNormals_SCB', repmat([0; 0; 1], 1, 2), ...
                'dQuadsPressCentre_SCB', [2/3, 1/3; 2/3, 1/3; 0, 1], ...
                'charLengthUnit', charUnit);
            dRayOffset = 1e-8;
            if strcmp(charUnit, 'km')
                dxState(1:3) = dxState(1:3) / 1e3;
                strDynParams.strBody3rdData(1).strOrbitData.dChbvPolycoeffs = ...
                    strDynParams.strBody3rdData(1).strOrbitData.dChbvPolycoeffs / 1e3;
                strDynParams.strSRPdata.dReferenceDistance = strDynParams.strSRPdata.dReferenceDistance / 1e3;
                strDynParams.strSRPdata.dP_SRP0 = strDynParams.strSRPdata.dP_SRP0 * 1e3;
                strDynParams.strSRPdata.dP_SRP = strDynParams.strSRPdata.dP_SRP * 1e3;
                strDynParams.strSCdata.dA_SRP = strDynParams.strSCdata.dA_SRP / 1e6;
                strPanel.dVerticesPos = strPanel.dVerticesPos / 1e3;
                strPanel.dSCquadsArea = strPanel.dSCquadsArea / 1e6;
                strPanel.dQuadsPressCentre_SCB = strPanel.dQuadsPressCentre_SCB / 1e3;
                dRayOffset = dRayOffset / 1e3;
            end

            [dVertices, dSamples] = BuildPanelSrpShadowData(strPanel, uint32(3));
            strPanel.strShadowData = struct('dFaceVertices_SCB', dVertices, ...
                'dSamplePoints_SCB', dSamples, 'dRayOffset', dRayOffset);
            strDynParams.strSCdata.strSRPpanelData = strPanel;
        end

        function strModelConfigFlags = buildPanelModelFlags(~)
            strModelConfigFlags = struct('bIncludeMainGravity', false, ...
                                         'bIncludeSunThirdBody', false, ...
                                         'bIncludeEarthThirdBody', false, ...
                                         'bIncludeSRP', true, ...
                                         'bIncludeEclipse', false, ...
                                         'bUsePanelSRP', true, ...
                                         'bIncludeSphericalHarmonics', false, ...
                                         'bRecomputeSRPpressureFromDistance', true);
        end

        function strDynParams = buildPanelDynParams(testCase, dPosSC_IN)
            dSunPos_IN = [11.0; 4.0; 3.0];
            dQuat_INfromSCB = [1; 0; 0; 0];
            dSCtoSun_IN = dSunPos_IN - dPosSC_IN;
            dSunDir_SCB = dSCtoSun_IN / norm(dSCtoSun_IN);

            dTangent1 = [0; 1; 0] - dSunDir_SCB * dot([0; 1; 0], dSunDir_SCB);
            dTangent1 = dTangent1 / norm(dTangent1);
            dTangent2 = cross(dSunDir_SCB, dTangent1);
            dNormals_SCB = [dSunDir_SCB + 0.30 * dTangent1, ...
                            dSunDir_SCB - 0.20 * dTangent1 + 0.25 * dTangent2, ...
                            -dSunDir_SCB];
            dNormals_SCB = dNormals_SCB ./ vecnorm(dNormals_SCB, 2, 1);

            strDynParams = struct();
            strDynParams.strMainData = struct('dGM', 0.0, ...
                                              'dRefRadius', 0.0, ...
                                              'dSHcoeff', [], ...
                                              'ui16MaxSHdegree', uint16(0));
            strDynParams.strBody3rdData(1).dGM = 0.0;
            strDynParams.strBody3rdData(1).dRefRadius = 0.0;
            strDynParams.strBody3rdData(1).strOrbitData = testCase.constantOrbitData(dSunPos_IN);
            strDynParams.strBody3rdData(2).dGM = 0.0;
            strDynParams.strBody3rdData(2).dRefRadius = 0.0;
            strDynParams.strBody3rdData(2).strOrbitData = testCase.constantOrbitData([0; 0; 0]);
            strDynParams.strSRPdata = struct('dP_SRP0', 4.0, ...
                                             'dP_SRP', 4.0, ...
                                             'dReferenceDistance', 10.0, ...
                                             'bRecomputePressureFromDistance', true);
            strDynParams.strSCdata = struct('dReflCoeff', 1.0, ...
                                            'dSCmass', 7.0, ...
                                            'dA_SRP', 99.0, ...
                                            'dQuat_INfromSCB', dQuat_INfromSCB, ...
                                            'dCoMpos_SCB', zeros(3, 1));
            strDynParams.strSCdata.strSRPpanelData = struct( ...
                'dSCquadsArea', [1.1; 0.7; 0.5], ...
                'dDiffSpecQuadsCoeffs', [0.2, 0.1; 0.4, 0.25; 0.6, 0.2], ...
                'dQuadsNormals_SCB', dNormals_SCB, ...
                'dQuadsPressCentre_SCB', zeros(3, 3), ...
                'charLengthUnit', 'm');
        end

        function dJacFD = finiteDifferenceRhsJacobian(~, dxState, strDynParams, strModelConfigFlags, dStep)
            if nargin < 5
                dStep = 1e-4;
            end
            dJacFD = zeros(3, 3);

            for idAxis = 1:3
                dxPerturb = zeros(6, 1);
                dxPerturb(idAxis) = dStep;

                dDxDtPlus = EvalRHS_InertialDynMaxFidelity(0.0, ...
                                                           dxState + dxPerturb, ...
                                                           strDynParams, ...
                                                           strModelConfigFlags);
                dDxDtMinus = EvalRHS_InertialDynMaxFidelity(0.0, ...
                                                            dxState - dxPerturb, ...
                                                            strDynParams, ...
                                                            strModelConfigFlags);

                dJacFD(:, idAxis) = (dDxDtPlus(4:6) - dDxDtMinus(4:6)) / (2.0 * dStep);
            end
        end

        function strOrbitData = constantOrbitData(testCase, dPosition)
            strOrbitData = struct('ui32PolyDeg', uint32(2), ...
                                  'dChbvPolycoeffs', testCase.constantChebPosition(dPosition), ...
                                  'dTimeLowBound', -100.0, ...
                                  'dTimeUpBound', 100.0);
        end

        function dCoeffs = constantChebPosition(~, dPosition)
            dCoeffs = [dPosition(1); 0; 0; ...
                       dPosition(2); 0; 0; ...
                       dPosition(3); 0; 0];
        end
    end
end
