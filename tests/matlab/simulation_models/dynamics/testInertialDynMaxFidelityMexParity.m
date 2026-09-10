classdef testInertialDynMaxFidelityMexParity < matlab.unittest.TestCase
    %% TESTINERTIALDYNMAXFIDELITYMEXPARITY
    % Generated-code parity regression for max-fidelity inertial dynamics.
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Build and execute the spherical-harmonics and polyhedron RHS/Jacobian
    % MEX targets against their MATLAB source functions. The representative
    % payload includes Sun and Earth ephemerides so the test protects both
    % the leading Sun slice and the remaining-body reshape contract.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 24-07-2026  Pietro Califano, Codex    Add generated-code ephemeris orientation and parity regression.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % BuildMexTargets_InertialDynMaxFidelity()
    % evalRHS_InertialDynMaxFidelity()
    % evalJac_InertialDynMaxFidelity()
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = private)
        charMexBuildDir = ''         % Class-scoped generated-code directory.
        strMexInfo = struct()        % Builder target names and interface metadata.
    end

    methods (TestClassSetup)

        function buildMaxFidelityMexTargets(self)
            %% SIGNATURE
            % buildMaxFidelityMexTargets(self)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Build all supported max-fidelity MEX variants once in a
            % class-scoped temporary directory.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                  Active MATLAB unit-test instance.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 24-07-2026  Pietro Califano, Codex    First implementation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % BuildMexTargets_InertialDynMaxFidelity()
            % -----------------------------------------------------------------------------------------------------

            % Keep generated sources, reports, and binaries outside the
            % repository while retaining them for every method in this class.
            objTempFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            self.charMexBuildDir = char(objTempFixture.Folder);

            % Bootstrap the owning checkout and expose the builder and
            % generated targets only for this test class.
            charRepoRoot = self.repoRoot_();
            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));
            self.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(charRepoRoot, 'matlab', 'builders', 'mex')));
            self.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                self.charMexBuildDir));

            self.strMexInfo = BuildMexTargets_InertialDynMaxFidelity( ...
                self.charMexBuildDir);

            % Guard the target ordering consumed by the parity loop so a
            % builder-interface change fails with an explicit diagnostic.
            cellExpectedTargets = { ...
                'evalRHS_InertialDynMaxFidelity_mex', ...
                'evalJac_InertialDynMaxFidelity_mex', ...
                'evalRHS_InertialDynMaxFidelity_polyhedron_mex', ...
                'evalJac_InertialDynMaxFidelity_polyhedron_mex'};
            self.assertEqual(self.strMexInfo.cellMexTargets, ...
                cellExpectedTargets);
        end

    end

    methods (Test)

        function testSourceMexParityPreservesEphemerisColumn(self)
            %% SIGNATURE
            % testSourceMexParityPreservesEphemerisColumn(self)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Verify source/MEX RHS and Jacobian parity for both target
            % gravity selections while preserving the N-by-1 ephemeris
            % interface and diagnostic values.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                  Active MATLAB unit-test instance.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 24-07-2026  Pietro Califano, Codex    First implementation.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % evalRHS_InertialDynMaxFidelity()
            % evalJac_InertialDynMaxFidelity()
            % -----------------------------------------------------------------------------------------------------

            [dStateTimetag, dxState_IN, strDynParams, ...
                strModelConfigFlags, strAccelInfo] = ...
                self.buildRepresentativeInputs_();

            % Match the compile-time configurations emitted by the builder.
            strSHmodelConfigFlags = strModelConfigFlags;
            strSHmodelConfigFlags.bIncludeSphericalHarmonics = true;
            strSHmodelConfigFlags.bIncludePolyhedronGravity = false;

            strPolyhedronConfigFlags = strModelConfigFlags;
            strPolyhedronConfigFlags.bIncludeSphericalHarmonics = false;
            strPolyhedronConfigFlags.bIncludePolyhedronGravity = true;

            cellModelConfigFlags = { ...
                strSHmodelConfigFlags, strPolyhedronConfigFlags};
            cellRHSTargets = self.strMexInfo.cellMexTargets([1, 3]);
            cellJacTargets = self.strMexInfo.cellMexTargets([2, 4]);
            ui8ExpectedModelIds = uint8([2, 3]);

            % Execute every generated target; successful compilation alone
            % does not expose MATLAB Coder orientation-contract violations.
            for ui32ModelIdx = uint32(1):uint32(numel(cellModelConfigFlags))
                dModelIdx = double(ui32ModelIdx);
                charModelName = self.strMexInfo. ...
                    cellGravityModelNames{dModelIdx};
                strSelectedFlags = cellModelConfigFlags{dModelIdx};

                [dRHSsource, strInfoSource] = ...
                    evalRHS_InertialDynMaxFidelity( ...
                        dStateTimetag, dxState_IN, strDynParams, ...
                        strSelectedFlags);
                [dRHSmex, strInfoMex] = feval( ...
                    cellRHSTargets{dModelIdx}, ...
                    dStateTimetag, dxState_IN, strDynParams, ...
                    strSelectedFlags);

                dJacSource = evalJac_InertialDynMaxFidelity( ...
                    dStateTimetag, dxState_IN, strDynParams, ...
                    strSelectedFlags, strAccelInfo);
                dJacMex = feval(cellJacTargets{dModelIdx}, ...
                    dStateTimetag, dxState_IN, strDynParams, ...
                    strSelectedFlags, strAccelInfo);

                self.verifyEqual(dRHSmex, dRHSsource, ...
                    'AbsTol', 1.0e-13, ...
                    sprintf('%s RHS source/MEX mismatch.', charModelName));
                self.verifyEqual(dJacMex, dJacSource, ...
                    'AbsTol', 1.0e-12, ...
                    sprintf('%s Jacobian source/MEX mismatch.', ...
                        charModelName));
                self.verifySize(strInfoMex.dBodyEphemerides, [6, 1], ...
                    sprintf('%s ephemerides must remain N-by-1.', ...
                        charModelName));
                self.verifyEqual(strInfoMex.dBodyEphemerides, ...
                    strInfoSource.dBodyEphemerides, 'AbsTol', 0.0, ...
                    sprintf('%s ephemeris values differ.', charModelName));
                self.verifyEqual(strInfoMex.d3rdBodiesGM, ...
                    strInfoSource.d3rdBodiesGM, 'AbsTol', 0.0, ...
                    sprintf('%s third-body GM values differ.', ...
                        charModelName));
                self.verifyEqual(strInfoMex.ui8SelectedGravityModel, ...
                    ui8ExpectedModelIds(dModelIdx), ...
                    sprintf('%s selected model ID differs.', charModelName));
            end
        end

        function testFixedStepProviderKeepsDynamicsPayloadRuntime(self)
            %% SIGNATURE
            % testFixedStepProviderKeepsDynamicsPayloadRuntime(self)
            % -----------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Generate a thin max-fidelity entry point that delegates its
            % complete integration algorithm to the shared fixed-step provider.
            % Verify the generated target still accepts runtime dynamics
            % coefficients by changing the target GM after compilation.
            % -----------------------------------------------------------------------------------------------------
            %% INPUT
            % self                  Active MATLAB unit-test instance.
            % -----------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -----------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-07-2026  Pietro Califano, Codex    Add shared-provider codegen regression.
            % -----------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CodegenInertialFixedStepProbe()
            % PropagateFixedStep()
            % evalRHS_InertialDynMaxFidelity()
            % -----------------------------------------------------------------------------------------------------

            [~, dxInitialState, strDynParams, ...
                strModelConfigFlags] = self.buildRepresentativeInputs_();
            strModelConfigFlags.bIncludeSphericalHarmonics = false;
            strModelConfigFlags.bIncludePolyhedronGravity = false;
            dTimeSpan = [0.0, 0.2];
            dMaximumStep = 0.1;

            charProviderBuildDir = fullfile( ...
                self.charMexBuildDir, 'fixed_step_provider');
            mkdir(charProviderBuildDir);
            objCodegenConfig = coder.config('mex');
            objCodegenConfig.GenerateReport = true;
            charMexTarget = 'PropagateInertialFixedStep_mex';

            % MATLAB Coder emits the final named MEX into the current working
            % directory, so enter the class-scoped temporary build directory
            % before compiling and restore the caller directory afterward.
            charCallerDirectory = pwd;
            objDirectoryCleanup = onCleanup( ...
                @() cd(charCallerDirectory));
            cd(charProviderBuildDir);
            codegen('-config', objCodegenConfig, ...
                '-d', charProviderBuildDir, '-o', charMexTarget, ...
                'CodegenInertialFixedStepProbe', '-args', { ...
                dTimeSpan, dxInitialState, dMaximumStep, strDynParams, ...
                coder.Constant(strModelConfigFlags), ...
                coder.Constant(EnumFixedStepScheme.RK4)});
            clear objDirectoryCleanup
            self.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                charProviderBuildDir));

            [dxSourceHistory, dSourceGrid] = ...
                CodegenInertialFixedStepProbe(dTimeSpan, ...
                dxInitialState, dMaximumStep, strDynParams, ...
                strModelConfigFlags, EnumFixedStepScheme.RK4);
            [dxMexHistory, dMexGrid] = ...
                PropagateInertialFixedStep_mex( ...
                dTimeSpan, dxInitialState, dMaximumStep, strDynParams, ...
                strModelConfigFlags, EnumFixedStepScheme.RK4);

            self.verifyEqual(dMexGrid, dSourceGrid, 'AbsTol', 0.0);
            self.verifyEqual(dxMexHistory, dxSourceHistory, ...
                'AbsTol', 1.0e-13);

            strPerturbedDynParams = strDynParams;
            strPerturbedDynParams.strMainData.dGM = ...
                1.05 * strDynParams.strMainData.dGM;
            dxPerturbedSource = CodegenInertialFixedStepProbe( ...
                dTimeSpan, ...
                dxInitialState, dMaximumStep, strPerturbedDynParams, ...
                strModelConfigFlags, EnumFixedStepScheme.RK4);
            dxPerturbedMex = PropagateInertialFixedStep_mex( ...
                dTimeSpan, ...
                dxInitialState, dMaximumStep, strPerturbedDynParams, ...
                strModelConfigFlags, EnumFixedStepScheme.RK4);

            self.verifyEqual(dxPerturbedMex, dxPerturbedSource, ...
                'AbsTol', 1.0e-13);
            self.verifyGreaterThan( ...
                max(abs(dxPerturbedMex - dxMexHistory), [], 'all'), 0.0);
        end

    end

    methods (Static, Access = private)

        function charRepoRoot = repoRoot_()
            % Resolve the owning repository independently of invocation CWD.
            charTestDir = fileparts(mfilename('fullpath'));
            charRepoRoot = fullfile(charTestDir, '..', '..', '..', '..');
            charRepoRoot = char( ...
                java.io.File(charRepoRoot).getCanonicalPath());
        end

        function [dStateTimetag, dxState_IN, strDynParams, ...
                strModelConfigFlags, strAccelInfo] = ...
                buildRepresentativeInputs_()
            % Reproduce the builder's fixed MEX interface with two bodies.
            dStateTimetag = 0.0;
            dxState_IN = [4.0; 0.3; -0.2; 0.0; 0.01; 0.0];

            % Build a compact tetrahedron payload so both gravity target
            % variants share one fixed strDynParams schema.
            ui32FaceVertexIds = uint32( ...
                [1 2 3; 1 4 2; 1 3 4; 2 4 3]);
            dVerticesPos = [ 1.0  1.0  1.0; ...
                             1.0 -1.0 -1.0; ...
                            -1.0  1.0 -1.0; ...
                            -1.0 -1.0  1.0 ];
            dDensity = 2500.0;
            dGravConst = 6.67430e-11;

            [ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ...
                ComputePolyhedronFaceEdgeData( ...
                    ui32FaceVertexIds, dVerticesPos);
            [dVolume, ~] = ComputeMeshModelVolumeAndCoM( ...
                ui32FaceVertexIds, dVerticesPos);
            dGravParam = dGravConst * dDensity * dVolume;

            strPolyhedronGravityData = struct();
            strPolyhedronGravityData.ui32FaceVertexIds = ...
                ui32FaceVertexIds;
            strPolyhedronGravityData.dVerticesPos = dVerticesPos;
            strPolyhedronGravityData.dDensity = dDensity;
            strPolyhedronGravityData.ui32EdgeVertexIds = ...
                ui32EdgeVertexIds;
            strPolyhedronGravityData.dEdgeDyadics = dEdgeDyadics;
            strPolyhedronGravityData.dFaceDyadics = dFaceDyadics;
            strPolyhedronGravityData.dGravConst = dGravConst;

            dSHcoeff = zeros(4, 2);
            dSHcoeff(2, 1) = -1.0e-3;

            strDynParams = struct();
            strDynParams.strMainData.dGM = dGravParam;
            strDynParams.strMainData.dRefRadius = 1.0;
            strDynParams.strMainData.dSHcoeff = dSHcoeff;
            strDynParams.strMainData.ui16MaxSHdegree = uint16(2);
            strDynParams.strMainData.strPolyhedronGravityData = ...
                strPolyhedronGravityData;

            % Include Sun and Earth ephemerides to cover the first-body
            % slice and the remaining-body reshape in generated code.
            strDynParams.strBody3rdData(1).dGM = 3.0;
            strDynParams.strBody3rdData(1).dRefRadius = 0.1;
            strDynParams.strBody3rdData(1).strOrbitData = ...
                testInertialDynMaxFidelityMexParity. ...
                buildConstantOrbitData_([20.0; 0.0; 0.0]);
            strDynParams.strBody3rdData(2).dGM = 5.0;
            strDynParams.strBody3rdData(2).dRefRadius = 0.2;
            strDynParams.strBody3rdData(2).strOrbitData = ...
                testInertialDynMaxFidelityMexParity. ...
                buildConstantOrbitData_([0.0; 30.0; 0.0]);

            strDynParams.strSRPdata.dP_SRP0 = 4.0;
            strDynParams.strSRPdata.dP_SRP = 4.0;
            strDynParams.strSRPdata.dReferenceDistance = 10.0;
            strDynParams.strSRPdata. ...
                bRecomputePressureFromDistance = true;

            strDynParams.strSCdata.dReflCoeff = 1.0;
            strDynParams.strSCdata.dSCmass = 1.0;
            strDynParams.strSCdata.dA_SRP = 2.0;
            strDynParams.strSCdata.strSRPpanelData = struct( ...
                'dSCquadsArea', 1.0, ...
                'dDiffSpecQuadsCoeffs', [0.0 0.0], ...
                'dQuadsNormals_SCB', [1.0; 0.0; 0.0], ...
                'dQuadsPressCentre_SCB', [0.0; 0.0; 0.0], ...
                'charLengthUnit', 'm');
            strDynParams.strStochasticAccelData = ...
                GenerateGaussMarkovAccelSeq( ...
                    [0.0, 1.0], zeros(3, 1), ones(3, 1), 1.0, ...
                    uint32(1), 'bSampleInitialAccel', false);

            strModelConfigFlags = struct();
            strModelConfigFlags.bIncludeMainGravity = true;
            strModelConfigFlags.bIncludeSphericalHarmonics = false;
            strModelConfigFlags.bIncludeThirdBodies = true;
            strModelConfigFlags.bIncludeSunThirdBody = true;
            strModelConfigFlags.bIncludeEarthThirdBody = true;
            strModelConfigFlags.bIncludeSRP = true;
            strModelConfigFlags.bIncludeEclipse = true;
            strModelConfigFlags.bUsePanelSRP = false;
            strModelConfigFlags.bIncludeStochasticAcceleration = false;
            strModelConfigFlags.bIncludePolyhedronGravity = false;
            strModelConfigFlags. ...
                bRecomputeSRPpressureFromDistance = true;

            strAccelInfo = struct();
            strAccelInfo.dSRPdistToSun = norm( ...
                dxState_IN(1:3) - [20.0; 0.0; 0.0]);
            strAccelInfo.bIsSRPActive = true;
        end

        function strOrbitData = buildConstantOrbitData_(dPosition_IN)
            % Build a constant Chebyshev ephemeris for one third body.
            strOrbitData = struct();
            strOrbitData.ui32PolyDeg = uint32(2);
            strOrbitData.dChbvPolycoeffs = [ ...
                dPosition_IN(1); 0.0; 0.0; ...
                dPosition_IN(2); 0.0; 0.0; ...
                dPosition_IN(3); 0.0; 0.0];
            strOrbitData.dTimeLowBound = -100.0;
            strOrbitData.dTimeUpBound = 100.0;
        end

    end
end
