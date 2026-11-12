classdef testTargetRotationalStateMexParity < matlab.unittest.TestCase
    %% TESTTARGETROTATIONALSTATEMEXPARITY
    % Generated-code parity regression for torque-free target rotational-state histories.
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Build and reuse the whole-history MEX target in a temporary profile root, then compare explicitly selected
    % MEX execution with the MATLAB reference over spherical, asymmetric, nonuniform-grid, and two-output-epoch
    % cases.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 23-08-2026  Pietro Califano, Codex     First whole-history MEX parity regression.
    % 24-08-2026  Pietro Califano, Codex     Verify opt-in build, exact reuse, and explicit dispatch.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % BuildMexTargets_TargetRotation, PropagateTorqueFreeBodyHistory
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = private)
        charMexBuildDir = ''
        strMexInfo = struct()
    end

    methods (TestClassSetup)
        function buildTargetRotationMex(self)
            %% SIGNATURE
            % buildTargetRotationMex(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Build the target-rotation MEX once in a class-scoped temporary directory.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self  Active MATLAB unit-test instance.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 23-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % BuildMexTargets_TargetRotation
            % -------------------------------------------------------------------------------------------------

            % Require MATLAB Coder and the MathCore stateless step
            self.assumeTrue(license('test', 'MATLAB_Coder') == 1, ...
                'MATLAB Coder not available.');
            self.assertNotEmpty(which('IntegrateRigidBodyRK4RKMK4Step'), ...
                'Corrected MathCore stateless primitives must be on the MATLAB path.');

            % Build into a temporary folder with the MEX builders on the path
            objTempFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            self.charMexBuildDir = char(objTempFixture.Folder);

            charRepoRoot = self.RepoRoot_();
            self.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(charRepoRoot, 'matlab', 'builders', 'mex')));

        end
    end

    methods (Test)
        function testSourceMexParity(self)
            %% SIGNATURE
            % testSourceMexParity(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Verify source/MEX parity for representative torque-free histories.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self  Active MATLAB unit-test instance.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 23-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % PropagateTorqueFreeBodyHistory
            % -------------------------------------------------------------------------------------------------

            % The first request must publish one exact generated profile; the second must reuse it without
            % compiling again.
            self.strMexInfo = BuildMexTargets_TargetRotation(self.charMexBuildDir);
            self.assertTrue(self.strMexInfo.bArtifactBuilt);
            self.assertTrue(isfile(self.strMexInfo.charArtifactPath));
            self.assertTrue(isfile(self.strMexInfo.charMetadataPath));
            strReusedMexInfo = BuildMexTargets_TargetRotation(self.charMexBuildDir);
            self.assertFalse(strReusedMexInfo.bArtifactBuilt);
            self.assertEqual(strReusedMexInfo.charArtifactPath, self.strMexInfo.charArtifactPath);
            self.assertEqual(self.strMexInfo.cellMexTargets, {'PropagateTorqueFreeBodyHistory_mex'});
            self.applyFixture(matlab.unittest.fixtures.PathFixture(self.strMexInfo.charArtifactDir));

            % Share one initial state across three grids, inertias and step sizes
            dInitialDCM_INfromTB = RotationVectorToDCM([0.29; -0.17; 0.23]);
            dInitialQuaternion_TBfromIN = DCM2quatSeq(transpose(dInitialDCM_INfromTB), false);
            dInitialAngVel_TB = [0.14; -0.09; 0.27];

            cellTimegrids = {[0.0, 0.17, 0.53, 1.2], ...
                [0.0, 4.0], ...
                [0.0, 0.4, 1.1]};
            cellInertiaTensors = {[2.1, 0.08, -0.03; 0.08, 2.7, 0.05; -0.03, 0.05, 3.4], ...
                2.0 * eye(3), ...
                diag([1.8, 2.3, 3.1])};
            dMaxSteps = [0.03, 10.0, 0.02];

            for ui32CaseIdx = uint32(1):uint32(numel(cellTimegrids))
                dCaseIdx = double(ui32CaseIdx);
                dTimegrid = cellTimegrids{dCaseIdx};
                dInertiaTensor_TB = cellInertiaTensors{dCaseIdx};
                dMaxStep = dMaxSteps(dCaseIdx);

                % Run the MATLAB source, the MEX directly, and both backends through the selector
                [dQuaternionSource, dAngVelSource] = PropagateTorqueFreeBodyHistory(dTimegrid, ...
                    dInitialQuaternion_TBfromIN, ...
                    dInitialAngVel_TB, ...
                    dInertiaTensor_TB, ...
                    dMaxStep);
                [dQuaternionMex, dAngVelMex] = feval(self.strMexInfo.cellMexTargets{1}, ...
                    dTimegrid, ...
                    dInitialQuaternion_TBfromIN, ...
                    dInitialAngVel_TB, ...
                    dInertiaTensor_TB, ...
                    dMaxStep);
                [dQuaternionSelected, dAngVelSelected] = RunTorqueFreeBodyHistory(dTimegrid, ...
                    dInitialQuaternion_TBfromIN, ...
                    dInitialAngVel_TB, ...
                    dInertiaTensor_TB, ...
                    dMaxStep);
                [dQuaternionSelectedMex, dAngVelSelectedMex] = RunTorqueFreeBodyHistory(dTimegrid, ...
                    dInitialQuaternion_TBfromIN, ...
                    dInitialAngVel_TB, ...
                    dInertiaTensor_TB, ...
                    dMaxStep, ...
                    "enumBackend", EnumTargetRotationalStateBackend.MEX, ...
                    "charMexOutputRoot", self.charMexBuildDir);

                % The MEX matches the source to rounding; each selector run matches its backend exactly
                self.verifyEqual(dQuaternionMex, dQuaternionSource, 'AbsTol', 2.0e-14);
                self.verifyEqual(dAngVelMex, dAngVelSource, 'AbsTol', 2.0e-14);
                self.verifyEqual(dQuaternionSelected, dQuaternionSource, 'AbsTol', 0.0);
                self.verifyEqual(dAngVelSelected, dAngVelSource, 'AbsTol', 0.0);
                self.verifyEqual(dQuaternionSelectedMex, dQuaternionMex, 'AbsTol', 0.0);
                self.verifyEqual(dAngVelSelectedMex, dAngVelMex, 'AbsTol', 0.0);
            end
        end
    end

    methods (Static, Access = private)
        function charRepoRoot = RepoRoot_()
            %% SIGNATURE
            % charRepoRoot = RepoRoot_()
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Resolve the owning SimulationGears checkout from this test file.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % None.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % charRepoRoot  Owning repository root.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 23-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            charTestDirectory = fileparts(mfilename('fullpath'));
            charRepoRoot = fileparts(fileparts(fileparts(fileparts(charTestDirectory))));
        end
    end
end
