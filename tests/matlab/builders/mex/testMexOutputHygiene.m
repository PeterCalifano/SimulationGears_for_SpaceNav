classdef testMexOutputHygiene < matlab.unittest.TestCase
    %% DESCRIPTION
    % Regression tests for MEX builder output routing and bootstrap hygiene.

    methods (Test)

        function testSetupSimGearsAddsCurrentMathCoreSubmodule(testCase)
            charRepoRoot = testMexOutputHygiene.repoRoot_();
            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));

            charNormalizeVectorPath = which('NormalizeVector');
            testCase.verifyNotEmpty(charNormalizeVectorPath);
            testCase.verifyTrue(contains(string(charNormalizeVectorPath), ...
                fullfile('lib', 'MathCore_for_ComputerVision', 'matlab')));
        end

        function testResolverDefaultsToMirroredMatlabMexLayout(testCase)
            charRepoRoot = testMexOutputHygiene.repoRoot_();
            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));

            charResolvedDir = ResolveMexBuildDirectory('', ...
                'simulation_models', 'dynamics');

            testCase.verifyEqual(string(charResolvedDir), ...
                string(fullfile(charRepoRoot, 'matlab', 'mex', ...
                'simulation_models', 'dynamics')));
        end

        function testResolverPreservesExplicitBuildDir(testCase)
            charRepoRoot = testMexOutputHygiene.repoRoot_();
            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));

            charExplicitDir = fullfile(tempdir, 'simgears_explicit_mex_dir');
            charResolvedDir = ResolveMexBuildDirectory(charExplicitDir, ...
                'simulation_models', 'dynamics');

            testCase.verifyEqual(string(charResolvedDir), string(charExplicitDir));
        end

        function testSetupSimGearsDoesNotAddGeneratedMexTree(testCase)
            charRepoRoot = testMexOutputHygiene.repoRoot_();
            charMexRoot = fullfile(charRepoRoot, 'matlab', 'mex');
            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath));
            testMexOutputHygiene.removeTreeFromPath_(charMexRoot);

            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));

            cellPathEntries = strsplit(path, pathsep);
            for idPath = 1:numel(cellPathEntries)
                charPath = cellPathEntries{idPath};
                testCase.verifyFalse(strcmp(charPath, charMexRoot) || ...
                    startsWith(charPath, [charMexRoot filesep]));
            end
            clear objPathCleanup;
        end

        function testRootMexGuardrailAcceptsCleanRepoRoot(testCase)
            charRepoRoot = testMexOutputHygiene.repoRoot_();
            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));

            AssertNoRootMexArtifacts(charRepoRoot);
        end

        function testRootMexGuardrailRejectsRootArtifacts(testCase)
            charRepoRoot = testMexOutputHygiene.repoRoot_();
            run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));

            charTempRoot = tempname;
            mkdir(charTempRoot);
            objCleanup = onCleanup(@() rmdir(charTempRoot, 's'));

            charMexArtifact = fullfile(charTempRoot, 'generated_target.mexa64');
            fidArtifact = fopen(charMexArtifact, 'w');
            testCase.assertGreaterThan(fidArtifact, 0);
            fprintf(fidArtifact, 'placeholder');
            fclose(fidArtifact);

            testCase.verifyError(@() AssertNoRootMexArtifacts(charTempRoot), ...
                'AssertNoRootMexArtifacts:RootMexArtifactsFound');
            clear objCleanup;
        end

    end

    methods (Static, Access = private)

        function charRepoRoot = repoRoot_()
            charTestDir = fileparts(mfilename('fullpath'));
            charRepoRoot = fullfile(charTestDir, '..', '..', '..', '..');
            charRepoRoot = char(java.io.File(charRepoRoot).getCanonicalPath());
        end

        function removeTreeFromPath_(charRoot)
            cellPathEntries = strsplit(path, pathsep);
            for idPath = 1:numel(cellPathEntries)
                charPath = cellPathEntries{idPath};
                if strcmp(charPath, charRoot) || startsWith(charPath, [charRoot filesep])
                    rmpath(charPath);
                end
            end
        end

    end
end
