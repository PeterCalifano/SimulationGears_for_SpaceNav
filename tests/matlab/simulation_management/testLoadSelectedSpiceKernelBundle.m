classdef testLoadSelectedSpiceKernelBundle < matlab.unittest.TestCase
    %% DESCRIPTION
    % Verify bundle identity, pool ownership and relative asset-root behavior.
    % The full-bundle check uses installed external assets when available.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 27-09-2026  Pietro Califano, Codex gpt-6  Add selected-bundle integrity tests.
    % 29-09-2026  Pietro Califano     Strengthen caller-pool and folder preservation checks.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % LoadSelectedSpiceKernelBundle, ComputeFileSha256, MICE.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testFileSha256KnownDigest(self)
            %% SIGNATURE
            % testFileSha256KnownDigest(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Verify the streamed digest against a published SHA-256 vector.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ComputeFileSha256, TemporaryFolderFixture.
            % ---------------------------------------------------------------------------------------------------------

            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            charInputFile = fullfile(objFixture.Folder, 'abc.txt');
            dFileId = fopen(charInputFile, 'wb');
            self.assertGreaterThan(dFileId, 0);
            objFileCleanup = onCleanup(@() fclose(dFileId)); %#ok<NASGU>
            fwrite(dFileId, uint8('abc'));
            clear objFileCleanup

            self.verifyEqual(ComputeFileSha256(charInputFile), ...
                'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
        end

        function testWrongHashDoesNotChangeSpicePool(self)
            %% SIGNATURE
            % testWrongHashDoesNotChangeSpicePool(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Reject a changed bundle manifest before clearing or loading kernels.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadSelectedSpiceKernelBundle, LoadScenarioDataManifest, MICE.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_furnsh'), 'MICE is required.');
            charAssetRoot = char(ResolveSimulationRenderingAssetsRoot());
            charMetaKernelFile = fullfile(charAssetRoot, 'assets', 'bodies', ...
                'apophis', 'phase_d', 'mk', 'kernels.mk');
            self.assumeTrue(isfile(charMetaKernelFile), ...
                'Phase-D bundle assets are not installed.');
            self.addTeardown(@cspice_kclear);

            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            charManifestFolder = fullfile(objFixture.Folder, 'scenarios', 'Apophis');
            mkdir(charManifestFolder);
            strManifest = LoadScenarioDataManifest('Apophis');
            strManifest.kernel_bundles.meta_kernel.sha256 = repmat('0', 1, 64);
            charManifestFile = fullfile(charManifestFolder, 'manifest.json');
            dFileId = fopen(charManifestFile, 'wb');
            self.assertGreaterThan(dFileId, 0);
            objFileCleanup = onCleanup(@() fclose(dFileId)); %#ok<NASGU>
            fwrite(dFileId, jsonencode(strManifest));
            clear objFileCleanup

            % Keep one caller-owned kernel loaded so an accidental pool clear
            % cannot pass this fail-before-load regression.
            strAncillary = NormalizeManifestStructArray( ...
                strManifest.kernel_bundles.loaded_kernels);
            charCallerKernelFile = char(ResolveScenarioAssetPath( ...
                strAncillary(1).local_path, charAssetRootPath=string(charAssetRoot)));
            cspice_kclear();
            cspice_furnsh(charCallerKernelFile);
            i32OriginalCount = cspice_ktotal('ALL');
            self.assertGreaterThan(i32OriginalCount, 0);
            charOriginalFolder = pwd;
            self.verifyError(@() LoadSelectedSpiceKernelBundle( ...
                'Apophis', 'apophis_phase_d_rcs1', ...
                charDataRootPath=string(objFixture.Folder), ...
                charAssetRootPath=string(charAssetRoot)), ...
                'LoadSelectedSpiceKernelBundle:HashMismatch');
            self.verifyEqual(cspice_ktotal('ALL'), i32OriginalCount);
            [~, ~, ~, bCallerKernelLoaded] = cspice_kinfo(charCallerKernelFile);
            self.verifyTrue(bCallerKernelLoaded);
            self.verifyEqual(pwd, charOriginalFolder);
        end

        function testPhaseDBundleLoadsWithoutDefaultEphemeris(self)
            %% SIGNATURE
            % testPhaseDBundleLoadsWithoutDefaultEphemeris(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Verify the complete registered Phase-D bundle and source-only
            % spacecraft trajectory load into a clean process pool.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadSelectedSpiceKernelBundle, MICE.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_furnsh'), 'MICE is required.');
            charAssetRoot = char(ResolveSimulationRenderingAssetsRoot());
            charMetaKernelFile = fullfile(charAssetRoot, 'assets', 'bodies', ...
                'apophis', 'phase_d', 'mk', 'kernels.mk');
            self.assumeTrue(isfile(charMetaKernelFile), ...
                'Phase-D bundle assets are not installed.');
            self.addTeardown(@cspice_kclear);

            charOriginalFolder = pwd;
            strBundle = LoadSelectedSpiceKernelBundle( ...
                'Apophis', 'apophis_phase_d_rcs1');
            self.verifyEqual(pwd, charOriginalFolder);
            self.verifyEqual(strBundle.ui32LoadedKernelCount, uint32(12));
            self.verifyEqual(strBundle.charTrajectorySha256, ...
                '00dd911ececfa0c1d212a58d9855e3e60b47f5cb3cc0f455700550cd5fd60b80');
            self.verifyEqual(cspice_ktotal('SPK'), 3);

            charSpkFiles = strings(1, 3);
            for ui32SpkIdx = uint32(1):uint32(3)
                [charSpkFile, ~, ~, ~, bFound] = ...
                    cspice_kdata(int32(ui32SpkIdx), 'SPK');
                self.assertTrue(bFound);
                charSpkFiles(ui32SpkIdx) = string(charSpkFile);
            end
            self.verifyTrue(any(endsWith(charSpkFiles, ...
                'apophis_hor_000101_500101_v01.bsp')));
            self.verifyTrue(any(endsWith(charSpkFiles, ...
                'RCS1_dep2PrEPSSTO_v7.bsp')));
            self.verifyFalse(any(endsWith(charSpkFiles, ...
                '/spk/apophis.bsp')));
        end

        function testLoadedKernelsMatchVerifiedManifest(self)
            %% SIGNATURE
            % testLoadedKernelsMatchVerifiedManifest(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Reject a manifest that hashes a same-count ancillary file set
            % different from the files actually loaded by its metakernel.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  Guard selected-bundle loaded-file identity.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadSelectedSpiceKernelBundle, LoadScenarioDataManifest, MICE.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_kdata'), 'MICE is required.');
            charAssetRoot = char(ResolveSimulationRenderingAssetsRoot());
            charMetaKernelFile = fullfile(charAssetRoot, 'assets', 'bodies', ...
                'apophis', 'phase_d', 'mk', 'kernels.mk');
            self.assumeTrue(isfile(charMetaKernelFile), ...
                'Phase-D bundle assets are not installed.');
            self.addTeardown(@cspice_kclear);

            % Duplicate a valid hashed entry so count and hash checks pass,
            % but the declared file set differs from the metakernel load.
            strManifest = LoadScenarioDataManifest('Apophis');
            strKernels = NormalizeManifestStructArray( ...
                strManifest.kernel_bundles.loaded_kernels);
            strKernels(1) = strKernels(2);
            strManifest.kernel_bundles.loaded_kernels = strKernels;
            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            charManifestFolder = fullfile(objFixture.Folder, 'scenarios', 'Apophis');
            mkdir(charManifestFolder);
            charManifestFile = fullfile(charManifestFolder, 'manifest.json');
            dFileId = fopen(charManifestFile, 'wb');
            self.assertGreaterThan(dFileId, 0);
            objFileCleanup = onCleanup(@() fclose(dFileId)); %#ok<NASGU>
            fwrite(dFileId, jsonencode(strManifest));
            clear objFileCleanup

            cspice_kclear();
            charOriginalFolder = pwd;
            self.verifyError(@() LoadSelectedSpiceKernelBundle( ...
                'Apophis', 'apophis_phase_d_rcs1', ...
                charDataRootPath=string(objFixture.Folder), ...
                charAssetRootPath=string(charAssetRoot)), ...
                'LoadSelectedSpiceKernelBundle:LoadedKernelMismatch');
            self.verifyEqual(cspice_ktotal('ALL'), 0);
            self.verifyEqual(pwd, charOriginalFolder);
        end

        function testRelativeAssetRootUsesCallerFolder(self)
            %% SIGNATURE
            % testRelativeAssetRootUsesCallerFolder(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Resolve a relative asset root before changing folders for the
            % metakernel, independently of Java's process working folder.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano     Cover relative-root bundle preparation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadSelectedSpiceKernelBundle, onCleanup, MICE.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_kdata'), 'MICE is required.');
            charAssetRoot = char(ResolveSimulationRenderingAssetsRoot());
            charMetaKernelFile = fullfile(charAssetRoot, 'assets', 'bodies', ...
                'apophis', 'phase_d', 'mk', 'kernels.mk');
            self.assumeTrue(isfile(charMetaKernelFile), ...
                'Phase-D bundle assets are not installed.');
            self.addTeardown(@cspice_kclear);

            % Change MATLAB's folder without relying on Java to follow it.
            [charAssetParent, charAssetFolder] = fileparts(charAssetRoot);
            charOriginalFolder = pwd;
            objFolderCleanup = onCleanup(@() cd(charOriginalFolder)); %#ok<NASGU>
            cd(charAssetParent);
            charCallerFolder = pwd;
            strBundle = LoadSelectedSpiceKernelBundle( ...
                'Apophis', 'apophis_phase_d_rcs1', ...
                charAssetRootPath=string(charAssetFolder));

            self.verifyEqual(strBundle.ui32LoadedKernelCount, uint32(12));
            self.verifyEqual(strBundle.charMetaKernelFile, ...
                char(java.io.File(charMetaKernelFile).getCanonicalPath()));
            self.verifyEqual(pwd, charCallerFolder);
        end
    end
end
