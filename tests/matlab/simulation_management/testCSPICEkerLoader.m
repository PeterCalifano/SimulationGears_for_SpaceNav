classdef testCSPICEkerLoader < matlab.unittest.TestCase
    %% DESCRIPTION
    % Verify registry-owned scenario routing performed by CSPICEkerLoader
    % without depending on external kernel repositories or initializer scripts.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 03-08-2026  Pietro Califano, Codex     Add SimulationGears-owned Apophis metakernel coverage.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CSPICEkerLoader, MICE
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testApophisUsesRegistryOwnedDataRoot(testCase)
            % A temporary minimal metakernel isolates registry routing from the
            % contents of the production Apophis kernel pack.
            testCase.assumeNotEmpty(which('cspice_furnsh'), ...
                'MICE is required for CSPICEkerLoader tests.');
            testCase.addTeardown(@cspice_kclear);
            objFixture = testCase.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            charDataRoot = fullfile(objFixture.Folder, 'simgears_data');
            charMetaKernelFolder = fullfile(charDataRoot, 'scenarios', ...
                'Apophis', 'assets', 'spice', 'mk');
            mkdir(charMetaKernelFolder);
            testCSPICEkerLoader.WriteMinimalMetaKernel_( ...
                fullfile(charMetaKernelFolder, 'metakernel.mk'));

            % Preserve the process environment while proving that the
            % SimulationGears data root removes any requirement for a legacy
            % kernel root to exist for a registry-owned scenario.
            charOriginalDataRoot = getenv('SIMGEARS_DATA_ROOT');
            testCase.addTeardown(@setenv, 'SIMGEARS_DATA_ROOT', ...
                charOriginalDataRoot);
            setenv('SIMGEARS_DATA_ROOT', charDataRoot);
            charLegacyKernelRoot = fullfile(objFixture.Folder, ...
                'legacy_kernel_root');

            objLoader = CSPICEkerLoader(charLegacyKernelRoot, ...
                EnumScenarioName.Apophis, false); %#ok<NASGU>
            testCase.verifyGreaterThanOrEqual(cspice_ktotal('META'), int32(1));
            cspice_kclear();
        end
    end

    methods (Static, Access = private)
        function WriteMinimalMetaKernel_(charMetaKernelPath)
            %% SIGNATURE
            % WriteMinimalMetaKernel_(charMetaKernelPath)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Write a minimal SPICE metakernel and one referenced text kernel
            % used by routing tests.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % charMetaKernelPath    Destination path for the temporary metakernel.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 03-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % fopen, onCleanup
            % -------------------------------------------------------------------------------------------------------------

            arguments (Input)
                charMetaKernelPath (1,:) char
            end

            i32MetaKernelFileId = fopen(charMetaKernelPath, 'w');
            assert(i32MetaKernelFileId ~= -1, ...
                'Unable to create temporary SPICE metakernel fixture.');
            objMetaKernelCleanup = onCleanup( ...
                @() fclose(i32MetaKernelFileId)); %#ok<NASGU>
            fprintf(i32MetaKernelFileId, [ ...
                'KPL/MK\n\n', ...
                '\\begindata\n\n', ...
                'KERNELS_TO_LOAD = ( ''fixture.tpc'' )\n\n', ...
                '\\begintext\n']);

            charTextKernelPath = fullfile( ...
                fileparts(charMetaKernelPath), 'fixture.tpc');
            i32TextKernelFileId = fopen(charTextKernelPath, 'w');
            assert(i32TextKernelFileId ~= -1, ...
                'Unable to create temporary SPICE text-kernel fixture.');
            objTextKernelCleanup = onCleanup( ...
                @() fclose(i32TextKernelFileId)); %#ok<NASGU>
            fprintf(i32TextKernelFileId, [ ...
                'KPL/PCK\n\n', ...
                '\\begindata\n\n', ...
                'BODY999_RADII = ( 1.0 1.0 1.0 )\n\n', ...
                '\\begintext\n']);
        end
    end
end
