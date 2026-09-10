classdef CSPICEkerLoader
%% DESCRIPTION
% Load scenario SPICE kernels through the SimulationGears scenario registry
% when an official metakernel is available. Scenarios not yet migrated retain
% the legacy caller-provided kernel-root behavior.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-08-2024        Pietro Califano         Initial class definition with default values.
% 09-05-2025        Pietro Califano         Minor improvements for usability.
% 03-08-2026        Pietro Califano, Codex  Resolve migrated scenario kernels from SimulationGears data.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry, ResolveSimGearsDataRoot, CSPICE MICE
% -------------------------------------------------------------------------------------------------------------

    properties (Access = protected)
        KERNELS_BASE_PATH_ = ""
    end

    properties (Access = private)
        defaultTargetNames = {'Didymos', 'Itokawa', 'Bennu'};
        defaultTargetPaths = {"Milani_KERNELS", "Itokawa", "Bennu_OREx"};
        defaultTargetsDict;
    end

    methods (Access = public)

        function self = CSPICEkerLoader(charKERNELS_BASE_PATH, enumScenarioName, ...
                bLoadCommonKernels, charTargetFolderName)
            %% SIGNATURE
            % self = CSPICEkerLoader(charKERNELS_BASE_PATH, enumScenarioName, ...
            %     bLoadCommonKernels, charTargetFolderName)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Clear the current SPICE pool and load the selected scenario. A
            % registry-owned metakernel is resolved from the SimulationGears
            % data root and takes precedence over the legacy kernel root.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % charKERNELS_BASE_PATH Legacy kernel root used by scenarios not yet migrated.
            % enumScenarioName      Scenario enum or registered scenario name.
            % bLoadCommonKernels    Load the legacy common metakernel after the legacy scenario metakernel.
            % charTargetFolderName  Optional legacy scenario-folder override.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self                  Kernel-loader object. Call delete explicitly to clear the SPICE pool.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 03-08-2026  Pietro Califano, Codex     Prefer registry-owned scenario metakernels.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CScenarioRegistry, ResolveSimGearsDataRoot, cspice_kclear, cspice_furnsh
            % -------------------------------------------------------------------------------------------------------------

            arguments
                charKERNELS_BASE_PATH
                enumScenarioName (1,:) {mustBeA(enumScenarioName, ...
                    ["EnumScenarioName", "string", "char"])}
                bLoadCommonKernels (1,1) logical = false
                charTargetFolderName (1,1) {isstring, ischar} = ""
            end

            self.defaultTargetsDict = containers.Map( ...
                self.defaultTargetNames, self.defaultTargetPaths);
            cspice_kclear();

            % Registry-owned metakernels are complete scenario definitions;
            % they neither inspect nor validate the legacy kernel root.
            strScenarioSpec = CScenarioRegistry.GetScenarioSpec(enumScenarioName);
            if strlength(string( ...
                    strScenarioSpec.charDefaultSpiceMetaKernelRelativePath)) > 0
                charDataRootPath = ResolveSimGearsDataRoot();
                charMetaKernelPath = fullfile(charDataRootPath, ...
                    strScenarioSpec.charDefaultSpiceMetaKernelRelativePath);
                CSPICEkerLoader.LoadMetaKernel_(charMetaKernelPath);
                CSPICEkerLoader.PrintLoadedKernelCount_();
                return
            end

            % Preserve the legacy folder contract for scenarios whose SPICE
            % environment has not yet migrated into SimulationGears data.
            assert(isfolder(charKERNELS_BASE_PATH), ...
                'ERROR: specified path is not a valid or existing folder');
            self.KERNELS_BASE_PATH_ = charKERNELS_BASE_PATH;
            [enumResolvedScenarioName, ~] = ...
                CScenarioRegistry.ResolveScenario(enumScenarioName);

            switch enumResolvedScenarioName
                case EnumScenarioName.Didymos
                    charTargetFolderName = self.ResolveLegacyFolder_( ...
                        charTargetFolderName, 'Didymos');

                case EnumScenarioName.Itokawa
                    charTargetFolderName = self.ResolveLegacyFolder_( ...
                        charTargetFolderName, 'Itokawa');

                case EnumScenarioName.Bennu
                    charTargetFolderName = self.ResolveLegacyFolder_( ...
                        charTargetFolderName, 'Bennu');

                case EnumScenarioName.Moon
                    return

                otherwise
                    error('CSPICEkerLoader:UnsupportedLegacyScenario', ...
                        ['Scenario %s has no registry-owned metakernel and no ', ...
                         'legacy kernel-folder mapping.'], ...
                        string(strScenarioSpec.charCanonicalName));
            end

            charScenarioMetaKernelPath = fullfile( ...
                self.KERNELS_BASE_PATH_, char(charTargetFolderName), ...
                'mk', 'metakernel.mk');
            CSPICEkerLoader.LoadMetaKernel_(charScenarioMetaKernelPath);

            if bLoadCommonKernels
                charCommonMetaKernelPath = fullfile( ...
                    self.KERNELS_BASE_PATH_, 'common', 'mkcommon.mk');
                CSPICEkerLoader.LoadMetaKernel_(charCommonMetaKernelPath);
            end

            CSPICEkerLoader.PrintLoadedKernelCount_();
        end

        function delete(self) %#ok<INUSD>
            %% SIGNATURE
            % delete(self)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Explicitly clear all loaded SPICE kernels.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    Kernel-loader object.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 17-08-2024  Pietro Califano     First implementation.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % cspice_kclear
            % -------------------------------------------------------------------------------------------------------------

            cspice_kclear();
        end
    end

    methods (Access = private)
        function charTargetFolderName = ResolveLegacyFolder_(self, ...
                charTargetFolderName, charScenarioName)
            if strcmpi(charTargetFolderName, "")
                charTargetFolderName = ...
                    self.defaultTargetsDict(charScenarioName);
            end
        end
    end

    methods (Static, Access = private)
        function LoadMetaKernel_(charMetaKernelPath)
            if ~isfile(charMetaKernelPath)
                error('CSPICEkerLoader:MissingMetaKernel', ...
                    'SPICE metakernel not found at %s.', ...
                    string(charMetaKernelPath));
            end

            % SPICE resolves PATH_VALUES relative to the MATLAB working
            % directory. Restore the caller directory even when loading fails.
            charOriginalFolder = pwd;
            objFolderCleanup = onCleanup(@() cd(charOriginalFolder)); %#ok<NASGU>
            [charMetaKernelFolder, charMetaKernelName, charMetaKernelExt] = ...
                fileparts(charMetaKernelPath);
            cd(charMetaKernelFolder);
            charMetaKernelFileName = char(strcat( ...
                string(charMetaKernelName), string(charMetaKernelExt)));
            cspice_furnsh(charMetaKernelFileName);
        end

        function PrintLoadedKernelCount_()
            fprintf('Total number of loaded kernels: %d\n', ...
                cspice_ktotal('all'));
        end
    end
end
