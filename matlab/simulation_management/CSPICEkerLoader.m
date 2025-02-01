classdef CSPICEkerLoader
%% DESCRIPTION
% Utility class to handle loading of CSPICE kernels using MATLAB version (mice) for the nav-system,
% nav-backend, nav-frontend, nav-loopclosure repositories part of the COSMICA project. Directory structure
% is assumed as the default for these repositories.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-08-2024        Pietro Califano         Initial class definition with default values
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CSPICE mice
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% See TODO (search for the expression)
% -------------------------------------------------------------------------------------------------------------

    properties (Access = protected)
        KERNELS_BASE_PATH_ = ""
        scenarioName;
        projectDir = "";
    end

    properties (Access = private)
        defaultTargetNames = {'Didymos_Hera', 'Itokawa', 'Bennu_OREx'};
        defaultTargetPaths = {"Milani_KERNELS", "Itokawa", "Bennu_OREx"};
        defaultTargetsDict;
    end


    methods (Access = public)

        % CONSTRUCTOR
        function self = CSPICEkerLoader(KERNELS_BASE_PATH, enumScenarioName, targetFolderName)
            arguments
                KERNELS_BASE_PATH
                enumScenarioName
                targetFolderName (1,1) {isstring, ischar} = ""
            end
                    
            % Define dictionary of (names, paths) to avoid it being shared
            self.defaultTargetsDict = containers.Map(self.defaultTargetNames, self.defaultTargetPaths);
            self.projectDir = pwd;

            % Check CSPICE is available  
            % TODO
            
            % Store kernels base path
            self.KERNELS_BASE_PATH_ = KERNELS_BASE_PATH;
        
            % Clear all previously loaded kernels
            cspice_kclear();

            % Load common kernels
            cd(fullfile(self.KERNELS_BASE_PATH_, 'common/'));
            cspice_furnsh('mkcommon.mk');
                
            % Assign scenario specific data
            switch enumScenarioName

                case EnumScenarioName.Didymos_Hera
                    
                    if strcmpi(targetFolderName, "")
                        targetFolderName = self.defaultTargetsDict('Didymos_Hera');
                    end

                case EnumScenarioName.Itokawa

                    if strcmpi(targetFolderName, "")
                        targetFolderName = self.defaultTargetsDict('Itokawa');
                    end

                case EnumScenarioName.Bennu_OREx

                    if strcmpi(targetFolderName, "")
                        targetFolderName = self.defaultTargetsDict('Bennu_OREx');
                    end
                    
                otherwise
                    error("enumScenarioName is not a valid scenario.")
            end
            
            
            % Move to kernels base folder to load metakernel            
            tmpMKpath = char(fullfile(self.KERNELS_BASE_PATH_, targetFolderName, 'mk'));
     
            cd(tmpMKpath);
            cspice_furnsh('metakernel.mk'); % Load kernels
            cd(self.projectDir);

            % Print number of loaded kernels
            fprintf("Total number of loaded kernels: %d\n", cspice_ktotal('all'))

            % TODO: add info to specify type of kernels loaded
        end    
    
        % DESTRUCTOR (manual)
        function delete(self) %#ok<INUSD>
            cspice_kclear();
        end
    
    end
end
