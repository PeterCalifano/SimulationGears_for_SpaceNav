function [objShapeModel, strBpyCommManagerPaths] = DefineShapeModel(enumTargetName, ...
                                                                    charDataRootPath, ...
                                                                    charBpyRootPath, ...
                                                                    options)
arguments
    enumTargetName      (1,:) {mustBeA(enumTargetName, ["string", "char", "EnumScenarioName"]), ...
        mustBeMember(enumTargetName, ["Apophis", "Itokawa", "Bennu", "Moon", "Mars", "Ceres", "Dydimos", "Eros", "NotDefined"])}
    charDataRootPath    (1,:) string = fullfile(getenv("HOME"), "devDir/nav-backend/simulationCodes/data/SPICE_kernels")
    charBpyRootPath     (1,:) string = fullfile(getenv("HOME"), "devDir/rendering-sw/corto_PeterCdev")
end
arguments
    options.bVertFacesOnly  (1,1) {islogical} = true;
    options.bLoadShapeModel (1,1) {islogical} = true;
end
%% SIGNATURE
% [objShapeModel, strBpyCommManagerPaths] = DefineShapeModel(enumTargetName, charDataRootPath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function defining a general purpose shape model object from the specified target name as CShapeModel in 
% SimulationGears repository (https://github.com/PeterCalifano/SimulationGears_for_SpaceNav). 
% Paths to models forBlenderPyCommManager class are also defined (ACHTUNG: currently HARDCODED).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments
%     enumTargetName      (1,:) {mustBeA(enumTargetName, ["string", "char", "EnumScenarioName"]), ...
%         mustBeMember(enumTargetName, ["Apophis", "Itokawa", "Bennu", "Moon"])}
%     charDataRootPath    (1,:) string = fullfile(getenv("HOME"), "devDir/nav-backend/simulationCodes/data/SPICE_kernels")
%     charBpyRootPath     (1,:) string = fullfile(getenv("HOME"), "devDir/rendering-sw/corto_PeterCdev")
% end
% arguments
%     options.bVertFacesOnly (1,1) {islogical} = true;
%     options.bLoadShapeModel (1,1) {islogical} = true;
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objShapeModel
% strBpyCommManagerPaths
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-04-2025    Pietro Califano     Update of paths definition
% 03-05-2025    Pietro Califano     Minor revision, add Moon setup
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% TODO: remove hardcoded path (add as input root of all data!)
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Assert path existent
assert(isfolder(charDataRootPath), sprintf("ERROR: input data path %s not found", charDataRootPath));

[~, charUsrName] = system("whoami"); % Get user
assert(contains(charUsrName, "peter") || contains(string(charUsrName(1:end-1)), "peterc-flip\pietr"), ...
    'ERROR: current implementation is only valid for peterc machines due to hardcoded paths.')

charCallDir = pwd;

switch enumTargetName

    case "Apophis"
        % DEVNOTE: currently assumes rcs-1 simulator loader
        LoadUserConfig;
        charBlenderModelPath                = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-B/blender/ApophisParticles.blend");
        %charBlenderModelPath                = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-A/blender/Apophis_RGB_smoothed.blend1");
        
        % Define shape model object
        objShapeModel = CShapeModel('file_obj', fullfile(path_to_shape_models, "apophis_v233s7_vert2_new.mod.obj"), ...
            'km', 'm', options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        try
            ui32ID = 20099942;
            objShapeModel.dObjectReferenceSize  = 1E3 * mean(cspice_bodvrd(num2str(ui32ID),'RADII',3)); % [m] ACHTUNG: Value used for Gravity SH expansion!
        catch
            warning('Fetch of Apophis data from kernels failed. Fallback to hardcoded data...')
            objShapeModel.dObjectReferenceSize  = 1E3 * 0.175930344; % [m] ACHTUNG: Value used for Gravity SH expansion!
        end
        
        objShapeModel.charTargetUnitOutput = 'm';

    case "Itokawa"

        % Define blender model path
        charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S2_Itokawa/S2_Itokawa.blend");
        % Define shape model object
        charKernelname = fullfile(charDataRootPath, 'Itokawa/dsk/hay_a_amica_5_itokawashape_v1_0_64q.bds');

        objShapeModel = CShapeModel('cspice', charKernelname, 'km', 'm', options.bVertFacesOnly, char(enumTargetName));
        objShapeModel.dObjectReferenceSize = 1E3 * 0.161915; % [m]
        objShapeModel.charTargetUnitOutput = 'm';

    case "Bennu"

        % Define blender model path
        charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S4_Bennu/S4_Bennu.blend");

        % Define shape model object
        % charKernelname = fullfile(charDataRootPath, 'Bennu_OREx/dsk/bennu_l_00050mm_alt_ptm_5595n04217_v021.bds'); %  Too large!
        charKernelname = fullfile(charDataRootPath, 'Bennu_OREx/dsk/bennu_g_03170mm_spc_obj_0000n00000_v020.bds');
        % charKernelname = fullfile(charDataRootPath, 'Bennu_OREx/dsk/bennu_g_01680mm_alt_obj_0000n00000_v021.bds');

        objShapeModel = CShapeModel('cspice', charKernelname, 'km', 'm', ...
                        options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        % Assign reference radius
        objShapeModel.dObjectReferenceSize = 245.03; % [m]
        objShapeModel.charTargetUnitOutput = 'm';

    case "Moon"

        % Define blender model path
        charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S6_Moon/S6_Moon.blend");

        % Define shape model object
        charObjModelFilePath = fullfile(charBpyRootPath, "data/scenarios/S6_Moon/Moon.obj");

        objShapeModel = CShapeModel('file_obj', charObjModelFilePath, 'km', 'm', ...
            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        objShapeModel.dObjectReferenceSize = 1737.42 * 1E3; % [m]
        objShapeModel.charTargetUnitOutput = 'm';

    case "Mars"

        charObjModelFilePath = ""; % None for now

        objShapeModel = CShapeModel('file_obj', charObjModelFilePath, 'km', 'm', ...
            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        objShapeModel.dObjectReferenceSize = 3386.2 * 1E3; % [m]
        objShapeModel.charTargetUnitOutput = 'm';

        % Define shape matrix in principal TF
        dABC = [3395428, 3395428, 3377678];
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dABC(1)^2, 1/dABC(2)^2, 1/dABC(3)^2]);

    case "Ceres" 
        charObjModelFilePath = ""; % None for now

        objShapeModel = CShapeModel('file_obj', charObjModelFilePath, 'km', 'm', ...
            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        objShapeModel.dObjectReferenceSize = 939.0/2 * 1E3; % [m]
        objShapeModel.charTargetUnitOutput = 'm';
            
        % Define shape matrix in principal TF
        dABC = 1E3 * [483.1, 481.0, 445.9];
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dABC(1)^2, 1/dABC(2)^2, 1/dABC(3)^2]);

    case "Dydimos"
        error('Not implemented yet')

    case "Eros"
        error('Not implemented yet')
    
    case "NotDefined"
        objShapeModel = CShapeModel('file_obj', "", 'km', 'm', ...
            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);
        objShapeModel.charTargetUnitOutput = 'm';
        
        charBlenderModelPath = "";
        charBlenderPyInterfacePath = "";
        charStartBlenderServerScriptPath = "";

    otherwise
        if options.bLoadShapeModel == false
            warning('Invalid or unavailable scenario, but no loading of shape required. Returning empty shape model.');
            objShapeModel = CShapeModel('file_obj', "", 'km', 'm', ...
                            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);
        else
            error('Data for selected scenarios are either not setup or unavailable.');
        end
end

charBlenderPyInterfacePath          = fullfile(charBpyRootPath, "server_api/BlenderPy_UDP_TCP_interface_withCaching.py" );
charStartBlenderServerScriptPath    = fullfile(charBpyRootPath, "server_api/StartBlenderServer.sh");

if nargout > 1
    % Define output struct if required
    strBpyCommManagerPaths.charBlenderModelPath = charBlenderModelPath;
    strBpyCommManagerPaths.charBlenderPyInterfacePath = charBlenderPyInterfacePath;
    strBpyCommManagerPaths.charStartBlenderServerScriptPath = charStartBlenderServerScriptPath;
end
cd(charCallDir);

assert(objShapeModel.bHasData_ || options.bLoadShapeModel == false, 'ERROR: loading of mesh model has failed. Check input path first; report issue if it persists.')
end

