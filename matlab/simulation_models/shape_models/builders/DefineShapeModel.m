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
    options.bVertFacesOnly          (1,1) {islogical} = true;
    options.bLoadShapeModel         (1,1) {islogical} = true;
    options.charOutputLengthUnits   (1,:) char {mustBeMember(options.charOutputLengthUnits, ["km", "m"])} = "m"
    options.bLoadModifiedVariant    (1,1) logical = false;
end
%% SIGNATURE
% [objShapeModel, strBpyCommManagerPaths] = DefineShapeModel(enumTargetName, charDataRootPath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function defining a general purpose shape model object from the specified target name as CShapeModel in 
% SimulationGears repository (https://github.com/PeterCalifano/SimulationGears_for_SpaceNav). 
% Paths to models for BlenderPyCommManager class are also defined (ACHTUNG: currently HARDCODED).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumTargetName                    (1,:) {mustBeA(enumTargetName, ["string", "char", "EnumScenarioName"]), ...
%                                          mustBeMember(enumTargetName, ["Apophis", "Itokawa", "Bennu", "Moon"])}
% charDataRootPath                  (1,:) string = fullfile(getenv("HOME"), "devDir/nav-backend/simulationCodes/data/SPICE_kernels")
% charBpyRootPath                   (1,:) string = fullfile(getenv("HOME"), "devDir/rendering-sw/corto_PeterCdev")
% options.bVertFacesOnly            (1,1) {islogical} = true;
% options.bLoadShapeModel           (1,1) {islogical} = true;
% options.charOutputLengthUnits     (1,:) char {mustBeMember(options.charOutputLengthUnits, ["km", "m"])} = "m"
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objShapeModel
% strBpyCommManagerPaths
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-04-2025    Pietro Califano     Update of paths definition
% 03-05-2025    Pietro Califano     Minor revision, add Moon setup
% 25-08-2025    Pietro Califano     Extend function to work with km and meters based on input options
% 31-08-2025    Pietro Califano     Define ellipsoidal model for all available bodies
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

if strcmpi(options.charOutputLengthUnits, "km")
    dLengthScaleCoeff       = 1.0;
    dInvLengthScaleCoeff    = 1000.0;

elseif strcmpi(options.charOutputLengthUnits, "m")
    dLengthScaleCoeff        = 1000.0;
    dInvLengthScaleCoeff     = 1/1000.0;
end

switch enumTargetName
    case "Apophis"
        % DEVNOTE: currently assumes rcs-1 simulator loader
        bDO_NOT_INIT_RCS1_ENV = true;
        LoadUserConfig; % Required for some variables (may be removed)
        % charBlenderModelPath                = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-C/blender/ApophisParticles.blend");
        %charBlenderModelPath                = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-C/blender/Apophis_RGB.blend");
        % charBlenderModelPath = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-C/blender/Apophis_RGB_Centered_Elongated_500m.blend");

        % Define shape model object
        if not(options.bLoadModifiedVariant)
            % charBlenderModelPath   = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-C/blender/Apophis_RGB_smoothed.blend");
            % charShapeModelObjPath_ = fullfile(path_to_shape_models, "apophis_v233s7_vert2_new.mod.obj");
            charBlenderModelPath   = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-C/blender/Apophis_RGB_Centered_MeanSize.blend");
            charShapeModelObjPath_ = fullfile(path_to_shape_models, "Apophis_RGB_Centered_MeanSize.obj");
        else
            charBlenderModelPath   = fullfile(getenv("HOME"), "devDir/projects-DART/data/rcs-1/phase-C/blender/Apophis_RGB_Centered_Elongated_550m.blend");
            charShapeModelObjPath_ = fullfile(path_to_shape_models, "Apophis_RGB_Centered_Elongated_550m.obj");
        end

        objShapeModel = CShapeModel('file_obj', charShapeModelObjPath_, ...
            'km', options.charOutputLengthUnits, options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        try
            ui32ID = 20099942;
            objShapeModel.dObjectReferenceSize  = dLengthScaleCoeff * mean(cspice_bodvrd(num2str(ui32ID),'RADII',3)); % [m] ACHTUNG: Value used for Gravity SH expansion!
        catch
            warning('Fetch of Apophis data from kernels failed. Fallback to hardcoded data...')
            objShapeModel.dObjectReferenceSize  = dLengthScaleCoeff * 0.175930344; % [m] ACHTUNG: Value used for Gravity SH expansion!
        end
        
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;

        dEllipsoidABC = dLengthScaleCoeff * [0.19884391053956174, 0.15921442216621817, 0.14822745272788257]; % [m] or [km]
        % DEVNOTE: From Paolo's fitting + conversion from Inertia Tensor with unitary density to semi-axes
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dEllipsoidABC(1)^2, 1/dEllipsoidABC(2)^2, 1/dEllipsoidABC(3)^2]); % Ellipsoid inverse shape matrix entries [1/a2, 1/b2, 1/c2];

    case "Itokawa"

        % Define blender model path
        if not(options.bLoadModifiedVariant)

            charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S2_Itokawa/S2_Itokawa.blend");

            % Define shape model object
            charKernelname = fullfile(charDataRootPath, 'Itokawa/dsk/hay_a_amica_5_itokawashape_v1_0_64q.bds');

            objShapeModel = CShapeModel('cspice', charKernelname, 'km', options.charOutputLengthUnits, ...
                options.bVertFacesOnly, char(enumTargetName));
        else
            % Variant model
            charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S2_Itokawa/S2_Itokawa_modified.blend");

            % Define shape model object
            objShapeModel = CShapeModel('file_obj', fullfile(charBpyRootPath, "data/scenarios/S2_Itokawa/S2_Itokawa_modified.obj"), ...
                                         'km', ...
                                         options.charOutputLengthUnits, ...
                                         options.bVertFacesOnly, ...
                                         char(enumTargetName), ...
                                         false);
        end


        objShapeModel.dObjectReferenceSize = dLengthScaleCoeff * 0.161915; % [m] or [km]
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;

        dEllipsoidABC = dLengthScaleCoeff * 1E-3 * 0.5 * [535, 294, 209]; % [m] or [km]
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dEllipsoidABC(1)^2, 1/dEllipsoidABC(2)^2, 1/dEllipsoidABC(3)^2]); % Ellipsoid inverse shape matrix entries [1/a2, 1/b2, 1/c2];

    case "Bennu"

        % Define blender model path
        charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S4_Bennu/S4_Bennu.blend");

        % Define shape model object
        % charKernelname = fullfile(charDataRootPath, 'Bennu_OREx/dsk/bennu_l_00050mm_alt_ptm_5595n04217_v021.bds'); %  Too large!
        charKernelname = fullfile(charDataRootPath, 'Bennu_OREx/dsk/bennu_g_03170mm_spc_obj_0000n00000_v020.bds');
        % charKernelname = fullfile(charDataRootPath, 'Bennu_OREx/dsk/bennu_g_01680mm_alt_obj_0000n00000_v021.bds');

        objShapeModel = CShapeModel('cspice', charKernelname, 'km', options.charOutputLengthUnits, ...
                        options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        % Assign reference radius
        objShapeModel.dObjectReferenceSize = 245.03 / dInvLengthScaleCoeff; % [m]
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;

        % Define shape matrix in principal TF
        dEllipsoidABC = dLengthScaleCoeff * 1E-3 * [3395428, 3395428, 3377678]; % [m] or [km]
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dEllipsoidABC(1)^2, 1/dEllipsoidABC(2)^2, 1/dEllipsoidABC(3)^2]);


    case "Moon"

        % Define blender model path
        charBlenderModelPath                = fullfile(charBpyRootPath, "data/scenarios/S6_Moon/S6_Moon.blend");

        % Define shape model object
        charObjModelFilePath = fullfile(charBpyRootPath, "data/scenarios/S6_Moon/Moon.obj");

        objShapeModel = CShapeModel('file_obj', charObjModelFilePath, 'km', options.charOutputLengthUnits, ...
                                options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        objShapeModel.dObjectReferenceSize = dLengthScaleCoeff * 1737.42; % [m] or [km]
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;

        % Define shape matrix in principal TF
        dEllipsoidABC = objShapeModel.dObjectReferenceSize * ones(1,3); % [m] or [km]
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dEllipsoidABC(1)^2, 1/dEllipsoidABC(2)^2, 1/dEllipsoidABC(3)^2]);

    case "Mars"

        charObjModelFilePath = ""; % None for now

        objShapeModel = CShapeModel('file_obj', charObjModelFilePath, 'km', options.charOutputLengthUnits, ...
                            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        objShapeModel.dObjectReferenceSize = dLengthScaleCoeff * 3386.2; % [m] or [km]
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;

        % Define shape matrix in principal TF
        dEllipsoidABC = dLengthScaleCoeff * 1E-3 * [3395428, 3395428, 3377678]; % [m] or [km]
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dEllipsoidABC(1)^2, 1/dEllipsoidABC(2)^2, 1/dEllipsoidABC(3)^2]);

        
    case "Ceres" 
        charObjModelFilePath = ""; % None for now

        objShapeModel = CShapeModel('file_obj', charObjModelFilePath, 'km', options.charOutputLengthUnits, ...
                                options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);

        objShapeModel.dObjectReferenceSize = dLengthScaleCoeff * 939.0/2; % [m] or [km]
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;
            
        % Define shape matrix in principal TF
        dEllipsoidABC = dLengthScaleCoeff * 1E-3 * [483.1, 481.0, 445.9]; % [m] or [km]
        objShapeModel.dTargetShapeMatrix_OF = diag([1/dEllipsoidABC(1)^2, 1/dEllipsoidABC(2)^2, 1/dEllipsoidABC(3)^2]);

    case "Dydimos"
        error('Not implemented yet')

    case "Eros"
        error('Not implemented yet')
    
    case "NotDefined"
        objShapeModel = CShapeModel('file_obj', "", 'km', options.charOutputLengthUnits, ...
            options.bVertFacesOnly, char(enumTargetName), options.bLoadShapeModel);
        objShapeModel.charTargetUnitOutput = options.charOutputLengthUnits;
        
        charBlenderModelPath = "";
        charBlenderPyInterfacePath = "";
        charStartBlenderServerScriptPath = "";

    otherwise
        if options.bLoadShapeModel == false
            warning('Invalid or unavailable scenario, but no loading of shape required. Returning empty shape model.');
            objShapeModel = CShapeModel('file_obj', "", 'km', options.charOutputLengthUnits, ...
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

