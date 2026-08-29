classdef CShapeModel < CBaseDatastruct
    %% DESCRIPTION
    % Unified object class representing triangular meshes in the standard format (vertices, triangles),
    % where vertices are a set of 3D points and triangles a set of indices indicating which vertices form
    % each triangle. The `file_obj` loading method preserves the established OBJ behavior, while
    % `file_mesh` loads repaired geometry from OBJ or STL through the shared host-side reader.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 05-10-2024    Pietro Califano     First implementation completed.
    % 13-02-2025    Pietro Califano     Update implementation to inherit from CBaseDatastruct (handle)
    % 03-05-2025    Pietro Califano     Add implementation to support loading from and writing to obj file
    % 10-11-2025    Pietro Califano     Minor bug fixes
    % 22-04-2026    Pietro Califano     Extend class with utilities to build SH and Polyhedral gravity from
    %                                   shape model with known density and mass
    % 24-04-2026    Pietro Califano     Add mesh simplification utility and load-time keep-fraction option
    % 01-07-2026    Pietro Califano     Add workspace MICE resolution and support OBJ v//vn face syntax.
    % 28-08-2026    Pietro Califano     Add validated general OBJ/STL geometry loading.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % LoadShapeMesh for `file_mesh` and explicit OBJ repair.
    % -------------------------------------------------------------------------------------------------------------


    properties (SetAccess = public, GetAccess = public)
        dTargetShapeMatrix_OF (3,3) double {mustBeNumeric} = zeros(3,3,'double')
        dObjectReferenceSize  (1,1) double {mustBeNumeric}  = zeros(1,1)
        charTargetUnitOutput = 'm';
    end

    properties (SetAccess = protected, GetAccess = public)

        bHasData_ = false;
        ui32triangVertexPtr = []; % Assumed as [3, N] array
        dVerticesPos = [];        % Assumed as [3, N] array
        dShapeRadius (1,1) double {mustBeNumeric, mustBeFinite} = 0.0;
        ui32NumOfVertices = uint32(0);
        unitScaler = 1;
        dMeshSimplifyFactor (1,1) double {mustBeFinite} = 1.0;

        % Optional data
        dTexCoords = [];
        ui32TrianglesTexIndex = [];
        dNormals = [];
        ui32TrianglesNormalsIndex = [];
        charModelName = ""

        % Polyhedron gravity preprocessing cache (populated by BuildPolyhedronGravityData)
        bHasGravityData_ = false;
        ui32GravEdgeVertexIds  = [];  % [nEdges x 2, uint32]
        dGravEdgeDyadics       = [];  % [3 x 3 x nEdges, double]
        dGravFaceDyadics       = [];  % [3 x 3 x nFaces, double]

        % Spherical harmonics gravity cache (populated explicitly through setSphericalHarmonicsGravityData)
        bHasSpherHarmonicsGravityData_ = false;
        strSphHarmonicsGravityData_ struct = struct()

    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = CShapeModel(enumLoadingMethod, ...
                varInputData, ...
                charInputUnit, ...
                charTargetUnitOutput, ...
                bVertFacesOnly, ...
                charModelName, ...
                bLoadShapeModel, ...
                options)
            arguments
                enumLoadingMethod       (1,:) string {mustBeA(enumLoadingMethod, ["string", "char"]), ...
                    mustBeMember(enumLoadingMethod, ["mat", "cspice", "struct", "file_obj", "file_mesh"])} = "file_obj"
                varInputData            (1,:) = []
                charInputUnit           {mustBeA(charInputUnit, ["string", "char", "EnumLengthUnits"])} = 'km'
                charTargetUnitOutput    {mustBeA(charTargetUnitOutput, ["string", "char", "EnumLengthUnits"])} = 'm'
                bVertFacesOnly          (1,1) logical = true;
                charModelName           (1,:) char = ""
                bLoadShapeModel         (1,1) logical = true;
            end
            arguments
                options.dMeshSimplifyFactor (1,1) double {mustBeFinite} = 1.0
            end

            % For default (placeholder) construction
            if nargin < 1
                return
            end

            charInputUnit = char(EnumLengthUnits.toString(charInputUnit));
            self.charTargetUnitOutput = char(EnumLengthUnits.toString(charTargetUnitOutput));
            self.bDefaultConstructed  = false;
            self.dMeshSimplifyFactor  = min(max(double(options.dMeshSimplifyFactor), 0.0), 1.0);

            % Determine scaling to match length unit
            if (strcmpi(charInputUnit, 'm') && strcmpi(self.charTargetUnitOutput, 'm')) || ...
                    strcmpi(charInputUnit, 'km') && strcmpi(self.charTargetUnitOutput, 'km')
                self.unitScaler = 1;

            elseif strcmpi(charInputUnit, 'km') && strcmpi(self.charTargetUnitOutput, 'm')
                self.unitScaler = 1000;

            elseif (strcmpi(charInputUnit, 'm') && strcmpi(self.charTargetUnitOutput, 'km'))
                self.unitScaler = 1E-3;
            end

            if bLoadShapeModel
                % Call input specific loading function
                if strcmpi(enumLoadingMethod, 'cspice')

                    if strcmpi(charInputUnit, 'm')
                        warning("CSpice usually provides models' vertices in km, but 'meters' has been specified. Make sure this is correct.")
                    end

                    [self] = self.LoadModelFromSPICE(varInputData);

                elseif strcmpi(enumLoadingMethod, 'mat')
                    [self] = self.LoadModelFromMat(varInputData);

                elseif strcmpi(enumLoadingMethod, 'struct')
                    [self] = self.LoadModelFromStruct(varInputData);

                elseif strcmpi(enumLoadingMethod, 'file_obj')
                    [self] = self.LoadModelFromObj_(varInputData, bVertFacesOnly);

                elseif strcmpi(enumLoadingMethod, 'file_mesh')
                    [self] = self.LoadModelFromMeshFile_(varInputData, bVertFacesOnly);

                end
            end

            % Write model name
            self.charModelName = charModelName;

            % Get number of vertices
            self.ui32NumOfVertices = size(self.dVerticesPos, 2);

            if self.ui32NumOfVertices > 0
                % Update unit scaling
                self.dVerticesPos = self.unitScaler * self.dVerticesPos;

                if self.dMeshSimplifyFactor < 1.0
                    [self, ~] = self.SimplifyMesh(100.0 * (1.0 - self.dMeshSimplifyFactor));
                end
            end

            self = self.UpdateDerivedGeometry_();
        end

        %% GETTERS
        % Get shape model vertices and indices
        function [strData] = getShapeStruct(self)
            strData = struct();

            if self.bHasData_ == true
                strData.ui32triangVertexPtr = self.ui32triangVertexPtr;
                strData.dVerticesPos = self.dVerticesPos;
                strData.dShapeRadius = self.dShapeRadius;
            else
                warning('No model was loaded. Returning empty struct.')
            end
        end

        function [ui32NumOfVertices] = getNumOfVertices(self)
            if self.bHasData_ == false
                warning('Shape model was not initialized. Returning 0 as number of vertices.')
            end
            ui32NumOfVertices = self.ui32NumOfVertices;
        end

        function bool = hasData(self)
            bool = self.bHasData_;
        end

        %% PUBLIC METHODS
        function [objFig] = visualizeMesh(self)
            arguments
                self
            end

            objFig = gobjects(1,1);
            if self.bHasData_
                objFig = figure;
                hold on
                scatter3(self.dVerticesPos(1,:), self.dVerticesPos(2,:), self.dVerticesPos(3,:), ...
                    'black', 'filled');

                % Options
                objCurrentAx = gca();

                objCurrentAx.XMinorTick = 'on';
                objCurrentAx.YMinorTick = 'on';
                objCurrentAx.LineWidth = 1.05;
                objCurrentAx.LineWidth = 1.05;
                ylim('tickaligned');
                xlim('tight')

                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                axis equal

            else
                warning('Visualization method called without any loaded data. Nothing will be shown.')
            end

        end

        function [self, strReductionStats] = SimplifyMesh(self, dReductionPercent)
            %% DESCRIPTION
            % Reduces the number of faces in the current triangular mesh
            % using MATLAB's reducepatch. The requested input is expressed
            % as percentage of face reduction, while the achieved vertex
            % and face reductions are returned in the output stats struct.
            % A value of 100 clears the mesh completely.
            %
            % ACHTUNG: CShapeModel currently has value-class semantics
            % through CBaseDatastruct, so the updated object must be
            % captured by the caller.
            % -------------------------------------------------------------------------------------------------------------
            arguments
                self
                dReductionPercent (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(dReductionPercent, 0), mustBeLessThanOrEqual(dReductionPercent, 100)}
            end

            assert(self.bHasData_, 'CShapeModel:NoData', ...
                'Shape model must be loaded before calling SimplifyMesh().');

            ui32NumFacesBefore = uint32(size(self.ui32triangVertexPtr, 2));
            ui32NumVertsBefore = uint32(size(self.dVerticesPos, 2));

            % Initialize reduction stats struct with pre-reduction values and requested reduction
            strReductionStats = struct( ...
                'ui32NumFacesBefore', ui32NumFacesBefore, ...
                'ui32NumFacesAfter', ui32NumFacesBefore, ...
                'ui32NumVerticesBefore', ui32NumVertsBefore, ...
                'ui32NumVerticesAfter', ui32NumVertsBefore, ...
                'dRequestedReductionPercent', dReductionPercent, ...
                'dAppliedKeepFraction', 1.0 - dReductionPercent / 100.0, ...
                'dAchievedFaceReductionPercent', 0.0, ...
                'dAchievedVertexReductionPercent', 0.0);

            if dReductionPercent == 0 || ui32NumFacesBefore == 0
                self = self.UpdateDerivedGeometry_();
                return
            end

            if dReductionPercent == 100
                self.ui32triangVertexPtr = zeros(3, 0, 'uint32');
                self.dVerticesPos        = zeros(3, 0, 'double');

            else
                % Execute reduction and update mesh data
                dKeepFraction = strReductionStats.dAppliedKeepFraction;
                [dFacesReduced, dVertsReduced] = reducepatch( ...
                    double(self.ui32triangVertexPtr'), self.dVerticesPos', dKeepFraction);

                self.ui32triangVertexPtr = uint32(dFacesReduced');
                self.dVerticesPos        = dVertsReduced';
            end

            % Fill in reduction stats with achieved reductions
            self = self.UpdateDerivedGeometry_();

            strReductionStats.ui32NumFacesAfter = uint32(size(self.ui32triangVertexPtr, 2));
            strReductionStats.ui32NumVerticesAfter = self.ui32NumOfVertices;
            strReductionStats.dAchievedFaceReductionPercent = 100.0 * ...
                (1.0 - double(strReductionStats.ui32NumFacesAfter) / double(ui32NumFacesBefore));
            strReductionStats.dAchievedVertexReductionPercent = 100.0 * ...
                (1.0 - double(strReductionStats.ui32NumVerticesAfter) / double(ui32NumVertsBefore));

            if ~isempty(self.dTexCoords) || ~isempty(self.ui32TrianglesTexIndex) || ...
                    ~isempty(self.dNormals) || ~isempty(self.ui32TrianglesNormalsIndex)

                warning('CShapeModel:SimplifyMeshDropsAuxiliaryObjData', ...
                    ['SimplifyMesh() only updates mesh geometry. Existing texture and normal ', ...
                     'data are being cleared because their indices are no longer valid after decimation.']);
                
                self.dTexCoords = [];
                self.ui32TrianglesTexIndex = [];
                self.dNormals = [];
                self.ui32TrianglesNormalsIndex = [];
            end

            % Geometry-dependent caches become stale after decimation.
            self.bHasGravityData_ = false;
            self.ui32GravEdgeVertexIds = [];
            self.dGravEdgeDyadics = [];
            self.dGravFaceDyadics = [];

            self.bHasSpherHarmonicsGravityData_ = false;
            self.strSphHarmonicsGravityData_ = struct();
        end

        function self = BuildPolyhedronGravityData(self)
            %% DESCRIPTION
            % Precomputes and caches the geometric data (edge vertex IDs, edge
            % dyadic tensors, face dyadic tensors) required by EvalPolyhedronGrav.
            % Must be called after the shape model is loaded. The results are
            % stored in the object properties and can be retrieved via
            % getPolyhedronGravityData().
            % ACHTUNG: Vertices and faces are stored internally as [3,N]. This
            % method transposes them to [N,3] for ComputePolyhedronFaceEdgeData.
            %
            % ACHTUNG: CShapeModel currently has value-class semantics
            % through CBaseDatastruct, so the updated object must be
            % captured by the caller.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ComputePolyhedronFaceEdgeData
            % -------------------------------------------------------------------------------------------------------------
            arguments
                self
            end

            assert(self.bHasData_, 'CShapeModel:NoData', ...
                'Shape model must be loaded before building gravity data.');

            % Transpose from internal [3,N] to [N,3] convention
            dVerticesRows = self.dVerticesPos';
            ui32FacesRows = uint32(self.ui32triangVertexPtr');

            [self.ui32GravEdgeVertexIds, self.dGravEdgeDyadics, self.dGravFaceDyadics] = ...
                ComputePolyhedronFaceEdgeData(ui32FacesRows, dVerticesRows);

            self.bHasGravityData_ = true;
        end

        function [ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, ui32FacesRows, dVerticesRows] = getPolyhedronGravityData(self)
            %% DESCRIPTION
            % Returns the cached polyhedron gravity preprocessing data and
            % the mesh arrays in [N,3] row-major convention expected by
            % EvalPolyhedronGrav. Call BuildPolyhedronGravityData() first.
            % -------------------------------------------------------------------------------------------------------------
            arguments
                self
            end

            assert(self.bHasGravityData_, 'CShapeModel:NoGravityData', ...
                'Gravity data not available. Call BuildPolyhedronGravityData() first.');

            ui32EdgeVertexIds = self.ui32GravEdgeVertexIds;
            dEdgeDyadics      = self.dGravEdgeDyadics;
            dFaceDyadics      = self.dGravFaceDyadics;
            ui32FacesRows     = uint32(self.ui32triangVertexPtr');
            dVerticesRows     = self.dVerticesPos';
        end

        function self = setSphericalHarmonicsGravityData(self, strSHgravityData)
            %% DESCRIPTION
            % Stores previously computed spherical harmonics gravity data in
            % the object cache. Use the static
            % BuildSphericalHarmonicsGravityData() method to generate the
            % struct, then call this setter explicitly.
            %
            % ACHTUNG: CShapeModel currently has value-class semantics
            % through CBaseDatastruct, so the updated object must be
            % captured by the caller.
            % -------------------------------------------------------------------------------------------------------------
            arguments
                self
                strSHgravityData (1,1) struct
            end

            % Validate that the input struct has the required fields
            cellRequiredFields = {'dCSlmCoeffCols', 'ui32MaxDegree', 'dGravParam', ...
                'dBodyRadiusRef', 'dDensity', 'dGravConst', 'strFitStats'};

            for idField = 1:numel(cellRequiredFields)
                assert(isfield(strSHgravityData, cellRequiredFields{idField}), ...
                    'CShapeModel:InvalidSHGravityData', ...
                    'Missing required field "%s" in SH gravity data struct.', ...
                    cellRequiredFields{idField});
            end

            % Set data and flag
            self.strSphHarmonicsGravityData_ = strSHgravityData;
            self.bHasSpherHarmonicsGravityData_ = true;
        end

        function strSHgravityData = getSphericalHarmonicsGravityData(self)
            %% DESCRIPTION
            % Returns the cached spherical harmonics gravity data previously
            % stored through setSphericalHarmonicsGravityData().
            % -------------------------------------------------------------------------------------------------------------
            arguments
                self
            end

            assert(self.bHasSpherHarmonicsGravityData_, 'CShapeModel:NoSHgravityData', ...
                'Spherical harmonics gravity data not available. Compute and set it first.');

            strSHgravityData = self.strSphHarmonicsGravityData_;
        end

        function [self, strSHgravityData] = BuildAndSetSphericalHarmonicsGravityData(self, ui32MaxDegree, options)
            %% DESCRIPTION
            % Builds spherical harmonics gravity data from the loaded mesh
            % and stores it in the object cache. This is the mutating
            % instance-level companion to the static compute-only
            % BuildSphericalHarmonicsGravityData() method.
            %
            % ACHTUNG: CShapeModel currently has value-class semantics
            % through CBaseDatastruct, so the updated object must be
            % captured by the caller.
            % -------------------------------------------------------------------------------------------------------------
            arguments
                self
                ui32MaxDegree                  (1,1) uint32 = uint32(4)
                options.dGravParam             (1,1) double = NaN
                options.dDensity               (1,1) double = NaN
                options.dGravConst             (1,1) double = NaN
                options.dBodyRadiusRef         (1,1) double = NaN
                options.ui32MaxFitIterations   (1,1) uint32 = uint32(5)
                options.charMode               (1,:) string {mustBeA(options.charMode, ["string", "char"]), ...
                    mustBeMember(options.charMode, ["auto", "registry", "compute", "none"])} = "auto"
            end

            if strcmpi(options.charMode, "none")
                strSHgravityData = struct();
                return
            end

            bHasExplicitPhysicalInputs = isfinite(options.dGravParam) || ...
                isfinite(options.dDensity) || isfinite(options.dBodyRadiusRef);

            if any(strcmpi(options.charMode, ["auto", "registry"])) && ...
                    (~bHasExplicitPhysicalInputs || strcmpi(options.charMode, "registry"))
                
                try
                    % Try to get registry data first
                    [strRegistrySHgravityData, strRegistrySHmeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                        self.charModelName, ui32MaxDegree, string(self.charTargetUnitOutput));
                
                catch objException
                    
                    if strcmp(objException.identifier, 'CScenarioRegistry:UnsupportedScenario') && strcmpi(options.charMode, "auto")
                        
                        strRegistrySHmeta = struct('bHasHardcodedCoefficients', false, 'ui32HardcodedMaxDegree', uint32(0));
                        strRegistrySHgravityData = struct();
                    else
                        rethrow(objException)
                    end
                end

                if strRegistrySHmeta.bHasHardcodedCoefficients && ...
                        ui32MaxDegree <= strRegistrySHmeta.ui32HardcodedMaxDegree
                    
                    % Cache registry data on the object and return it
                    self = self.setSphericalHarmonicsGravityData(strRegistrySHgravityData);
                    strSHgravityData = self.getSphericalHarmonicsGravityData();
                    return
                end

                if strcmpi(options.charMode, "registry")
                    error('CShapeModel:RegistrySHUnavailable', ...
                        ['Registry SH data for %s are unavailable at requested degree %u. ' ...
                         'Available hardcoded max degree is %u.'], ...
                        string(self.charModelName), ui32MaxDegree, strRegistrySHmeta.ui32HardcodedMaxDegree);
                end
            end

            dGravParam = options.dGravParam;
            dDensity   = options.dDensity;

            % Get gravity defaults for model and target unit if not provided as input
            if ~isfinite(dGravParam) && ~isfinite(dDensity)

                strGravityDefaults = GetShapeModelScenarioGravityDefaults( ...
                    self.charModelName, self.charTargetUnitOutput);
            
                dGravParam = strGravityDefaults.dGravParam;
                dDensity   = strGravityDefaults.dDensity;
            end

            % Build SH gravity data struct and store it in the object cache
            strSHgravityData = CShapeModel.BuildSphericalHarmonicsGravityData(self, ui32MaxDegree, ...
                                                                            dGravParam=dGravParam, ...
                                                                            dDensity=dDensity, ...
                                                                            dGravConst=options.dGravConst, ...
                                                                            dBodyRadiusRef=options.dBodyRadiusRef, ...
                                                                            ui32MaxFitIterations=options.ui32MaxFitIterations);

            self = self.setSphericalHarmonicsGravityData(strSHgravityData);
            strSHgravityData = self.getSphericalHarmonicsGravityData();
        end

    end

    methods (Access = protected)
        %% PROTECTED METHODS
        function [self] = LoadModelFromSPICE(self, charKernelName)
            % ACHTUNG: output model dimensions are determined by targetLenghtUnit attribute. Default is meters.
            arguments
                self
                charKernelName (1,:) char
            end

            % Check if SPICE is available
            if isempty(which('cspice_furnsh'))
                CShapeModel.TryAddMiceFromWorkspace_();
            end

            if isempty(which('cspice_furnsh'))
                error('CShapeModel:CSPICEUnavailable', ...
                    ['SPICE DSK shape loading requires NAIF MICE on the MATLAB path. ' ...
                     'Set WS_SIMGEARS or WS_NAVSYS to a workspace containing mice/, ' ...
                     'install/add MICE before loading %s, or set bLoadShapeModel=false for metadata-only use.'], ...
                    string(charKernelName));
            end

            % Check that kernel is loaded else, try to load it
            % TODO
            % if kernelNotLoaded == true
            cspice_furnsh( char(charKernelName) );
            % end

            checkIfModelAlreadyLoaded(self);

            % Get info from kernel
            [~, ~, kernelhandle] = cspice_kinfo(char(charKernelName));
            dladsc = cspice_dlabfs(kernelhandle);

            % Get number of vertices and triangles
            [nVertices, nTriangles] = cspice_dskz02(kernelhandle, dladsc);
            % Get triangles from SPICE (p: plates)
            ui32TrianglesVertices = cspice_dskp02(kernelhandle, dladsc, 1, nTriangles);

            % Get vertices from SPICE (v: vertices)
            dModelVertices = cspice_dskv02(kernelhandle, dladsc, 1, nVertices);

            % Assign data to object attributes
            self.ui32triangVertexPtr = ui32TrianglesVertices;
            self.dVerticesPos        = dModelVertices;
            self = self.UpdateDerivedGeometry_();

            self.bHasData_ = true;
        end

        function [self] = LoadModelFromStruct(self, strShapeModel)
            % ACHTUNG: input struct is assumed to have fields with the same name as self.strShapeModel
            checkIfModelAlreadyLoaded(self);

            cellFieldnames = fieldnames(strShapeModel);
            assert( not(isempty(cellFieldnames{contains(cellFieldnames, '32')} )), 'ERROR: automatic fieldnames resolution failed.');
            assert( not(isempty(cellFieldnames{contains(cellFieldnames, 'dVert')} )), 'ERROR: automatic fieldnames resolution failed.');

            self.ui32triangVertexPtr = uint32(strShapeModel.(cellFieldnames{contains(cellFieldnames, '32')} ));
            self.dVerticesPos        = double(strShapeModel.(cellFieldnames{contains(cellFieldnames, 'dVert')} ));
            self = self.UpdateDerivedGeometry_();

            self.bHasData_ = true;
        end

        function [self] = LoadModelFromMat(self, charPathToMatfile)
            % ACHTUNG: input mat is assumed to contain a struct with fields with the same name as self.strShapeModel
            if exist(charPathToMatfile, "file")

                checkIfModelAlreadyLoaded(self);

                strData = load(charPathToMatfile); % DEVNOTE: check that structdata is the desired struct
                [self] = LoadModelFromStruct(self, strData);

                self.bHasData_ = true;
            else
                error('Mat file not found. Check path.')
            end

        end

        function [self] = LoadModelFromObj_(self, charObjFilePath, bVertFacesOnly)

            checkIfModelAlreadyLoaded(self);

            [self.ui32triangVertexPtr, self.dVerticesPos, ...
                self.dTexCoords, self.ui32TrianglesTexIndex, ...
                self.dNormals, self.ui32TrianglesNormalsIndex] = CShapeModel.LoadModelFromObj(charObjFilePath, bVertFacesOnly);

            % The legacy parser already returns column-major geometry; transpose only auxiliary data.
            if not(bVertFacesOnly)
                self.dTexCoords = transpose(self.dTexCoords);
                self.ui32TrianglesTexIndex = transpose(self.ui32TrianglesTexIndex);
                self.dNormals = transpose(self.dNormals);
                self.ui32TrianglesNormalsIndex = transpose(self.ui32TrianglesNormalsIndex);
            end

            self = self.UpdateDerivedGeometry_();
            self.bHasData_ = true;

        end

        function self = LoadModelFromMeshFile_(self, charMeshFilePath, bVertFacesOnly)
            %% SIGNATURE
            % self = LoadModelFromMeshFile_(self, charMeshFilePath, bVertFacesOnly)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Load repaired geometry from a supported OBJ or STL mesh file.
            % Texture and normal payloads are intentionally outside this
            % geometry-only loading contract.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self:             Shape-model instance to populate.
            % charMeshFilePath: Path to a supported OBJ or STL mesh.
            % bVertFacesOnly:   Must be true because the shared reader owns geometry only.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self:             Populated shape-model instance.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 28-08-2026  Pietro Califano     Add shared repaired OBJ/STL geometry loading.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh.
            % -------------------------------------------------------------------------------------------------------------

            arguments(Input)
                self
                charMeshFilePath (1,:) {mustBeA(charMeshFilePath, ["string", "char"])}
                bVertFacesOnly (1,1) logical
            end

            arguments(Output)
                self
            end

            % Reject auxiliary payload before invoking the geometry-only shared reader.
            if ~bVertFacesOnly
                error('CShapeModel:MeshAuxiliaryDataUnsupported', ...
                    'file_mesh loading supports geometry only; set bVertFacesOnly to true.');
            end

            % Load repaired row-major geometry and adapt it to the established object layout.
            checkIfModelAlreadyLoaded(self);
            strShapeMesh = LoadShapeMesh(char(charMeshFilePath), bRepairMesh=true);
            self.ui32triangVertexPtr = transpose(strShapeMesh.ui32FaceVertexIds);
            self.dVerticesPos = transpose(strShapeMesh.dVerticesPos);

            % Refresh every property derived from geometry before publishing the loaded state.
            self = self.UpdateDerivedGeometry_();
            self.bHasData_ = true;
        end

        function [self] = UpdateDerivedGeometry_(self)
            % Method to update geometry-dependent attributes (number of vertices, shape radius) after loading or modifying the mesh. Called internally at the end of loading and simplification methods.
            self.ui32NumOfVertices = uint32(size(self.dVerticesPos, 2));

            if isempty(self.dVerticesPos)
                self.dShapeRadius = 0.0;
                return
            end

            dVertexNorms = vecnorm(self.dVerticesPos, 2, 1);
            dVertexNorms = dVertexNorms(isfinite(dVertexNorms));

            if isempty(dVertexNorms)
                self.dShapeRadius = 0.0;
            else
                self.dShapeRadius = mean(dVertexNorms);
            end
        end

        function checkIfModelAlreadyLoaded(self)
            if self.bHasData_
                warning('A shape model was already loaded before and is being overwritten.')
            end
        end

    end

    methods (Static, Access = public)

        function [objShapeModel, strSHgravityData] = BuildSphericalHarmonicsGravityDataFromObj( ...
                charObjFilePath, ui32MaxDegree, options)
            arguments
                charObjFilePath                 (1,:) string {mustBeA(charObjFilePath, ["string", "char"])}
                ui32MaxDegree                   (1,1) uint32
                options.charInputUnit          {mustBeA(options.charInputUnit, ["string", "char", "EnumLengthUnits"])} = "m"
                options.charTargetUnitOutput   {mustBeA(options.charTargetUnitOutput, ["string", "char", "EnumLengthUnits"])} = "m"
                options.bVertFacesOnly         (1,1) logical = true
                options.charModelName          (1,:) string {mustBeA(options.charModelName, ["string", "char"])} = ""
                options.dGravParam             (1,1) double = NaN
                options.dDensity               (1,1) double = NaN
                options.dGravConst             (1,1) double = NaN
                options.dBodyRadiusRef         (1,1) double = NaN
                options.ui32MaxFitIterations   (1,1) uint32 = uint32(5)
                options.dMeshSimplifyFactor    (1,1) double {mustBeFinite} = 1.0
                options.bCacheOnShapeModel     (1,1) logical = true
            end
            %% DESCRIPTION
            % Static compute-only utility that loads a shape model from a
            % Wavefront .obj file and builds spherical harmonics gravity
            % data from it.
            %
            % The method mirrors the compute-only style of
            % BuildSphericalHarmonicsGravityData(): it performs no
            % diagnostics plots and no workflow-side reporting. Use the
            % returned object directly, or cache the fitted SH data on it
            % by leaving options.bCacheOnShapeModel = true.
            % -------------------------------------------------------------------------------------------------------------

            if ~isfile(charObjFilePath)
                error('CShapeModel:ObjFileNotFound', ...
                    'Cannot find .obj file: %s', char(charObjFilePath));
            end

            % Determine model name from input path if not provided explicitly
            if options.charModelName == ""
                [~, charModelStem, ~] = fileparts(char(charObjFilePath));
                charModelName = string(charModelStem);
            else
                charModelName = options.charModelName;
            end

            % Clamp mesh simplification factor to [0,1]
            dMeshSimplifyFactor = min(max(double(options.dMeshSimplifyFactor), 0.0), 1.0);

            % Load shape model from obj file
            objShapeModel = CShapeModel("file_obj", charObjFilePath, options.charInputUnit, options.charTargetUnitOutput, ...
                                        options.bVertFacesOnly, char(charModelName), true, ...
                                        dMeshSimplifyFactor=dMeshSimplifyFactor);

            if options.bCacheOnShapeModel

                % Build SH gravity data and store it in the object cache
                objShapeModel = objShapeModel.BuildAndSetSphericalHarmonicsGravityData(ui32MaxDegree, ...
                    dGravParam=options.dGravParam, ...
                    dDensity=options.dDensity, ...
                    dGravConst=options.dGravConst, ...
                    dBodyRadiusRef=options.dBodyRadiusRef, ...
                    ui32MaxFitIterations=options.ui32MaxFitIterations);
                
                    strSHgravityData = objShapeModel.getSphericalHarmonicsGravityData();
            else
                % Just build SH gravity data without caching on the object
                strSHgravityData = CShapeModel.BuildSphericalHarmonicsGravityData(objShapeModel, ui32MaxDegree, ...
                                                        dGravParam=options.dGravParam, ...
                                                        dDensity=options.dDensity, ...
                                                        dGravConst=options.dGravConst, ...
                                                        dBodyRadiusRef=options.dBodyRadiusRef, ...
                                                        ui32MaxFitIterations=options.ui32MaxFitIterations);
            end
        end

        function strSHgravityData = BuildSphericalHarmonicsGravityData(objShapeModel, ui32MaxDegree, options)
            arguments
                objShapeModel                  (1,1) CShapeModel
                ui32MaxDegree                  (1,1) uint32
                options.dGravParam             (1,1) double = NaN
                options.dDensity               (1,1) double = NaN
                options.dGravConst             (1,1) double = NaN
                options.dBodyRadiusRef         (1,1) double = NaN
                options.ui32MaxFitIterations   (1,1) uint32 = uint32(5)
            end
            %% DESCRIPTION
            % Static compute-only utility to build spherical harmonics
            % gravity data from the mesh stored in a CShapeModel object.
            % The method does not mutate the object. Store the returned
            % struct explicitly through setSphericalHarmonicsGravityData().
            % -------------------------------------------------------------------------------------------------------------

            assert(objShapeModel.bHasData_, 'CShapeModel:NoData', ...
                'Shape model must be loaded before building SH gravity data.');

            ui32FacesRows = uint32(objShapeModel.ui32triangVertexPtr');
            dVerticesRows = objShapeModel.dVerticesPos';
            dGravConst = CShapeModel.ResolveGravConstForLengthUnit_( ...
                objShapeModel.charTargetUnitOutput, options.dGravConst);

            strSHgravityData = FitSpherHarmCoeffToPolyhedrGrav(ui32FacesRows, dVerticesRows, ui32MaxDegree, ...
                                                                options.dGravParam, ...
                                                                options.dDensity, ...
                                                                dGravConst, ...
                                                                options.dBodyRadiusRef, ...
                                                                options.ui32MaxFitIterations);
        end

        function [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
                ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = LoadModelFromObj( ...
                charObjFilePath, bVertFacesOnly, options)
            %% SIGNATURE
            % [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
            %  ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = ...
            %     LoadModelFromObj(charObjFilePath, bVertFacesOnly, options)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Load Wavefront OBJ data into the established column-major CShapeModel layout. The
            % default geometry-only path retains the optimized legacy parser. Explicit repair
            % delegates geometry to LoadShapeMesh; repair is incompatible with texture/normal
            % index loading because it changes vertex and face indices.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % charObjFilePath:       Path to a Wavefront OBJ file.
            % bVertFacesOnly:        Load geometry only when true.
            % options.bRepairMesh:   Weld duplicates and remove degenerate geometry when true.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % ui32TrianglesIndex:        Triangle vertex indices as 3-by-F uint32.
            % dVerticesCoords:           Vertex coordinates as 3-by-N double.
            % dTexCoords:                Texture coordinates as 2-by-T double when requested.
            % ui32TrianglesTexIndex:     Triangle texture indices as 3-by-F uint32.
            % dNormals:                  Normals as 3-by-Q double when requested.
            % ui32TrianglesNormalsIndex: Triangle normal indices as 3-by-F uint32.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 03-01-2025  Pietro Califano          First general OBJ implementation.
            % 16-11-2025  Pietro Califano, GPT-5   Use vectorized whole-file parsing.
            % 28-08-2026  Pietro Califano          Add explicit shared geometry repair.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh when options.bRepairMesh is true.
            % -------------------------------------------------------------------------------------------------------------

            arguments(Input)
                charObjFilePath (1,1) string {mustBeA(charObjFilePath, ["string", "char"])}
                bVertFacesOnly (1,1) logical = true
                options.bRepairMesh (1,1) logical = false
            end

            arguments(Output)
                ui32TrianglesIndex uint32
                dVerticesCoords double
                dTexCoords double
                ui32TrianglesTexIndex uint32
                dNormals double
                ui32TrianglesNormalsIndex uint32
            end

            %% Function code

            % Repair remaps geometry indices, so auxiliary index arrays cannot remain valid.
            if options.bRepairMesh && ~bVertFacesOnly
                error('CShapeModel:RepairWithAuxiliaryDataUnsupported', ...
                    ['Mesh repair changes geometry indices and cannot be combined with ', ...
                     'OBJ texture or normal index loading.']);
            end

            % Preserve the established public error identifiers before choosing a parser path.
            [~,~, charFileExt] = fileparts(charObjFilePath);

            if ~strcmpi(charFileExt, '.obj')
                error('LoadModelFromObj:InvalidExtension', 'Input file must have .obj extension.');
            end

            if not(isfile(charObjFilePath))
                error('LoadModelFromObj:FileNotFound', 'Cannot find file: %s', charObjFilePath);
            end

            % Keep repair opt-in and adapt the shared reader's rows to the legacy column layout.
            if options.bRepairMesh
                strShapeMesh = LoadShapeMesh(char(charObjFilePath), bRepairMesh=true);
                ui32TrianglesIndex = transpose(strShapeMesh.ui32FaceVertexIds);
                dVerticesCoords = transpose(strShapeMesh.dVerticesPos);
                dTexCoords = zeros(0, 2);
                ui32TrianglesTexIndex = zeros(0, 3, 'uint32');
                dNormals = zeros(0, 3);
                ui32TrianglesNormalsIndex = zeros(0, 3, 'uint32');
                return
            end

            % Retain the measured vectorized parser for the default, unrepaired OBJ contract.
            tic
            charFileText = fileread(charObjFilePath);

            % Vertex lines: 'v x y z'
            vMatch = regexp(charFileText, '^v\s+.*$', 'match', 'lineanchors');

            if ~isempty(vMatch)
                charVBlock = sprintf('%s\n', vMatch{:});           % Single big char array
                dVerticesCoords = sscanf(charVBlock, 'v %f %f %f\n', [3, Inf]);
            else
                dVerticesCoords = zeros(0,3);
            end

            % Texture-coordinate lines: 'vt u v'
            vtMatch = regexp(charFileText, '^vt\s+.*$', 'match', 'lineanchors');

            if ~isempty(vtMatch) && ~bVertFacesOnly
                charVTBlock = sprintf('%s\n', vtMatch{:});
                dTexCoords = sscanf(charVTBlock, 'vt %f %f\n', [2, Inf]);
            else
                dTexCoords = zeros(0,2);
            end

            % Normal lines: 'vn nx ny nz'
            vnMatch = regexp(charFileText, '^vn\s+.*$', 'match', 'lineanchors');

            if ~isempty(vnMatch) && ~bVertFacesOnly
                charVNBlock = sprintf('%s\n', vnMatch{:});
                dNormals = sscanf(charVNBlock, 'vn %f %f %f\n', [3, Inf]);
            else
                dNormals = zeros(0,3);
            end

            % Defaults
            ui32TrianglesIndex          = zeros(0,3,'uint32');
            ui32TrianglesTexIndex       = zeros(0,3,'uint32');
            ui32TrianglesNormalsIndex   = zeros(0,3,'uint32');

            fMatch = regexp(charFileText, '^f\s+.*$', 'match', 'lineanchors');
            if ~isempty(fMatch)
                charFBlock = sprintf('%s\n', fMatch{:});
                charFirstFace = string(strtrim(fMatch{1}));

                if ~isempty(regexp(charFirstFace, '^f\s+\d+//\d+', 'once'))
                    ui32AllFaceLines = sscanf(charFBlock, 'f %u//%u %u//%u %u//%u\n', [6, Inf]);
                    ui32AllFaceLines = uint32(ui32AllFaceLines);
                    ui32TrianglesIndex = ui32AllFaceLines(1:2:end, :);
                    if ~bVertFacesOnly
                        ui32TrianglesNormalsIndex = ui32AllFaceLines(2:2:end, :);
                    end

                elseif ~isempty(regexp(charFirstFace, '^f\s+\d+/\d+/\d+', 'once'))
                    ui32AllFaceLines = sscanf(charFBlock, 'f %u/%u/%u %u/%u/%u %u/%u/%u\n', [9, Inf]);
                    ui32AllFaceLines = uint32(ui32AllFaceLines);
                    ui32TrianglesIndex = ui32AllFaceLines(1:3:end, :);
                    if ~bVertFacesOnly
                        ui32TrianglesTexIndex = ui32AllFaceLines(2:3:end, :);
                        ui32TrianglesNormalsIndex = ui32AllFaceLines(3:3:end, :);
                    end

                elseif ~isempty(regexp(charFirstFace, '^f\s+\d+/\d+', 'once'))
                    ui32AllFaceLines = sscanf(charFBlock, 'f %u/%u %u/%u %u/%u\n', [6, Inf]);
                    ui32AllFaceLines = uint32(ui32AllFaceLines);
                    ui32TrianglesIndex = ui32AllFaceLines(1:2:end, :);
                    if ~bVertFacesOnly
                        ui32TrianglesTexIndex = ui32AllFaceLines(2:2:end, :);
                    end

                else
                    ui32AllFaceLines = sscanf(charFBlock, 'f %u %u %u\n', [3, Inf]);
                    ui32TrianglesIndex = uint32(ui32AllFaceLines);
                end
            end

            dElapsedTime = toc;
            fprintf("\nFile obj loaded in %.5g seconds\n", dElapsedTime);
        end

    end

    methods (Static, Access = private)

        function TryAddMiceFromWorkspace_()
            cellWorkspaceEnvNames = ["WS_SIMGEARS", "WS_NAVSYS"];

            for idxEnv = 1:numel(cellWorkspaceEnvNames)
                charWorkspaceRoot = string(getenv(cellWorkspaceEnvNames(idxEnv)));
                if strlength(strtrim(charWorkspaceRoot)) == 0
                    continue
                end

                cellMiceRootCandidates = CShapeModel.BuildMiceRootCandidates_(charWorkspaceRoot);
                for idxCandidate = 1:numel(cellMiceRootCandidates)
                    charMiceRoot = cellMiceRootCandidates(idxCandidate);
                    charMiceSrcPath = fullfile(charMiceRoot, "src", "mice");
                    charMiceLibPath = fullfile(charMiceRoot, "lib");
                    if ~isfile(fullfile(charMiceSrcPath, "cspice_furnsh.m"))
                        continue
                    end

                    if isfolder(charMiceLibPath)
                        addpath(char(charMiceLibPath));
                    end
                    addpath(char(charMiceSrcPath));

                    if ~isempty(which('cspice_furnsh'))
                        return
                    end
                end
            end
        end

        function cellMiceRootCandidates = BuildMiceRootCandidates_(charWorkspaceRoot)
            charWorkspaceRoot = string(charWorkspaceRoot);
            cellMiceRootCandidates = strings(1, 0);

            if strlength(strtrim(charWorkspaceRoot)) == 0
                return
            end

            cellMiceRootCandidates(end + 1) = fullfile(charWorkspaceRoot, "mice");
            cellMiceRootCandidates(end + 1) = charWorkspaceRoot;

            charParentRoot = string(fileparts(charWorkspaceRoot));
            if strlength(charParentRoot) > 0
                cellMiceRootCandidates(end + 1) = fullfile(charParentRoot, "mice");
            end

            cellMiceRootCandidates = unique(cellMiceRootCandidates, "stable");
        end

        function dGravConst = ResolveGravConstForLengthUnit_(charLengthUnits, dExplicitGravConst)
            if isfinite(dExplicitGravConst)
                dGravConst = dExplicitGravConst;
                return
            end

            if strcmpi(charLengthUnits, "km")
                dGravConst = 6.67430e-20;
            else
                dGravConst = 6.67430e-11;
            end
        end

    end

end
