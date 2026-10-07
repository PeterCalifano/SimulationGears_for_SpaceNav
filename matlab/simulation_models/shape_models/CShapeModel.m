classdef CShapeModel < CBaseDatastruct
    %% DESCRIPTION
    % Store triangular meshes as 3-by-V vertex coordinates and 3-by-F vertex indices.
    % Use file_obj for the established OBJ/attribute reader and file_mesh for repaired OBJ/STL
    % geometry. Select exact OBJ objects with charObjObjectNames before unit conversion and
    % simplification; an empty array keeps all faces, and "" selects unnamed faces.
    % Cache prepared target-frame triangle tracing separately from scene poses;
    % invalidate it after geometry mutations and reject changed unit metadata.
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
    % 21-09-2026    Pietro Califano, Codex gpt-5.6  Parse large OBJ face payloads in bounded blocks.
    % 29-09-2026    Pietro Califano, Codex gpt-6    Select OBJ objects before shape and gravity preparation.
    % 05-10-2026    Pietro Califano, Codex (GPT-6)  Cache reusable fixed-size flat/BVH ray geometry.
    % 07-10-2026    Pietro Califano, Codex (GPT-6)  Reuse prepared caches and exclude them from exports.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CBaseDatastruct, EnumLengthUnits, LoadShapeMesh, SelectObjFaceRecords,
    % CompactShapeMeshVertices, FitSpherHarmCoeffToPolyhedrGrav,
    % BuildTriangleRayData, ValidateTriangleRayData
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

    properties (Access = private, Transient)
        % Rebuild runtime tracing data after loading; never serialize a second mesh.
        strRayCache_ struct = struct()
    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = CShapeModel(enumLoadingMethod, varInputData, charInputUnit, ...
                charTargetUnitOutput, bVertFacesOnly, charModelName, bLoadShapeModel, options)
            %% SIGNATURE
            % self = CShapeModel(enumLoadingMethod, varInputData, charInputUnit, ...
            %     charTargetUnitOutput, bVertFacesOnly, charModelName, bLoadShapeModel, options)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Load geometry, convert length units and optionally simplify the selected mesh.
            % Preserve the source frame and origin. Calling without inputs creates a placeholder.
            % Select OBJ objects before computing geometry-dependent quantities; auxiliary OBJ
            % attributes retain their independent corner indices until optional simplification.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % enumLoadingMethod             mat, cspice, struct, file_obj, or file_mesh.
            % varInputData                  Source path or data accepted by the selected loader.
            % charInputUnit                 Source coordinate unit, m or km.
            % charTargetUnitOutput          Stored coordinate unit, m or km.
            % bVertFacesOnly                Load geometry only; required by file_mesh.
            % charModelName                 Model label.
            % bLoadShapeModel               Load source data when true; otherwise retain an empty model.
            % options.dMeshSimplifyFactor    Mesh keep fraction, clamped to [0,1].
            % options.charObjObjectNames     Exact OBJ names; empty keeps all, "" selects unnamed faces.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self                          Shape model in the requested output units.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Document selection and loading order.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EnumLengthUnits, LoadShapeMesh, CShapeModel.LoadModelFromObj
            % -------------------------------------------------------------------------------------------------------------
            arguments (Input)
                enumLoadingMethod       (1,:) string {mustBeA(enumLoadingMethod, ["string", "char"]), ...
                    mustBeMember(enumLoadingMethod, ["mat", "cspice", "struct", "file_obj", "file_mesh"])} = "file_obj"
                varInputData            (1,:) = []
                charInputUnit           {mustBeA(charInputUnit, ["string", "char", "EnumLengthUnits"])} = 'km'
                charTargetUnitOutput    {mustBeA(charTargetUnitOutput, ["string", "char", "EnumLengthUnits"])} = 'm'
                bVertFacesOnly          (1,1) logical = true;
                charModelName           (1,:) char = ""
                bLoadShapeModel         (1,1) logical = true;
            end
            arguments (Input)
                options.dMeshSimplifyFactor (1,1) double {mustBeFinite} = 1.0
                options.charObjObjectNames (1,:) string {mustBeNonmissing} = strings(1, 0)
            end
            arguments (Output)
                self (1,1) CShapeModel
            end

            % Preserve placeholder construction without reading a source or resolving its units.
            if nargin < 1
                return
            end

            % Reject object selection for loaders that have no OBJ object records.
            if ~isempty(options.charObjObjectNames) && ...
                    ~any(enumLoadingMethod == ["file_obj", "file_mesh"])
                error('CShapeModel:ObjectSelectionRequiresObj', ...
                    'OBJ object selection requires file_obj or an OBJ file_mesh source.');
            end

            charInputUnit = char(EnumLengthUnits.toString(charInputUnit));
            self.charTargetUnitOutput = char(EnumLengthUnits.toString(charTargetUnitOutput));
            self.bDefaultConstructed  = false;
            self.dMeshSimplifyFactor  = min(max(double(options.dMeshSimplifyFactor), 0.0), 1.0);

            % Convert stored geometry exactly once after the source loader completes.
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
                    self = self.LoadModelFromObj_(varInputData, bVertFacesOnly, ...
                                                 options.charObjObjectNames);

                elseif strcmpi(enumLoadingMethod, 'file_mesh')
                    self = self.LoadModelFromMeshFile_(varInputData, bVertFacesOnly, ...
                                                      options.charObjObjectNames);

                end
            end

            % Retain the caller's label and derive counts from the selected geometry.
            self.charModelName = charModelName;

            self.ui32NumOfVertices = size(self.dVerticesPos, 2);

            % Apply scaling and simplification before updating geometry-derived quantities.
            if self.ui32NumOfVertices > 0
                self.dVerticesPos = self.unitScaler * self.dVerticesPos;

                if self.dMeshSimplifyFactor < 1.0
                    [self, ~] = self.SimplifyMesh(100.0 * (1.0 - self.dMeshSimplifyFactor));
                end
            end

            self = self.UpdateDerivedGeometry_();
        end

        %% GETTERS
        function self = prepareRayTracingData(self, bUseBvh)
            %% SIGNATURE
            % self = self.prepareRayTracingData(bUseBvh)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Prepare a reusable exact ray model in the stored geometry length unit.
            % Cache triangle edges and fixed numeric traversal data. Geometry changes
            % clear the cache; rigid scene poses reuse it. Repeated preparation with
            % the same units and traversal selection reuses the existing payload.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self        Loaded shape model.
            % bUseBvh     Select exact BVH traversal; default false.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self        Shape model with prepared numeric ray data.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % BuildTriangleRayData, ValidateTriangleRayData.
            % -------------------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1, 1) CShapeModel
                bUseBvh (1, 1) logical = false
            end

            arguments (Output)
                self (1, 1) CShapeModel
            end

            % Keep preparation out of repeated acquisitions and startup re-entry.
            if isfield(self.strRayCache_, 'strModelData') && ...
                    strcmp(self.strRayCache_.charUnits, self.charTargetUnitOutput) && ...
                    self.strRayCache_.strModelData.strRayData.bUseBvh == bUseBvh
                return
            end

            % Validate source indices before preparing fixed triangle edges and bounds.
            assert(self.bHasData_, 'CShapeModel:NoData', 'Load geometry before ray preparation.');
            assert(all(self.ui32triangVertexPtr >= 1, 'all') && ...
                all(self.ui32triangVertexPtr <= size(self.dVerticesPos, 2), 'all'), ...
                'CShapeModel:RayIndices', 'Triangle indices must reference stored vertices.');
                
            dFaceVertices = reshape(self.dVerticesPos(:, self.ui32triangVertexPtr(:)), 3, 3, []);
            strRayData = BuildTriangleRayData(dFaceVertices, bUseBvh);
            ValidateTriangleRayData(strRayData);

            % Retain unit metadata outside the fixed numeric payload.
            self.strRayCache_ = struct('charUnits', self.charTargetUnitOutput, ...
                                      'strModelData', struct('strRayData', strRayData));
        end

        function strModelData = getRayTracingModel(self)
            %% SIGNATURE
            % strModelData = self.getRayTracingModel()
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return prepared numeric ray geometry or the compatible legacy mesh schema.
            % Reject stale unit metadata instead of silently reinterpreting a prepared
            % cache. Keep source/model metadata outside the numerical ray payload.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self          Shape model.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strModelData  Prepared strRayData, or raw vertices and signed triangle pointers.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1, 1) CShapeModel
            end

            arguments (Output)
                strModelData (1, 1) struct
            end

            if isfield(self.strRayCache_, 'strModelData')
                assert(strcmp(self.strRayCache_.charUnits, self.charTargetUnitOutput), ...
                    'CShapeModel:StaleRayUnits', 'Reprepare ray geometry after changing length units.');
                strModelData = self.strRayCache_.strModelData;
            else
                % Retain legacy callers while keeping their conversion out of the prepared path.
                assert(all(self.ui32triangVertexPtr <= uint32(intmax('int32')), 'all'), ...
                    'CShapeModel:MeshIndexOverflow', 'Triangle indices exceed int32 capacity.');
                strModelData = struct('dVerticesPositions', self.dVerticesPos, ...
                                      'i32triangVertexPtrs', int32(self.ui32triangVertexPtr));
            end
        end

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

        function self = LoadModelFromObj_(self, charObjFilePath, bVertFacesOnly, charObjObjectNames)
            % Populate geometry and independently indexed attributes before constructor unit scaling.
            arguments (Input)
                self
                charObjFilePath (1,:) {mustBeA(charObjFilePath, ["string", "char"])}
                bVertFacesOnly (1,1) logical
                charObjObjectNames (1,:) string {mustBeNonmissing} = strings(1, 0)
            end
            arguments (Output)
                self
            end
            checkIfModelAlreadyLoaded(self);

            [self.ui32triangVertexPtr, self.dVerticesPos, ...
                self.dTexCoords, self.ui32TrianglesTexIndex, ...
                self.dNormals, self.ui32TrianglesNormalsIndex] = ...
                CShapeModel.LoadModelFromObj(charObjFilePath, bVertFacesOnly, ...
                                            charObjObjectNames=charObjObjectNames);

            % Retain column-major geometry and transpose only the auxiliary parser outputs.
            if not(bVertFacesOnly)
                self.dTexCoords = transpose(self.dTexCoords);
                self.ui32TrianglesTexIndex = transpose(self.ui32TrianglesTexIndex);
                self.dNormals = transpose(self.dNormals);
                self.ui32TrianglesNormalsIndex = transpose(self.ui32TrianglesNormalsIndex);
            end

            self = self.UpdateDerivedGeometry_();
            self.bHasData_ = true;

        end

        function self = LoadModelFromMeshFile_(self, charMeshFilePath, bVertFacesOnly, charObjObjectNames)
            %% SIGNATURE
            % self = LoadModelFromMeshFile_(self, charMeshFilePath, bVertFacesOnly, charObjObjectNames)
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
            % charObjObjectNames: Exact OBJ names selected before repair; empty means all objects.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self:             Populated shape-model instance.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 28-08-2026  Pietro Califano     Add shared repaired OBJ/STL geometry loading.
            % 29-09-2026  Pietro Califano, Codex gpt-6    Forward selection before repair and conversion.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh.
            % -------------------------------------------------------------------------------------------------------------

            arguments (Input)
                self
                charMeshFilePath (1,:) {mustBeA(charMeshFilePath, ["string", "char"])}
                bVertFacesOnly (1,1) logical
                charObjObjectNames (1,:) string {mustBeNonmissing} = strings(1, 0)
            end

            arguments (Output)
                self
            end

            % Reject auxiliary payload before invoking the geometry-only shared reader.
            if ~bVertFacesOnly
                error('CShapeModel:MeshAuxiliaryDataUnsupported', ...
                    'file_mesh loading supports geometry only; set bVertFacesOnly to true.');
            end

            % Load repaired row-major geometry and adapt it to the established object layout.
            checkIfModelAlreadyLoaded(self);
            strShapeMesh = LoadShapeMesh(char(charMeshFilePath), bRepairMesh=true, ...
                charObjObjectNames=charObjObjectNames);
            self.ui32triangVertexPtr = transpose(strShapeMesh.ui32FaceVertexIds);
            self.dVerticesPos = transpose(strShapeMesh.dVerticesPos);

            % Refresh every property derived from geometry before publishing the loaded state.
            self = self.UpdateDerivedGeometry_();
            self.bHasData_ = true;
        end

        function [self] = UpdateDerivedGeometry_(self)
            % Invalidate every ray cache after loading, scaling or simplifying geometry.
            self.strRayCache_ = struct();

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
            %% SIGNATURE
            % [objShapeModel, strSHgravityData] = BuildSphericalHarmonicsGravityDataFromObj( ...
            %     charObjFilePath, ui32MaxDegree, options)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Load the selected OBJ geometry and fit exterior spherical-harmonics gravity from
            % its polyhedron field. Select objects before unit conversion, simplification and
            % fitting so excluded vertices cannot affect the fit radius. Optionally cache the
            % fit on the returned shape model. Use RunFitSpherHarmonicsToPolyhedronGravityFromObj
            % for holdout diagnostics and figures.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % charObjFilePath               Path to a supported OBJ source.
            % ui32MaxDegree                 Maximum spherical-harmonics degree.
            % options.charInputUnit         Source coordinate unit, m or km.
            % options.charTargetUnitOutput  Stored coordinate unit, m or km.
            % options.bVertFacesOnly        Load geometry only when true.
            % options.charModelName         Optional label; empty uses the file stem.
            % options.dGravParam            GM in output length units cubed per second squared.
            % options.dDensity              Density in kg per output length unit cubed.
            % options.dGravConst            G in output length units cubed per kg per second squared.
            % options.dBodyRadiusRef        Reference radius in output length units.
            %                               NaN uses the existing fitter's inference contract.
            % options.ui32MaxFitIterations  Maximum adaptive fit iterations.
            % options.dMeshSimplifyFactor   Mesh keep fraction, clamped by the constructor to [0,1].
            % options.bCacheOnShapeModel    Cache the fitted gravity data when true.
            % options.charObjObjectNames    Exact OBJ names; empty keeps all, "" selects unnamed faces.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objShapeModel                 Loaded shape in the requested output units.
            % strSHgravityData              Fitted coefficients, physical inputs and fit statistics.
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Document selected-geometry gravity preparation.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel, CShapeModel.BuildSphericalHarmonicsGravityData
            % -------------------------------------------------------------------------------------------------------------
            arguments (Input)
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
                options.charObjObjectNames     (1,:) string {mustBeNonmissing} = strings(1, 0)
            end
            arguments (Output)
                objShapeModel (1,1) CShapeModel
                strSHgravityData (1,1) struct
            end

            % Reject missing geometry before preparing a model label or gravity inputs.
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

            % Let the constructor select, scale and simplify geometry in its established order.
            objShapeModel = CShapeModel("file_obj", charObjFilePath, options.charInputUnit, ...
                options.charTargetUnitOutput, options.bVertFacesOnly, char(charModelName), true, ...
                dMeshSimplifyFactor=options.dMeshSimplifyFactor, ...
                charObjObjectNames=options.charObjObjectNames);

            % Reuse the existing fitter with the caller's cache policy.
            if options.bCacheOnShapeModel
                objShapeModel = objShapeModel.BuildAndSetSphericalHarmonicsGravityData(ui32MaxDegree, ...
                    dGravParam=options.dGravParam, dDensity=options.dDensity, ...
                    dGravConst=options.dGravConst, dBodyRadiusRef=options.dBodyRadiusRef, ...
                    ui32MaxFitIterations=options.ui32MaxFitIterations);
                strSHgravityData = objShapeModel.getSphericalHarmonicsGravityData();
            else
                strSHgravityData = CShapeModel.BuildSphericalHarmonicsGravityData(objShapeModel, ...
                    ui32MaxDegree, dGravParam=options.dGravParam, dDensity=options.dDensity, ...
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
            % Exact object selection precedes face decoding and repair. It compacts only vertex
            % indices; independently indexed vt/vn arrays and selected corner indices are preserved.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % charObjFilePath:       Path to a Wavefront OBJ file.
            % bVertFacesOnly:        Load geometry only when true.
            % options.bRepairMesh:   Weld duplicates and remove degenerate geometry when true.
            % options.charObjObjectNames: Exact names; empty array keeps all, "" selects unnamed.
            %                        Missing/empty selected objects raise SelectObjFaceRecords:MissingObjects.
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
            % 29-09-2026  Pietro Califano, Codex gpt-6    Select objects and preserve auxiliary indices.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh, SelectObjFaceRecords, CompactShapeMeshVertices
            % -------------------------------------------------------------------------------------------------------------

            arguments(Input)
                charObjFilePath (1,1) string {mustBeA(charObjFilePath, ["string", "char"])}
                bVertFacesOnly (1,1) logical = true
                options.bRepairMesh (1,1) logical = false
                options.charObjObjectNames (1,:) string {mustBeNonmissing} = strings(1, 0)
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
                strShapeMesh = LoadShapeMesh(char(charObjFilePath), bRepairMesh=true, ...
                    charObjObjectNames=options.charObjObjectNames);
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

            % Parse bounded face blocks so large OBJ files do not exceed MATLAB's
            % contiguous character or sscanf payload limits.
            [dFaceLineStartIdx, dFaceLineEndIdx] = regexp(charFileText, ...
                '^f[ \t]+[^\r\n]*$', 'start', 'end', 'lineanchors');
            [dFaceLineStartIdx, dFaceLineEndIdx] = SelectObjFaceRecords(charFileText, ...
                dFaceLineStartIdx, dFaceLineEndIdx, options.charObjObjectNames);
            [ui32TrianglesIndex, ui32TrianglesTexIndex, ui32TrianglesNormalsIndex] = ...
                CShapeModel.ParseObjFaceLines_(charFileText, dFaceLineStartIdx, ...
                dFaceLineEndIdx, bVertFacesOnly);

            % Compact geometry alone; selected texture and normal indices address separate arrays.
            if ~isempty(options.charObjObjectNames)
                [dSelectedVertices, ui32SelectedFaces] = ...
                    CompactShapeMeshVertices(dVerticesCoords.', ui32TrianglesIndex.');
                dVerticesCoords = dSelectedVertices.';
                ui32TrianglesIndex = ui32SelectedFaces.';
            end

            dElapsedTime = toc;
            fprintf("\nFile obj loaded in %.5g seconds\n", dElapsedTime);
        end

    end

    methods (Static, Access = private)

        function [ui32TrianglesIndex, ui32TrianglesTexIndex, ui32TrianglesNormalsIndex] = ...
                ParseObjFaceLines_(charFileText, dFaceLineStartIdx, dFaceLineEndIdx, bVertFacesOnly)
            %% SIGNATURE
            % [ui32TrianglesIndex, ui32TrianglesTexIndex, ui32TrianglesNormalsIndex] = ...
            %     CShapeModel.ParseObjFaceLines_(charFileText, dFaceLineStartIdx, ...
            %     dFaceLineEndIdx, bVertFacesOnly)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Parse triangular OBJ face records in bounded blocks while preserving source order and optional
            % texture and normal indices.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % charFileText         Complete OBJ text payload.
            % dFaceLineStartIdx    Start index of each face record in charFileText.
            % dFaceLineEndIdx      End index of each face record in charFileText.
            % bVertFacesOnly       True when auxiliary texture and normal indices are not required.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % ui32TrianglesIndex           Vertex indices as a 3-by-F array.
            % ui32TrianglesTexIndex        Texture-coordinate indices as a 3-by-F array when requested.
            % ui32TrianglesNormalsIndex    Normal indices as a 3-by-F array when requested.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 21-09-2026  Pietro Califano, Codex gpt-5.6  First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------
            arguments (Input)
                charFileText (1,:) char
                dFaceLineStartIdx (1,:) double
                dFaceLineEndIdx (1,:) double
                bVertFacesOnly (1,1) logical
            end
            arguments (Output)
                ui32TrianglesIndex uint32
                ui32TrianglesTexIndex uint32
                ui32TrianglesNormalsIndex uint32
            end

            ui32NumFaces = uint32(numel(dFaceLineStartIdx));
            assert(numel(dFaceLineEndIdx) == double(ui32NumFaces), ...
                'CShapeModel:InvalidObjFaceLineBounds', ...
                'OBJ face line start and end index arrays must have equal lengths.');
            ui32TrianglesIndex = zeros(0, 3, 'uint32');
            ui32TrianglesTexIndex = zeros(0, 3, 'uint32');
            ui32TrianglesNormalsIndex = zeros(0, 3, 'uint32');
            if ui32NumFaces == uint32(0)
                return
            end
            ui32TrianglesIndex = zeros(3, double(ui32NumFaces), 'uint32');

            % Resolve the uniform face syntax once. Mixed face syntaxes remain
            % outside the legacy vectorized loader contract.
            charFirstFace = string(strtrim(charFileText( ...
                dFaceLineStartIdx(1):dFaceLineEndIdx(1))));
            bHasTextureIndices = false;
            bHasNormalIndices = false;
            if ~isempty(regexp(charFirstFace, '^f\s+\d+//\d+', 'once'))
                charFaceFormat = 'f %u//%u %u//%u %u//%u\n';
                ui32ValuesPerFace = uint32(6);
                ui32VertexValueRows = uint32([1, 3, 5]);
                ui32NormalValueRows = uint32([2, 4, 6]);
                ui32TextureValueRows = zeros(1, 0, 'uint32');
                bHasNormalIndices = true;
            elseif ~isempty(regexp(charFirstFace, '^f\s+\d+/\d+/\d+', 'once'))
                charFaceFormat = 'f %u/%u/%u %u/%u/%u %u/%u/%u\n';
                ui32ValuesPerFace = uint32(9);
                ui32VertexValueRows = uint32([1, 4, 7]);
                ui32TextureValueRows = uint32([2, 5, 8]);
                ui32NormalValueRows = uint32([3, 6, 9]);
                bHasTextureIndices = true;
                bHasNormalIndices = true;
            elseif ~isempty(regexp(charFirstFace, '^f\s+\d+/\d+', 'once'))
                charFaceFormat = 'f %u/%u %u/%u %u/%u\n';
                ui32ValuesPerFace = uint32(6);
                ui32VertexValueRows = uint32([1, 3, 5]);
                ui32TextureValueRows = uint32([2, 4, 6]);
                ui32NormalValueRows = zeros(1, 0, 'uint32');
                bHasTextureIndices = true;
            else
                charFaceFormat = 'f %u %u %u\n';
                ui32ValuesPerFace = uint32(3);
                ui32VertexValueRows = uint32([1, 2, 3]);
                ui32TextureValueRows = zeros(1, 0, 'uint32');
                ui32NormalValueRows = zeros(1, 0, 'uint32');
            end

            if ~bVertFacesOnly && bHasTextureIndices
                ui32TrianglesTexIndex = zeros(3, double(ui32NumFaces), 'uint32');
            end
            if ~bVertFacesOnly && bHasNormalIndices
                ui32TrianglesNormalsIndex = zeros(3, double(ui32NumFaces), 'uint32');
            end

            % Keep each temporary text and numeric payload comfortably below
            % MATLAB's contiguous-array limits.
            ui32FaceParseChunkSize = uint32(250000);
            ui32BlockStart = uint32(1);
            while ui32BlockStart <= ui32NumFaces
                ui32BlockEnd = min(ui32BlockStart + ui32FaceParseChunkSize - uint32(1), ui32NumFaces);

                % Do not include object, group, or material records that separate
                % otherwise contiguous face runs in a parsed text block.
                if ui32BlockEnd > ui32BlockStart
                    dFaceLineGaps = dFaceLineStartIdx(double(ui32BlockStart + uint32(1)):double(ui32BlockEnd)) - ...
                        dFaceLineEndIdx(double(ui32BlockStart):double(ui32BlockEnd - uint32(1)));
                    dFirstRunBreak = find(dFaceLineGaps > 3.0, 1, 'first');
                    if ~isempty(dFirstRunBreak)
                        ui32BlockEnd = ui32BlockStart + uint32(dFirstRunBreak) - uint32(1);
                    end
                end

                ui32DestinationColumns = ui32BlockStart:ui32BlockEnd;
                charFaceBlock = charFileText( ...
                    dFaceLineStartIdx(double(ui32BlockStart)):dFaceLineEndIdx(double(ui32BlockEnd)));
                dParsedFaceValues = sscanf(charFaceBlock, charFaceFormat, ...
                    [double(ui32ValuesPerFace), Inf]);

                ui32ExpectedBlockFaces = ui32BlockEnd - ui32BlockStart + uint32(1);
                if size(dParsedFaceValues, 2) ~= double(ui32ExpectedBlockFaces)
                    error('CShapeModel:MalformedObjFaceBlock', ...
                        ['OBJ face block near face %u contains %u records but ', ...
                         'the selected syntax parsed %u.'], ...
                        ui32BlockStart, ui32ExpectedBlockFaces, uint32(size(dParsedFaceValues, 2)));
                end

                ui32ParsedFaceValues = uint32(dParsedFaceValues);
                ui32TrianglesIndex(:, double(ui32DestinationColumns)) = ...
                    ui32ParsedFaceValues(double(ui32VertexValueRows), :);
                if ~bVertFacesOnly && bHasTextureIndices
                    ui32TrianglesTexIndex(:, double(ui32DestinationColumns)) = ...
                        ui32ParsedFaceValues(double(ui32TextureValueRows), :);
                end
                if ~bVertFacesOnly && bHasNormalIndices
                    ui32TrianglesNormalsIndex(:, double(ui32DestinationColumns)) = ...
                        ui32ParsedFaceValues(double(ui32NormalValueRows), :);
                end
                ui32BlockStart = ui32BlockEnd + uint32(1);
            end
        end

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
