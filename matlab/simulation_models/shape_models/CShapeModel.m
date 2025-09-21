classdef CShapeModel < CBaseDatastruct
    %% DESCRIPTION
    % Unified object class representing triangular meshes in the standard format (vertices, triangles),
    % where vertices are a set of 3D points and triangles a set of indices indicating which vertices form
    % each triangle.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 05-10-2024    Pietro Califano     First implementation completed.
    % 13-02-2025    Pietro Califano     Update implementation to inherit from CBaseDatastruct (handle)
    % 03-05-2025    Pietro Califano     Add implementation to support loading from and writing to obj file
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = public, GetAccess = public)
        dTargetShapeMatrix_OF {ismatrix, isnumeric} = zeros(0,0,'double')
        dObjectReferenceSize {isscalar, isnumeric}  = zeros(0,0)
        charTargetUnitOutput = 'm';
    end

    properties (SetAccess = protected, GetAccess = public)

        bHasData_ = false;
        ui32triangVertexPtr = []; % Assumed as [3, N] array
        dVerticesPos = [];        % Assumed as [3, N] array
        ui32NumOfVertices = uint32(0);
        unitScaler = 1;

        % Optional data
        dTexCoords = [];
        ui32TrianglesTexIndex = [];
        dNormals = [];
        ui32TrianglesNormalsIndex = [];
        charModelName = ""

    end

    methods (Access = public)
        % CONSTRUCTOR 
        function self = CShapeModel(enumLoadingMethod, ...
                                    varInputData, ...
                                    charInputUnit, ...
                                    charTargetUnitOutput, ...
                                    bVertFacesOnly, ...
                                    charModelName, ...
                                    bLoadShapeModel)
            arguments
                enumLoadingMethod       (1,:) string {mustBeA(enumLoadingMethod, ["string", "char"]), ...
                                                mustBeMember(enumLoadingMethod, ["mat", "cspice", "struct", "file_obj"])} = "file_obj"
                varInputData            (1,:) = []
                charInputUnit           (1,:) string {mustBeA(charInputUnit       , ["string", "char"]), ...
                                                    mustBeMember(charInputUnit, ["m", "km"])} = 'km'
                charTargetUnitOutput    (1,:) string {mustBeA(charTargetUnitOutput, ["string", "char"]), mustBeMember(charTargetUnitOutput, ["m", "km"])} = 'm' % TODO add enumaration
                bVertFacesOnly          (1,1) {islogical} = true;
                charModelName           (1,:) char = ""
                bLoadShapeModel         (1,1) {islogical} = true;
            end

            % For default (placeholder) construction
            if nargin < 1
                return
            end

            self.charTargetUnitOutput = charTargetUnitOutput;

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

                end
            end
        
            % Write model name 
            self.charModelName = charModelName;

            % Get number of vertices
            self.ui32NumOfVertices = size(self.dVerticesPos, 2);
            
            if self.ui32NumOfVertices > 0
                % Update unit scaling
                self.dVerticesPos = self.unitScaler * self.dVerticesPos;
            end
        end

        %% GETTERS
        % Get shape model vertices and indices
        function [strData] = getShapeStruct(self)
            strData = struct();

            if self.bHasData_ == true
                strData.ui32triangVertexPtr = self.ui32triangVertexPtr;
                strData.dVerticesPos = self.dVerticesPos;
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
            % TODO

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
    
            % Transpose vertices and triangles
            self.ui32triangVertexPtr = self.ui32triangVertexPtr;
            self.dVerticesPos        = self.dVerticesPos;

            if not(bVertFacesOnly)
                self.dTexCoords = transpose(self.dTexCoords);
                self.ui32TrianglesTexIndex = transpose(self.ui32TrianglesTexIndex);
                self.dNormals = transpose(self.dNormals);
                self.ui32TrianglesNormalsIndex = transpose(self.ui32TrianglesNormalsIndex);
            end

            self.bHasData_ = true;

        end

        function checkIfModelAlreadyLoaded(self)
            if self.bHasData_
                warning('A shape model was already loaded before and is being overwritten.')
            end
        end
    end

    methods (Static, Access = public)

        function [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
                ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = LoadModelFromObj(charObjFilePath, bVertFacesOnly) %#codegen
            arguments
                charObjFilePath (1,1) string {mustBeA(charObjFilePath, ["string", "char"])}
                bVertFacesOnly  (1,1) {islogical} = true;
            end
            %% SIGNATURE
            % [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
            %  ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = LoadModelFromObj(charObjFilePath, bVertFacesOnly) %#codegen
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % [ui32TrianglesIndex, dVerticesCoords] = LoadModelFromObj(charObjFilePath) reads the vertices and the
            % triangles data as specified in the input Wavefront .obj file. 
            % This implementation uses vectorized regexp and sscanf on the entire file content, avoiding 
            % per-line loops and dynamic allocation. Output formats:
            %     ui32TrianglesIndex    - R-by-3 uint32 array of face indices (v/vt/vn)
            %     dVerticesCoords       - M-by-3 double array of vertex coordinates
            %     dTexCoords            - P-by-2 double array of texture coordinates (if present)
            %     dNormals              - Q-by-3 double array of normals (if present)
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 03-01-2025    Pietro Califano     Function implemented for general obj format loading
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Future upgrades
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Function code
            
            tic
            % Check extension
            [~,~, charFileExt] = fileparts(charObjFilePath);

            if ~strcmpi(charFileExt, '.obj')
                error('LoadModelFromObj:InvalidExtension', 'Input file must have .obj extension.');
            end

            if not(isfile(charObjFilePath))
                error('LoadModelFromObj:FileNotFound', 'Cannot find file: %s', charObjFilePath);
            end

            % Read entire file as text
            charFileText = fileread(charObjFilePath);
        
            % Vertex lines: 'v x y z'
            vPattern = '^v\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)';
            vLines = regexp(charFileText, vPattern, 'tokens', 'lineanchors');

            if ~isempty(vLines)
                vTokens = vertcat(vLines{:});
                dVerticesCoords = str2double(vTokens);
                dVerticesCoords = reshape(dVerticesCoords', 3, []);
            else
                dVerticesCoords = zeros(0,3);
            end

            % Texture-coordinate lines: 'vt u v'
            vtPattern = '^vt\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)';
            vtLines = regexp(charFileText, vtPattern, 'tokens', 'lineanchors');

            if ~isempty(vtLines) && not(bVertFacesOnly)
                vtTokens = vertcat(vtLines{:});
                dTexCoords = str2double(vtTokens);
                dTexCoords = reshape(dTexCoords, 2, [])';
            else
                dTexCoords = zeros(0,2);
            end

            % Normal lines: 'vn nx ny nz'
            vnPattern = '^vn\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)';
            vnLines = regexp(charFileText, vnPattern, 'tokens', 'lineanchors');

            
            if ~isempty(vnLines) && not(bVertFacesOnly)
                vnTokens = vertcat(vnLines{:});
                dNormals = str2double(vnTokens);
                dNormals = reshape(dNormals, 3, [])';
            else
                dNormals = zeros(0,3);
            end


            % Defaults
            ui32TrianglesIndex          = zeros(0,3,'uint32');
            ui32TrianglesTexIndex       = zeros(0,3,'uint32');
            ui32TrianglesNormalsIndex   = zeros(0,3,'uint32');

            % Determine face format by presence of vt/vn
            bHasVT = ~isempty(dTexCoords);
            bHasVN = ~isempty(dNormals);

            % Build regex and index maps
            if bHasVT && bHasVN
                charPattern = '^f\s+(\d+)/(\d+)/(\d+)\s+(\d+)/(\d+)/(\d+)\s+(\d+)/(\d+)/(\d+)';
                vidx = 1:3:9; tidx = 2:3:9; nidx = 3:3:9;

            elseif bHasVT
                charPattern = '^f\s+(\d+)/(\d+)\s+(\d+)/(\d+)\s+(\d+)/(\d+)';
                vidx = 1:2:6; tidx = 2:2:6; nidx = [];
            
            elseif bHasVN
                charPattern = '^f\s+(\d+)//(\d+)\s+(\d+)//(\d+)\s+(\d+)//(\d+)';
                vidx = 1:2:6; tidx = [];    nidx = 2:2:6;
            
            else
                charPattern = '^f\s+(\d+)\s+(\d+)\s+(\d+)';
                vidx = 1:3; tidx = [];    nidx = [];
            end

            % Parse face lines
            fTokens = regexp(charFileText, charPattern, 'tokens', 'lineanchors');
            assert(not(isempty(fTokens)), ['ERROR: no faces detected in obj file. ' ...
                'Something in regexpr interpreter may have failed. ' ...
                'Your file may contain quads, but this function only supports triangular meshes.'])

            if ~isempty(fTokens)

                dValues = str2double(vertcat(fTokens{:}));
                dFacesMatrix = reshape(dValues', numel(vidx) + numel(tidx) + numel(nidx), []);
                ui32TrianglesIndex = uint32(dFacesMatrix(vidx, :));

                if not(bVertFacesOnly)
                    if bHasVT, ui32TrianglesTexIndex = uint32(dFacesMatrix(tidx, :)); end
                    if bHasVN, ui32TrianglesNormalsIndex   = uint32(dFacesMatrix(nidx, :)); end
                end
            end

            dElapsedTime = toc;
            fprintf("\nFile obj loaded in %.5g seconds\n", dElapsedTime);
        end

    end

end

