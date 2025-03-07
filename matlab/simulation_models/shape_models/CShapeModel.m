classdef CShapeModel < CBaseDatastruct
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 05-10-2024        Pietro Califano         First implementation completed.
    % 13-02-2025        Pietro Califano         Update implementation to inherit from CBaseDatastruct (handle)
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------


    properties (SetAccess = protected, GetAccess = public)

        charTargetUnitOutput = 'm';

    end

    properties (SetAccess = protected, GetAccess = public)

        bHasData_ = false;
        ui32triangVertexPtr = []; % Assumed as [3, N] array
        dVerticesPos = [];        % Assumed as [3, N] array
        ui32NumOfVertices = uint32(0);
        unitScaler = 1;

    end

    methods (Access=public)
        % CONSTRUCTOR 
        function self = CShapeModel(charLoadingMethod, inputData, charInputUnit, charTargetUnitOutput)
            arguments
                charLoadingMethod    (1,:) string {mustBeA(charLoadingMethod, ["string", "char"]), mustBeMember(charLoadingMethod, ["mat", "cspice", "struct"])}
                inputData            (1,1) 
                charInputUnit        (1,:) string {mustBeA(charInputUnit       , ["string", "char"]), mustBeMember(charInputUnit, ["m", "km"])} = 'km'
                charTargetUnitOutput (1,:) string {mustBeA(charTargetUnitOutput, ["string", "char"]), mustBeMember(charTargetUnitOutput, ["m", "km"])} = 'm' % TODO add enumaration
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
            
            % Call input specific loading function
            if strcmpi(charLoadingMethod, 'cspice')

                if strcmpi(charInputUnit, 'm') 
                    warning("CSpice usually gives models' vertices in km, but 'meters' has been specified. Make sure this is correct.")
                end

                [self] = LoadModelFromSPICE(self, inputData);

            elseif strcmpi(charLoadingMethod, 'mat')
                [self] = LoadModelFromMat(self, inputData);

            elseif strcmpi(charLoadingMethod, 'struct')
                [self] = LoadModelFromStruct(self, inputData);
            end

            % Get number of vertices
            self.ui32NumOfVertices = size(self.dVerticesPos, 2);
        end
        
        %% GETTERS
        % Get shape model vertices and indices
        function [dataStruct] = getShapeStruct(self)
            dataStruct = struct();

            if self.bHasData_ == true
                dataStruct.ui32triangVertexPtr = self.ui32triangVertexPtr;
                dataStruct.dVerticesPos = self.dVerticesPos;
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
        % None for now
    end
      
    methods (Access=protected)
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

            CheckIfModelAlreadyLoaded(self);

            % Get info from kernel
            [~, ~, kernelhandle] = cspice_kinfo(char(charKernelName));
            dladsc = cspice_dlabfs(kernelhandle);

            % Get number of vertices and triangles
            [nVertices, nTriangles] = cspice_dskz02(kernelhandle, dladsc);
            % Get triangles from SPICE (p: plates)
            trianglesVertices = cspice_dskp02(kernelhandle, dladsc, 1, nTriangles);

            % Get vertices from SPICE (v: vertices)
            modelVertices = self.unitScaler * cspice_dskv02(kernelhandle, dladsc, 1, nVertices);

            % Assign data to object attributes
            self.ui32triangVertexPtr = trianglesVertices;
            self.dVerticesPos        = modelVertices;

            self.bHasData_ = true;
        end

        function [self] = LoadModelFromStruct(self, strShapeModel)
            % ACHTUNG: input struct is assumed to have fields with the same name as self.strShapeModel
            CheckIfModelAlreadyLoaded(self);

            cellFieldnames = fieldnames(strShapeModel);
            assert( not(isempty(cellFieldnames{contains(cellFieldnames, '32')} )), 'ERROR: automatic fieldnames resolution failed.');
            assert( not(isempty(cellFieldnames{contains(cellFieldnames, 'dVert')} )), 'ERROR: automatic fieldnames resolution failed.');

            self.ui32triangVertexPtr = uint32(strShapeModel.(cellFieldnames{contains(cellFieldnames, '32')} ));
            self.dVerticesPos        = double(strShapeModel.(cellFieldnames{contains(cellFieldnames, 'dVert')} ));

            self.bHasData_ = true;
        end

        function [self] = LoadModelFromMat(self, pathToMatfile)
            % ACHTUNG: input mat is assumed to contain a struct with fields with the same name as self.strShapeModel
            if exist(pathToMatfile, "file")

                CheckIfModelAlreadyLoaded(self);

                structdata = load(pathToMatfile); % DEVNOTE: check that structdata is the desired struct
                [self] = LoadModelFromStruct(self, structdata);

                self.bHasData_ = true;
            else
                error('Mat file not found. Check path.')
            end

        end

        function CheckIfModelAlreadyLoaded(self)
            if self.bHasData_
                warning('A shape model was already loaded before and is being overwritten.')
            end
        end
    end

end

