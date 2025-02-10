classdef CTargetEmulator < CSceneObject
    % TODO verify this works as subclass of CSceneObject
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 13-08-2024        Pietro Califano         Class created from previous script code.
    % 05-10-2024        Pietro Califano         Implementation v1.0 completed with all relevant functionalities.
    % 10-10-2024        Pietro Califano         Minor changes of code for bug fixes; verification completed.
    % 09-11-2024        Pietro Califano         Improved design and robustness of points generation and
    %                                           retrieval methods.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % 0) IMPROVE landmarks generation function: current version is temporary.
    % 1) Add functionality to make target emulator build CShapeModel directly from its inputs, instead of
    % requiring the user to pass it as input.
    % 2) Add getter to retrive points in World frame as specified by the pose of the target
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = protected, GetAccess = public)

        % Storage attributes
        targetUnitOutput = 'm'; % TODO make this an enum class for safe usage
        % Entity position and attitude wrt World frame
        dPosVector_W  = [0; 0; 0];
        dRot3_WfromTB = eye(3,3); % Internally stored as DCM
    end

    properties (SetAccess = protected, GetAccess = public)
        bHasModel_ = false;
        objShapeModel;

        ui32NumOfPointsGT         = uint32(0);
        i32LandmarksID            = zeros(1, 0, "uint32")
        dPointsPositionsGT_TB     = zeros(3, 0, 'double');
        enumPointGTsamplingMethod     = -1;
    end


    methods (Access = public)
        % CONSTRUCTORS
        function self = CTargetEmulator(objShapeModel, ui32NumOfPointsGT)
            arguments
                objShapeModel     (1,1) CShapeModel
                ui32NumOfPointsGT (1,1) uint32 = 0
            end

            % Set ShapeModel object
            self.objShapeModel = objShapeModel;

            if ui32NumOfPointsGT == 0
                % Assume number of points equal to CShapeModel number of vertices
                self.ui32NumOfPointsGT        = objShapeModel.getNumOfVertices();
            else
                self.ui32NumOfPointsGT        = ui32NumOfPointsGT;
            end
            
            if self.objShapeModel.hasData() == true
                self.bHasModel_ = true;
            end

            % Allocate memory for landmarks if required
            self.dPointsPositionsGT_TB = zeros(3, ui32NumOfPointsGT, 'double');
            self.i32LandmarksID        = zeros(1, ui32NumOfPointsGT, 'int32');
            
        end
        
        % GETTERS
        function bool = hasLandmarks(self)
            if self.enumPointGTsamplingMethod ~= -1
                bool = true;
            else
                bool = false;
            end
        end

        function [dPosVector_W, dRot3_WfromTB] = GetPose(self, enumParamType)
            % Method to get pose of target emulator
            % [dPosVector_W, dRot3_WfromTB] = GetPose(self, enumParamType)
            arguments
                self
                enumParamType (1,1) = EnumRotParams.DCM
            end

            dPosVector_W  = self.dPosVector_W;
            dRot3_WfromTB = self.rotation(enumParamType);
        end

        function dPosVector_W = translation(self)
            dPosVector_W = self.dPosVector_W;
        end

        function dRot3_WfromTB = rotation(self, enumParamType)
            arguments
                self
                enumParamType (1,1) = EnumRotParams.DCM
            end

            switch enumParamType
                case EnumRotParams.DCM
                    dRot3_WfromTB = self.dRot3_WfromTB;
                    
                case EnumRotParams.QUAT_VSRPplus
                    dRot3_WfromTB = DCM2quat(self.dRot3_WfromTB, true);
                    % case EnumRotParams.QUAT_JPL
                    % case EnumRotParams.QUAT_HAMILTON
                    % case EnumRotParams.MRP
                otherwise
                    error('Not (yet) implemented parameterization.')
            end
        end

        % GETTERS
        % TODO function to get landmarks in nav frame using target pose instead of target fixed frame
        function [] = GetPointsInWorldFrame(self)
            arguments
                self
            end
            %TODO
            error('Not implemented yet')
        end

        function [dPointsPositionsGT_TB, ui32PointsIDs] = GetPointsInTargetFrame(self, i32PointsIndices, bUseIndicesAsPtrs)
            arguments
                self (1,1)
                i32PointsIndices (1,:) int32 = 0
                bUseIndicesAsPtrs (1,1) logical {isscalar} = false
            end
            
            assert(self.enumPointGTsamplingMethod ~= -1, 'No simulated GT points found. Did you forget to call "SampleMeshPoints" method first?') % Assert if points have been generated

            if nargin < 2 || all(i32PointsIndices == 0)  % Return all points
                dPointsPositionsGT_TB = self.dPointsPositionsGT_TB;
                ui32PointsIDs = self.i32LandmarksID;
                return
            end

            % Return points specified by indices
            % if max(ui32PointsIndices) > self.ui32NumOfPointsGT
            %     error('Max index %d is out of bound. You set a maximum number of points of %d.', max(ui32PointsIndices), self.ui32NumOfPointsGT)
            % end

            if bUseIndicesAsPtrs
                assert(  max(i32PointsIndices) <= self.ui32NumOfPointsGT, sprintf('Max index %d is out of bound. You set a maximum number of points of %d.', max(i32PointsIndices), self.ui32NumOfPointsGT) )
                dPointsPositionsGT_TB = self.dPointsPositionsGT_TB(:, i32PointsIndices);
                ui32PointsIDs = self.i32LandmarksID(:, i32PointsIndices);

            else
                % Find all IDs in ui32LandMarkIDs
                [bMaskLocA, ui32RelColLocB] = ismember(i32PointsIndices, self.i32LandmarksID);
                assert(sum(bMaskLocA) == length(i32PointsIndices), 'Sanity check assert. One or more queried indices do not correspond to IDs of points in target emulator instance.')
                
                dPointsPositionsGT_TB = self.dPointsPositionsGT_TB(:, ui32RelColLocB(ui32RelColLocB > 0));
                ui32PointsIDs = self.i32LandmarksID(:, ui32RelColLocB(ui32RelColLocB > 0));
            end

        end
            
        
        function [strTargetDataStruct] = getTargetStruct(self)
            
            % TEMPORARY DEFINITION BEFORE UPDATE of CheckLMvisibility function
            strTargetDataStruct.strShapeModel = self.getShapeStruct();
            strTargetDataStruct.dTargetPos_IN = self.dPosVector_W; 
            strTargetDataStruct.dQuat_INfromTB = self.rotation(EnumRotParams.QUAT_VSRPplus);
            strTargetDataStruct.bIS_JPL_QUAT = true; % Incorrect fieldname, because QUAT_VSRPplus not yet defined at time of implementation
            
            % Order fields by name
            strTargetDataStruct = orderfields(strTargetDataStruct);
        end


        function [strShapeDataStruct] = getShapeStruct(self)
            strShapeDataStruct = self.objShapeModel.getShapeStruct();
            strShapeDataStruct = orderfields(strShapeDataStruct);
        end

        % SETTERS
        function [self] = SetPose(self, dPosVector_W, dRot3_WfromTB)
            arguments
                self
                dPosVector_W  (3,1) double {isvector}
                dRot3_WfromTB (3,3) double {ismatrix}
            end
            self.dPosVector_W  = dPosVector_W  ;
            self.dRot3_WfromTB = dRot3_WfromTB ;
        end
        
        % PUBLIC METHODS
        function [self, dPointsPositionsGT_TB, i32LandmarksID] = GenerateSimulatedPoints_TB(self, enumPointGTsamplingMethod)
            % DEVNOTE: deprecated GenerateSimulatedPoints_TB method currently kept for legacy reasons
            [self, dPointsPositionsGT_TB, i32LandmarksID] = SampleMeshPoints(self, enumPointGTsamplingMethod);
        end

        function [self, dPointsPositionsGT_TB, i32LandmarksID] = SampleMeshPoints(self, enumPointGTsamplingMethod)
            arguments
                self
                enumPointGTsamplingMethod (1,1) EnumPointGTsamplingMethod = EnumPointGTsamplingMethod.PICK_UNIFORM_RANDOM_ID
            end

            bREUSE_FLAG = false;

            if self.enumPointGTsamplingMethod == -1
                % First call, assign method
                self.enumPointGTsamplingMethod = enumPointGTsamplingMethod;
            else
                % Determine if the same method is used
                if enumPointGTsamplingMethod == self.enumPointGTsamplingMethod
                    bREUSE_FLAG = true;
                end
            end

            assert(self.bHasModel_, 'No model loaded. You need to load one before generating simulated landmarks.');

            if self.ui32NumOfPointsGT > 0 && bREUSE_FLAG == false
                
                % Generate landmarks in target fixed frame
                [dAllPointsPositionsGT_TB] = generateLandmarksMap(self.objShapeModel.getShapeStruct(), self.ui32NumOfPointsGT);

                self.dPointsPositionsGT_TB = dAllPointsPositionsGT_TB(2:4, :);
                self.i32LandmarksID = int32(dAllPointsPositionsGT_TB(1, :));
            
                dPointsPositionsGT_TB = self.dPointsPositionsGT_TB;
                i32LandmarksID = self.i32LandmarksID;

            elseif self.ui32NumOfPointsGT > 0 && bREUSE_FLAG == true
                % Return previously computed landmarks
                dPointsPositionsGT_TB   = self.dPointsPositionsGT_TB;
                i32LandmarksID          = self.i32LandmarksID;
            end

        end

    end

    methods (Access = protected)
        function [] = SampleMeshPoints_impl(self, enumPointGTsamplingMethod)
            arguments
                self
                enumPointGTsamplingMethod (1,1) EnumPointGTsamplingMethod = EnumPointGTsamplingMethod.PICK_UNIFORM_RANDOM_ID
            end


            switch enumPointGTsamplingMethod

                case EnumPointGTsamplingMethod.PICK_UNIFORM_RANDOM_ID
                    error('Not implemented yet')

                case EnumPointGTsamplingMethod.ICOSAHEDRON_SAMPLING
                    error('Not implemented yet')
            end
        end


    end

end


