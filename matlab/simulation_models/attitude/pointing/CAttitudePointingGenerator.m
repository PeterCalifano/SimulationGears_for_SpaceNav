classdef CAttitudePointingGenerator < handle
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 08-12-2024    Pietro Califano     Class implemented from adapted codes for pointing generation
    % 12-02-2025    Pietro Califano     Update of class to add Sun position as input and static methods
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % Method1: Description
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % Property1: Description, dtype, nominal size
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = protected, GetAccess = public)

        dCameraPosition_Frame = -ones(3,1);
        dTargetPosition_Frame = -ones(3,1);
        dSunPosition_Frame    = -ones(3,1);

        bInvertZaxisForBlender    = false;
        bShowAttitudePointingPlot = false;
        bVerbose                  = false;
    end


    methods (Access = public)
        %% CONSTRUCTOR
        function self = CAttitudePointingGenerator(dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame, options)
            arguments
                dCameraPosition_Frame {ismatrix, mustBeNumeric} = -ones(3,1); % Defaults to placeholder value
                dTargetPosition_Frame {ismatrix, mustBeNumeric} = -ones(3,1); 
                dSunPosition_Frame {ismatrix, mustBeNumeric}    = -ones(3,1); 
            end

            arguments
                options.bInvertZaxisForBlender (1,1) logical {islogical, isscalar} = false;
            end

            % Assign properties
            self.dCameraPosition_Frame  = dCameraPosition_Frame;
            self.dTargetPosition_Frame  = dTargetPosition_Frame;
            self.dSunPosition_Frame     = dSunPosition_Frame;
            self.bInvertZaxisForBlender = options.bInvertZaxisForBlender;

        end

        % GETTERS

        % SETTERS

        % METHODS
        % PARTIAL TODO
        function [self, dOutRot3, dDCM_FramefromCAM] = pointToTarget_PositionOnly(self, dCameraPosition_Frame, dTargetPosition_Frame, options)
            arguments
                self
                dCameraPosition_Frame (3, :) double {ismatrix, mustBeNumeric} = self.dCameraPosition_Frame
                dTargetPosition_Frame (3, :) double {ismatrix, mustBeNumeric} = self.dTargetPosition_Frame
            end
            arguments
                options.enumOutRot3Param (1,1) EnumRotParams {isa(options.enumOutRot3Param, ...
                    'EnumRotParams')} = EnumRotParams.DCM
                options.dAuxiliaryAxis (3,1) double {isvector} = [0; 0; 1];
            end

            ui32NumOfEntries = uint32(size(dCameraPosition_Frame, 2));
            assert(size(dTargetPosition_Frame, 2) == 1 || size(dTargetPosition_Frame, 2) == ui32NumOfEntries, ...
                'Target position must have size equal to either a [3x1] vector (implicit expansion) or dCameraPosition_Frame array.')

            % Camera pointing direction (Z axis)
            dCamBoresightZ_Frame = dTargetPosition_Frame - dCameraPosition_Frame;

            if self.bInvertZaxisForBlender
                dCamBoresightZ_Frame = -dCamBoresightZ_Frame;
            end
            % Normalize direction
            dCamBoresightZ_Frame = dCamBoresightZ_Frame./vecnorm(dCamBoresightZ_Frame, 2, 1);

            % Camera X axis (Right)
            dAuxiliaryAxis = options.dAuxiliaryAxis;

            if ui32NumOfEntries > 1
                dAuxiliaryAxis = repmat(dAuxiliaryAxis, 1, ui32NumOfEntries);
            end

            dCamDirX_Frame = cross(dAuxiliaryAxis, dCamBoresightZ_Frame, 1);
            dCamDirX_Frame = dCamDirX_Frame./vecnorm(dCamDirX_Frame, 2, 1);

            % Camera Y axis (Left)
            dCamDirY_Frame = cross(dCamBoresightZ_Frame, dCamDirX_Frame);
            dCamDirY_Frame = dCamDirY_Frame./vecnorm(dCamDirY_Frame, 2, 1);

            % Compute attitude rotation matrix
            dDCM_FramefromCAM = zeros(3,3, ui32NumOfEntries, "double");

            for idx = 1:ui32NumOfEntries
                dDCM_FramefromCAM(:,:, idx) = [dCamDirX_Frame(:, idx), dCamDirY_Frame(:, idx), dCamBoresightZ_Frame(:, idx)];
            end
            % Get Quaternion corresponding to the DCM
            % dQuat_INfromCAM = self.DCM2quat_(i_dDCM_fromCAMtoIN, i_bIS_VSRPplus);
            dOutRot3 = dDCM_FramefromCAM; % TEMPORARY
            if options.enumOutRot3Param ~= EnumRotParams.DCM 
                warning('Current version only supports EnumRotParams.DCM output parameterization.')
            end
        end

        % TODO
        function self = pointToTarget_LVLH(self)
            arguments
                self
            end
            error('Not implemented yet')
        end

        % PARTIAL TODO
        function [self, dOutRot3, dDCM_FrameFromCAM] = pointToTarget_SunDirConstraint(self, dCameraPosition_Frame, ...
                dTargetPosition_Frame, ...
                dSunPosition_Frame, ...
                options)
            arguments (Input)
                self
                dCameraPosition_Frame (3, :) double = self.dCameraPosition_Frame
                dTargetPosition_Frame (3, :) double = self.dTargetPosition_Frame
                dSunPosition_Frame    (3, :) double = self.dSunPosition_Frame
            end
            arguments (Input)
                options.enumOutRot3Param (1,1) EnumRotParams {isa(options.enumOutRot3Param, ...
                    'EnumRotParams')} = EnumRotParams.DCM
            end
            arguments (Output)
                self    
                dOutRot3                  {mustBeNumeric, mustBeNonNan, mustBeFinite}
                dDCM_FrameFromCAM (3,3,:) {mustBeNumeric, mustBeNonNan, mustBeFinite}
            end
            
            ui32NumOfEntries = uint32(size(dCameraPosition_Frame, 2));
            assert(ui32NumOfEntries == size(dTargetPosition_Frame, 2) || size(dTargetPosition_Frame, 2) == 1)
            assert(ui32NumOfEntries == size(dSunPosition_Frame, 2) || size(dSunPosition_Frame, 2) == 1)
        
            assert( all(vecnorm(dSunPosition_Frame, 2, 1) ~= 0 ), "Invalid input data: Sun position cannot be zero.");

            % Construct camera boresight
            dCamBoresightZ_Frame = -(dCameraPosition_Frame./vecnorm(dCameraPosition_Frame, 2, 1));

            % TODO: Add "displacement option here"

            % Construct Y axis to satisfy Sun orthogonality constraint
            dCamPosFromSun_Frame = dCameraPosition_Frame - dSunPosition_Frame;

            dCamDirY_Frame = -cross(dCamPosFromSun_Frame./vecnorm(dCamPosFromSun_Frame, 2, 1),...
                dCamBoresightZ_Frame, 1);

            dCamDirY_Frame = dCamDirY_Frame./vecnorm(dCamDirY_Frame, 2, 1);

            % Complete frame
            dCamDirX_Frame = cross(dCamDirY_Frame, dCamBoresightZ_Frame, 1);
            dCamDirX_Frame = dCamDirX_Frame./vecnorm(dCamDirX_Frame, 2, 1);
    
            % Assemble DCM
            dDCM_FrameFromCAM = zeros(3, 3, ui32NumOfEntries, "double");

            for idx = 1:ui32NumOfEntries
                dDCM_FrameFromCAM(:,:, idx) = [dCamDirX_Frame(:, idx), dCamDirY_Frame(:, idx), dCamBoresightZ_Frame(:, idx)];
            end

            dOutRot3 = dDCM_FrameFromCAM; % TEMPORARY
            if options.enumOutRot3Param ~= EnumRotParams.DCM
                warning('Current version only supports EnumRotParams.DCM output parameterization.')
            end

            % i_dq_TFfromIN             = zeros(ui32nPoses, 4);
            % i_dq_CAMfromIN_forBlender = zeros(ui32nPoses, 4);
            % 
            % for idP = 1:ui32nPoses
            % 
            %     i_dDCM_TFfromIN(:, :, idP) = transpose(dDCM_Target_INfromTB(:,:, idP));
            %     % Matix is built like [Xrow; Yrow; Zrow] where the axes are the cameras ones in Inertial
            %     dDCM_CAMfromIN(:,:, idP) = [dCamDirX_Frame(idP, :); dCamDirY_Frame(idP, :); dCamBoresightZ_Frame(idP, :)];

                % Construct attitude quaternion for Blender

                % Generate quaternion as required by Blender
                % i_bQUAT2BLENDER = true;
                % bINVERSE_Z_AXIS = true;
                % i_bIS_JPL_CONV = false;
                % i_dq_CAMfromIN_forBlender(idP, :) = simulateTBpointing_PosOnly(dZaxisCam_IN(:, idP), i_drTargetBody_IN(idP, :), ...
                %     i_bIS_JPL_CONV, bINVERSE_Z_AXIS, i_bQUAT2BLENDER);
                % dXdir_IN = i_dDCM_CAMfromIN(1, :, idT);
                % dYdir_IN = i_dDCM_CAMfromIN(2, :, idT);
                % dZdir_IN = i_dDCM_CAMfromIN(3, :, idT);

            %     dDCM_tmp_forBlender(:,:, idP) = [-dCamDirX_Frame(idP, :); dCamDirY_Frame(idP, :); -dCamBoresightZ_Frame(idP, :)]; %#ok<SAGROW>
            % 
            %     i_dq_CAMfromIN_forBlender(idP, :) = DCM2quat(dDCM_tmp_forBlender(:,:, idP), false);
            % 
            %     % Convert to attitude quaternion through attitude matrix
            %     i_dq_TFfromIN(idP, :) = DCM2quat(i_dDCM_TFfromIN(:,:,idP), false);
            % 
            % end


        end

        % TODO
        function self = setGeneratorState(self)
            arguments
                self
            end
            error('Not implemented yet')

        end
        
        % TODO
        function self = printGeneratorState(self)
           arguments
                self
            end
            error('Not implemented yet')

        end

    end

    methods (Static, Access = public)
        % TODO implement by moving class methods here and leaving only the call there.
        % Static methods implementations (called by instance methods)
        function [dOutRot3, dDCM_FrameFromCAM] = pointToTargetStatic_SunDirConstraint(dCameraPosition_Frame, ...
            dTargetPosition_Frame, ...
            dSunPosition_Frame, ...
            options)

        end

        function [] = pointToTargetStatic_LVLH()
            arguments
                
            end
            error('Not implemented yet')
        end

        function [dOutRot3, dDCM_FrameFromCAM] = pointToTargetStatic_positionOnly(dCameraPosition_Frame, ...
                dTargetPosition_Frame, ...
                dSunPosition_Frame, ...
                options)
            error('Not implemented yet')

        end

        % Auxiliary functions
        function [self, dCameraPositionToPoint_Frame, dCamBoresight_Frame] = displaceBoresightAlongSunRays(self, ...
                dCameraPosition_Frame, ...
                dSunPosition_Frame, ...
                dReferenceDistance, ...
                settings)
            arguments
                self
                dCameraPosition_Frame (3,1) double {isvector}
                dSunPosition_Frame    (3,1) double {isvector}
                dReferenceDistance    (1,1) double {isscalar}
            end
            arguments
                settings.dScaleSigma (1,1) double {isscalar, mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
            end

            error('Not implemented yet')

        end
        
        % TODO
        function [] = plot3DtrajectoryAndBoresight()
            arguments

            end
            disp('Not implemented yet')
            % title('Camera positions, boresights and Sun directions')
        end
    end

    methods (Access = protected)
        % METHODS
        % TODO
        function [self, dCameraPositionToPoint_Frame, dCamBoresight_Frame] = displaceBoresightAlongSunRays_(self, ...
                dCameraPosition_Frame, ...
                dSunPosition_Frame, ...
                dReferenceDistance, ...
                settings)
            arguments
                self
                dCameraPosition_Frame (3,1) double {isvector} % Defaults in self
                dSunPosition_Frame    (3,1) double {isvector}
                dReferenceDistance    (1,1) double {isscalar}
            end
            arguments
                settings.dScaleSigma (1,1) double {isscalar, mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
            end

                        error('Not implemented yet')

        end
    
    
        % Internal implementation (temporary)
        function dq_INfromCAM = DCM2quat_(self, i_dDCM_fromCAMtoIN, i_bIS_VSRPplus)
            arguments
                self
                i_dDCM_fromCAMtoIN
                i_bIS_VSRPplus

            end

        end

        % Sanity checks internal functions
        function [] = assertPositionValidity(self)
            arguments
                self
            end
        end

        function [] = assertAttitudeValidity(self)
            arguments
                self
            end
        end
    end

    % Abstract methods
    % methods (Abstract, Access=protected)
    % 
    % end



end
