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
        function [self, dCameraPositionToPoint_Frame, dCamBoresight_Frame] = displaceBoresightAlongSunRays(self, dReferenceDistance, ...
                                                                                                                 dCameraPosition_Frame, ...
                                                                                                                 dSunPosition_Frame, ...
                                                                                                                 settings)
            arguments
                self
                dReferenceDistance    (1,:) double {isscalar}
                dCameraPosition_Frame (3,:) double {isvector} = self.dCameraPosition_Frame;
                dSunPosition_Frame    (3,:) double {isvector} = self.dSunPosition_Frame;
            end
            arguments
                settings.dScaleSigma (1,:) double {mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
            end

            % Call equivalent static method
            [dCameraPositionToPoint_Frame, dCamBoresight_Frame] = self.DisplaceBoresightAlongSunRays(dCameraPosition_Frame, ...
                                                                                                    dSunPosition_Frame, ...
                                                                                                    dReferenceDistance, ...
                                                                                                    "dScaleSigma", settings.dScaleSigma);


        end


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

        function [self, dOutRot3, dDCM_FrameFromPose] = pointToTarget_SunDirConstraint(self, ...
                                                                                    dCameraPosition_Frame, ...
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
                options.dDCM_displacedPoseFromPose      (3,3,:) double {ismatrix, isnumeric} = zeros(3) % Custom rotation to apply to the rotation
                options.dSigmaDegRotAboutBoresight      (1,1)   double {isscalar, isnumeric} = 0.0 % Sigma to scatter camera pose around boresight
                options.dSigmaOffPointingDegAlongSun       (1,1)   double {isscalar, isnumeric} = 0.0 % Sigma to scatter camera boresight along a direction pointing to Sun
            end
            arguments (Output)
                self    
                dOutRot3                  {mustBeNumeric, mustBeNonNan, mustBeFinite}
                dDCM_FrameFromPose        (3,3,:) {mustBeNumeric, mustBeNonNan, mustBeFinite}
            end
            
            ui32NumOfEntries = uint32(size(dCameraPosition_Frame, 2));
            assert(ui32NumOfEntries == size(dTargetPosition_Frame, 2) || size(dTargetPosition_Frame, 2) == 1)
            assert(ui32NumOfEntries == size(dSunPosition_Frame, 2) || size(dSunPosition_Frame, 2) == 1)
        
            assert( all(vecnorm(dSunPosition_Frame, 2, 1) ~= 0 ), "Invalid input data: Sun position cannot be zero.");

            % Construct camera boresight
            dCamBoresightZ_Frame = -(dCameraPosition_Frame./vecnorm(dCameraPosition_Frame, 2, 1));

             
            % Scatter boresight along Sun rays
            if options.dSigmaOffPointingDegAlongSun > 0.0
                
                % Compute scale sigma from angle
                dScaleSigma = vecnorm(dCameraPosition_Frame, 2, 1) * tan(deg2rad(options.dSigmaOffPointingDegAlongSun));


                [self, ~, dCamBoresightZ_Frame] = displaceBoresightAlongSunRays(self, 0.0, ...
                                                                            dCameraPosition_Frame, ...
                                                                            dSunPosition_Frame, ...
                                                                            "dScaleSigma", dScaleSigma);

            end



            % Construct Y axis to satisfy Sun orthogonality constraint
            dCamPosFromSun_Frame = dSunPosition_Frame - dCameraPosition_Frame;

            dCamDirY_Frame = cross(dCamBoresightZ_Frame, dCamPosFromSun_Frame./vecnorm(dCamPosFromSun_Frame, 2, 1), 1);
            dCamDirY_Frame = dCamDirY_Frame./vecnorm(dCamDirY_Frame, 2, 1);

            % Rotate Y axis of scatter angles if not zero
            if options.dSigmaDegRotAboutBoresight > 0

                % Sample rotation angle in [rad]
                assert(options.dSigmaDegRotAboutBoresight > 0, 'ERROR: a variance cannot be negative!');
                dScatterBoresightAngle = deg2rad(options.dSigmaDegRotAboutBoresight) * randn(1, size(dCamBoresightZ_Frame, 2));
                dCamDirY_Frame = Rot3dVecAboutDir(dCamBoresightZ_Frame, dCamDirY_Frame, dScatterBoresightAngle);

            end

            % Complete frame
            dCamDirX_Frame = cross(dCamDirY_Frame, dCamBoresightZ_Frame, 1);
            dCamDirX_Frame = dCamDirX_Frame./vecnorm(dCamDirX_Frame, 2, 1);
    
            % Assemble DCM
            dDCM_FrameFromPose = zeros(3, 3, ui32NumOfEntries, "double");

            for idx = 1:ui32NumOfEntries
                dDCM_FrameFromPose(:,:, idx) = [dCamDirX_Frame(:, idx), dCamDirY_Frame(:, idx), dCamBoresightZ_Frame(:, idx)];
            end

            % Apply additional custom rotation if required
            if any( options.dDCM_displacedPoseFromPose ~= zeros(3,3, size(options.dDCM_displacedPoseFromPose, 3)) )
                dDCM_FrameFromPose(:,:,:) = pagetranspose(pagemtimes(options.dDCM_displacedPoseFromPose, pagetranspose(dDCM_FrameFromPose)));
            end

            dOutRot3 = dDCM_FrameFromPose; % TEMPORARY

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

        function [dCamDirY_Frame] = RandomGuassianBoresightRoll(dCamBoresightZ_Frame, ...
                                                                dCamDirY_Frame, ...
                                                                dSigmaDegRotAboutBoresight)
            arguments
                dCamBoresightZ_Frame 
                dCamDirY_Frame 
                dSigmaDegRotAboutBoresight 
            end

                % Sample rotation angle in [rad]
                assert(dSigmaDegRotAboutBoresight > 0, 'ERROR: a variance cannot be negative!');
                dScatterBoresightAngle = deg2rad(dSigmaDegRotAboutBoresight) * randn(1, size(dCamBoresightZ_Frame, 2));
                dCamDirY_Frame = Rot3dVecAboutDir(dCamBoresightZ_Frame, dCamDirY_Frame, dScatterBoresightAngle);
        end

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
        function [dCameraPositionToPoint_Frame, dCamBoresightUnitVec_Frame] = DisplaceBoresightAlongSunRays(dBoresightVector_Frame, ...
                                                                                                    dRefPoint_Frame, ...
                                                                                                    dReferenceDistance, ...
                                                                                                    settings)
            arguments
                dBoresightVector_Frame  (3,:) double {isvector, isnumeric}
                dRefPoint_Frame         (3,:) double {isvector, isnumeric} 
                dReferenceDistance      (1,:) double {isnumeric} = 0;
            end
            arguments
                settings.dScaleSigma    (1,:) double {mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
            end
            %% SIGNATURE
            % [dCameraPositionToPoint_Frame, dCamBoresight_Frame] = DisplaceBoresightAlongSunRays(dBoresightVector_Frame, ...
            %                                                                             dRefPoint_Frame, ...
            %                                                                             dReferenceDistance, ...
            %                                                                             settings)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Function displacing the boresight vector along a direction determined by dRefPoint_Frame in 3D
            % space. dReferenceDistance is used as constant offset for the displacement, scattered of a
            % value given by the optional scale sigma value (Gaussian distribution). Function supports
            % vectorized mode.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % dCameraPosition_Frame (3,:) double {isvector, isnumeric}
            % dRefPoint_Frame       (3,:) double {isvector, isnumeric}
            % dReferenceDistance    (1,:) double {isscalar, isnumeric}
            % settings.dScaleSigma  (1,:) double {isscalar, mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dCameraPositionToPoint_Frame
            % dCamBoresight_Frame
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 21-03-2025    Pietro Califano     Implement from previous function code, update for vect.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Future upgrades
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Function code
            ui32NumOfSamples = size(dBoresightVector_Frame, 2);
            assert( length(settings.dScaleSigma) == 1 || length(settings.dScaleSigma) == ui32NumOfSamples);
            
            dDeltaDir = zeros(3, ui32NumOfSamples);

            % Compute direction delta unit vector
            % TODO: verify whether the commented code is functionally equivalent
            % dDeltaDir(1:3) = dot(dCameraPosition_Frame, dCameraPosition_Frame) * dSunPosition_Frame ...
            %                 - dot(dCameraPosition_Frame, dSunPosition_Frame) * dCameraPosition_Frame;
            %
            % dDeltaDir(1:3) = dDeltaDir./norm(dDeltaDir); % Displacement along a perpendicular to position vector in the plane defined by this and the sun directions

            % Direction orthogonal to camera position and Sun position plane
            dRefPointNorms  = vecnorm(dRefPoint_Frame, 2, 1);
            dBoresightNorms = vecnorm(dBoresightVector_Frame, 2, 1);

            dAuxDir1 = cross(dRefPoint_Frame./dRefPointNorms, -dBoresightVector_Frame./dBoresightNorms, 1);
            dAuxDir1 = dAuxDir1./vecnorm(dAuxDir1, 2, 1);

            % Direction orthogonal to plane formed by camera position and AuxDir1, toward the Sun
            dDeltaDir(:, :) = cross(-dBoresightVector_Frame./dBoresightNorms, dAuxDir1, 1);
            dDeltaDir(:, :) = dDeltaDir./vecnorm(dDeltaDir, 2, 1);

            % Set scale of change
            if any(settings.dScaleSigma > 0)
                dScaleScatterValue = settings.dScaleSigma .* randn(1, ui32NumOfSamples);
            else
                dScaleScatterValue = 0;
            end

            % Compute displacement scale and add random coefficient to randomize pointing "across" the limb
            dScaleModulus = dReferenceDistance + dScaleScatterValue;

            % Compute new camera position in Frame from look_at point
            dCameraPositionToPoint_Frame = dBoresightVector_Frame - (dScaleModulus .* dDeltaDir);

            if nargout > 1
                dCamBoresightUnitVec_Frame = dCameraPositionToPoint_Frame./vecnorm(dCameraPositionToPoint_Frame, 2, 1);
            end
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
