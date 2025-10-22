classdef CAttitudePointingGenerator < handle
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 08-12-2024    Pietro Califano     Class implemented from adapted codes for pointing generation
    % 12-02-2025    Pietro Califano     Update of class to add Sun position as input and static methods
    % 22-03-2025    Pietro Califano     [MAJOR] Upgrade class usage with new general-purpose methods
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = protected, GetAccess = public)

        dCameraPosition_Frame   = -ones(3,1);
        dTargetPosition_Frame   = -ones(3,1);
        dSunPosition_Frame      = zeros(3,1);
        dVelocity_Frame         = zeros(3,1);
        dAuxiliaryAxis          = [1;0;0];

        % bInvertZaxisForBlender    = false;
        bShowAttitudePointingPlot = false;
    end


    methods (Access = public)
        %% CONSTRUCTOR
        function self = CAttitudePointingGenerator(dCameraPosition_Frame, dTargetPosition_Frame, dSunPosition_Frame, options)
            arguments
                dCameraPosition_Frame {mustBeNumeric} = -ones(3,1); % Defaults to placeholder value
                dTargetPosition_Frame {mustBeNumeric} = -ones(3,1); 
                dSunPosition_Frame    {mustBeNumeric} = -ones(3,1); 
            end

            arguments
                options.bVerbose                    (1,1) logical {islogical, isscalar} = false;
                options.bShowAttitudePointingPlot   (1,1) logical {islogical, isscalar} = false;
                % options.bInvertZaxisForBlender      (1,1) logical {islogical, isscalar} = false;
            end

            % Assign properties
            self.dCameraPosition_Frame      = dCameraPosition_Frame;
            self.dTargetPosition_Frame      = dTargetPosition_Frame;
            self.dSunPosition_Frame         = dSunPosition_Frame;
            self.bShowAttitudePointingPlot  = options.bShowAttitudePointingPlot;

            % self.bInvertZaxisForBlender = options.bInvertZaxisForBlender;

        end

        % GETTERS

        % SETTERS

        % PUBLIC METHODS
        % Main entry-point function (all calls)
        function [self, dOutRot3, dDCM_FrameFromPose, dOffPointingAngles] = pointToTarget(self, ...
                                                                                        dCameraPosition_Frame, ...
                                                                                        dTargetPosition_Frame, ...
                                                                                        kwargs, ...
                                                                                        options)
            arguments (Input)
                self
                dCameraPosition_Frame (3,:) double = self.dCameraPosition_Frame
                dTargetPosition_Frame (3,:) double = self.dTargetPosition_Frame
            end
            arguments (Input)
                kwargs.dSunPosition_Frame   (3,:) double = self.dSunPosition_Frame;
                kwargs.dVelocity_Frame      (3,:) double = self.dVelocity_Frame;
                kwargs.dAuxiliaryAxis       (3,:) double = self.dAuxiliaryAxis;
            end
            arguments (Input)
                options.bInputIsTargetDirectionFromCam  (1,1) logical = false
                options.enumConstraintType              (1,:) string {mustBeMember(options.enumConstraintType, ["YorthogonalSun", "trackLVLH", "auxiliaryAxis"])} = "YorthogonalSun"
                options.enumOutRot3Param                (1,1) EnumRotParams {isa(options.enumOutRot3Param, 'EnumRotParams')} = EnumRotParams.DCM
                options.dDCM_displacedPoseFromPose      (3,3,:) double {mustBeNumeric} = zeros(3,3) % Custom rotation to apply to the rotation
                options.dSigmaDegRotAboutBoresight      (1,:)   double {mustBeNumeric} = 0.0 % Sigma to scatter camera pose around boresight
                options.dSigmaOffPointingDegAngle       (1,:)   double {mustBeNumeric} = 0.0 % Sigma to scatter camera boresight in a random direction
                options.enumOffPointingMode             (1,1) string {mustBeMember(options.enumOffPointingMode, ["randomAxis", "refAxisOutOfPlane", "refAxisInPlane"])} = "randomAxis";
                options.dReferenceAxis_Frame            (3,:) double {mustBeNumeric} = zeros(3,0)
                options.enumDisplaceDistribution           (1,:) string {mustBeMember(options.enumDisplaceDistribution, ...
                                                            ["uniform", "gaussian", "time_correlation", "gaussian_same_on_batch", "uniform_same_on_batch"])} = "uniform";            
            end
            arguments (Output)
                self
                dOutRot3                  {mustBeNumeric, mustBeNonNan, mustBeFinite}
                dDCM_FrameFromPose        (3,3,:) {mustBeNumeric, mustBeNonNan, mustBeFinite}
                dOffPointingAngles        (:,1)   double {mustBeNumeric}
            end

            % Determine number of entries
            ui32NumOfEntries = uint32(size(dCameraPosition_Frame, 2));
            
            % Perform input checks (common)
            assert(ui32NumOfEntries == size(dTargetPosition_Frame, 2) || size(dTargetPosition_Frame, 2) == 1)
            
            charOffPoint = "";
            charBoreRoll = "";

            % Perform input checks (cosntraint specific)
            if options.enumConstraintType == "YorthogonalSun"
                % YorthogonalSun constraint
                assert(ui32NumOfEntries == size(kwargs.dSunPosition_Frame, 2) || size(kwargs.dSunPosition_Frame, 2) == 1)
                assert( all(vecnorm(kwargs.dSunPosition_Frame, 2, 1) ~= 0 ), ...
                    "Invalid input data: Sun position cannot be zero for YorthogonalSun contraint type.");

                assert(any(vecnorm(kwargs.dSunPosition_Frame) ~= 1, "all"), ['ERROR: invalid Sun positions. ' ...
                    'Must be provided either when constructing the class or when calling this method.']);

            elseif options.enumConstraintType == "trackLVLH"
                % trackLVLH constraint 
                assert(ui32NumOfEntries == size(kwargs.dVelocity_Frame, 2) || size(kwargs.dVelocity_Frame, 2) == 1)
                assert( all(vecnorm(kwargs.dVelocity_Frame, 2, 1) ~= 0 ), "ERROR: invalid input data: velocity vectors cannot be zero for trackLVLH contraint type.");

            elseif options.enumConstraintType == "auxiliaryAxis"
                % Normalize if not unit vectors
                if any(abs(kwargs.dAuxiliaryAxis) > 1)
                    kwargs.dAuxiliaryAxis = vecnorm(kwargs.dAuxiliaryAxis, 2,1);
                end

            else
                % Invalid option
                error('Invalid constraint type. Must be one of [auxiliaryAxis, trackLVLH, YorthogonalSun]. Got %s.', string(options.enumConstraintType) );
            end

            % Assign temporary variables
            dSunPosition_Frame_ = kwargs.dSunPosition_Frame;
            dVelocity_Frame_ = kwargs.dVelocity_Frame;
            dAuxiliaryAxis_ = kwargs.dAuxiliaryAxis;

            % Compute camera boresight from lookAt point
            if options.bInputIsTargetDirectionFromCam
                dLookAtPointFromCam_Frame = dTargetPosition_Frame;
            else
                dLookAtPointFromCam_Frame = dTargetPosition_Frame - dCameraPosition_Frame;
            end


            dCamBoresightZ_Frame = dLookAtPointFromCam_Frame./vecnorm(dLookAtPointFromCam_Frame, 2, 1);
            % Is3dPointOn3dLine(dTargetPosition_Frame(:,1), dCameraPosition_Frame(:,1), dCamBoresightZ_Frame(:,1), 1E-6);

            % Scatter boresight TODO
            if any(options.dSigmaOffPointingDegAngle > 0.0)
                    
                %function [dCamBoresightAxis_Frame, dCamLookAtPoint_Frame] = ApplyAxisOffPointing(dCamLookAtPoint_Frame, ...
                %                                                                             dReferenceAxis_Frame, ...
                %                                                                            kwargs, ...
                %                                                                            options)
                % arguments
                %     dCamLookAtPoint_Frame   (3,:) double {mustBeNumeric}
                %     dReferenceAxis_Frame    (3,:) double {mustBeNumeric} = zeros(3,0)
                % end
                % arguments
                %     kwargs.dDisplaceValue               (1,:) double {mustBeNumeric} = 0; % Interpreted as angle or distance depending on enumDisplacementMode
                %     options.enumDisplacementMethod      (1,1) string {mustBeMember(options.enumDisplacementMethod, ["lookAtPoint", "rotate3d"])} = "lookAtPoint";
                %     options.dSigmaOffPointingDegAngle   (1,:) double {isscalar, mustBeNumeric} = 0.0 % Sigma to scatter camera boresight in a random direction
                %     options.dScaleDistOffPointing       (1,:) double {isscalar, mustBeNumeric} = 0.0
                %     options.enumOffPointingMode         (1,1) string {mustBeMember(options.enumOffPointingMode, ["randomAxis", "refAxisOutOfPlane", "refAxisInPlane"])} = "randomAxis";
                % end

                dCamBoresightZ_Frame = self.ApplyAxisOffPointing(dLookAtPointFromCam_Frame, ...
                                                                 options.dReferenceAxis_Frame, ...
                                                                 "enumDisplacementMethod", "rotate3d", ...
                                                                 "dSigmaOffPointingDegAngle", options.dSigmaOffPointingDegAngle, ...
                                                                 "enumOffPointingMode", options.enumOffPointingMode, ...
                                                                 "enumDisplaceDistribution", options.enumDisplaceDistribution);

                % Compute off-pointing half-cone angle
                dOffPointingAngles = transpose(acosd(dot(dLookAtPointFromCam_Frame./vecnorm(dLookAtPointFromCam_Frame, 2, 1), dCamBoresightZ_Frame)));

                charOffPoint = "+ off-pointing";
            else
                dOffPointingAngles = zeros(ui32NumOfEntries, 1);
            end

            if not(all(kwargs.dAuxiliaryAxis == 0))
                self.dAuxiliaryAxis = kwargs.dAuxiliaryAxis;
            end

            % Construct camera Y axis 
            dCamDirY_Frame = self.DefineYaxisFromConstraint(dCameraPosition_Frame, ...
                                                            dCamBoresightZ_Frame, ...
                                                            "enumConstraintType", options.enumConstraintType, ...
                                                            "dSunPosition_Frame", dSunPosition_Frame_, ...
                                                            "dVelocity_Frame", dVelocity_Frame_, ...
                                                            "dAuxiliaryAxis", dAuxiliaryAxis_);
                                                                     
            if any(isnan(dCamDirY_Frame), 'all')
                error('Detected "nan" in rotation matrices. Something may have gone wrong in constructing the Y axes. Please check inputs.')
            end

            % Rotate Y axis of scatter angles if not zero
            if options.dSigmaDegRotAboutBoresight > 0

                if contains(lower(options.enumDisplaceDistribution), "same")
                    bSameOnBatch = true;
                else
                    bSameOnBatch = false;
                end

                [dCamDirY_Frame] = self.ApplyRandomGuassianBoresightRoll(dCamBoresightZ_Frame, ...
                                                                    dCamDirY_Frame, ...
                                                                    options.dSigmaDegRotAboutBoresight, ...
                                                                    bSameOnBatch);

                charBoreRoll = "+ boresight roll";
            end

            % Complete frame enforcing right-handed triad constraint
            dCamDirX_Frame = cross(dCamDirY_Frame, dCamBoresightZ_Frame, 1);
            dCamDirX_Frame = dCamDirX_Frame./vecnorm(dCamDirX_Frame, 2, 1);
        
            % Assemble DCMs
            dDCM_FrameFromPose = zeros(3, 3, ui32NumOfEntries, "double");
            for idx = 1:ui32NumOfEntries
                dDCM_FrameFromPose(:,:, idx) = [dCamDirX_Frame(:, idx), dCamDirY_Frame(:, idx), dCamBoresightZ_Frame(:, idx)];
            end

            if any(isnan(dDCM_FrameFromPose),'all')
                error('Detected "nan" in rotation matrices. Something may have gone wrong in constructing the X-Y-Z axes. Please check inputs.')
            end

            % Apply additional custom rotation if provided (e.g. to account for sensor poses)
            if any( options.dDCM_displacedPoseFromPose ~= zeros(3,3, size(options.dDCM_displacedPoseFromPose, 3)) )
                dDCM_FrameFromPose(:,:,:) = pagetranspose(pagemtimes(options.dDCM_displacedPoseFromPose, pagetranspose(dDCM_FrameFromPose)));
            end

            % Define output according to required parameterization TODO
            switch options.enumOutRot3Param
                case EnumRotParams.DCM
                    % Copy only
                    dOutRot3 = dDCM_FrameFromPose;

                case EnumRotParams.MRP
                    % Convert DCM to MRP % TODO
                    error('Not implemented yet')
                case EnumRotParams.QUAT_VSRPplus
                    % Convert DCM to QUAT_VSRPplus (scalar last)
                    dOutRot3 = DCM2quatSeq(dDCM_FrameFromPose, true);

                case EnumRotParams.QUAT_SVRPplus
                    % Convert DCM to QUAT_VSRPplus (scalar first)
                    dOutRot3 = DCM2quatSeq(dDCM_FrameFromPose, false);

                otherwise
                    error('Invalid rotation parameterization. Please select one defined in EnumRotParams class.')
            end


            if self.bShowAttitudePointingPlot == true
                
                dSceneEntityDCM_RenderFrameFromOF = eye(3,3);
                objSceneFig = 0;

                % Construct figure with plot
                for idE = ceil(linspace(1, double(ui32NumOfEntries), min(50, ui32NumOfEntries)))

                    if size(dTargetPosition_Frame, 2) == 1
                        dTargetPosition_Frame_ = dTargetPosition_Frame;
                    else
                        dTargetPosition_Frame_ = dTargetPosition_Frame(:, idE);
                    end

                    [objSceneFig, cellFramesAxesGlobal] = PlotSceneFrames_DCM(dTargetPosition_Frame_, ...
                                                                            dSceneEntityDCM_RenderFrameFromOF, ...
                                                                            dCameraPosition_Frame(:, idE), ...
                                                                            transpose(dDCM_FrameFromPose(:,:,idE)), ...
                                                                            'bUseBlackBackground', true, ...
                                                                            "charFigTitle", sprintf("Attitude pointing generator output %s %s", charBoreRoll, charOffPoint), ...
                                                                            "objFig", objSceneFig, ...
                                                                            "bEnableLegend", false, ...
                                                                            "cellPlotColors", {"r", "g", "b", "w", "w", "w"});
                end
    
                legend([cellFramesAxesGlobal{:}], 'TextColor', 'w');
            end
        end

        % LEGACY FUNCTION (keep to prevent breaking codes)
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
                options.dDCM_displacedPoseFromPose      (3,3,:) double {mustBeNumeric} = zeros(3) % Custom rotation to apply to the rotation
                options.dSigmaDegRotAboutBoresight      (1,1)   double {isscalar, mustBeNumeric} = 0.0 % Sigma to scatter camera pose around boresight
            end
            arguments (Output)
                self    
                dOutRot3                  {mustBeNumeric, mustBeNonNan, mustBeFinite}
                dDCM_FrameFromPose        (3,3,:) {mustBeNumeric, mustBeNonNan, mustBeFinite}
            end
            
            ui32NumOfEntries = uint32(size(dCameraPosition_Frame, 2));
            
            % Perform input checks
            assert(ui32NumOfEntries == size(dTargetPosition_Frame, 2) || size(dTargetPosition_Frame, 2) == 1)
            assert(ui32NumOfEntries == size(dSunPosition_Frame, 2) || size(dSunPosition_Frame, 2) == 1)
            assert( all(vecnorm(dSunPosition_Frame, 2, 1) ~= 0 ), "Invalid input data: Sun position cannot be zero.");

            % Compute camera boresight from lookAt point
            dLookAtPointFromCam_Frame = dTargetPosition_Frame - dCameraPosition_Frame;
            dCamBoresightZ_Frame = -(dLookAtPointFromCam_Frame./vecnorm(dLookAtPointFromCam_Frame, 2, 1));
            
            % Construct Y axis to satisfy Sun orthogonality constraint
            dCamPosFromSun_Frame = dSunPosition_Frame - dCameraPosition_Frame;

            dCamDirY_Frame = cross(dCamBoresightZ_Frame, dCamPosFromSun_Frame./vecnorm(dCamPosFromSun_Frame, 2, 1), 1);
            dCamDirY_Frame = dCamDirY_Frame./vecnorm(dCamDirY_Frame, 2, 1);

            % Rotate Y axis of scatter angles if not zero
            if options.dSigmaDegRotAboutBoresight > 0

                [dCamDirY_Frame] = self.ApplyRandomGuassianBoresightRoll(dCamBoresightZ_Frame, ...
                                                                    dCamDirY_Frame, ...
                                                                    options.dSigmaDegRotAboutBoresight);

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

            dOutRot3 = dDCM_FrameFromPose; 
            if options.enumOutRot3Param ~= EnumRotParams.DCM
                warning('Current version only supports EnumRotParams.DCM output parameterization.')
            end
        end
    end

    methods (Static, Access = public)
        function [dCamDirY_Frame] = ApplyRandomGuassianBoresightRoll(dCamBoresightZ_Frame, ...
                                                                    dCamDirY_Frame, ...
                                                                    dSigmaDegRotAboutBoresight, ...
                                                                    bSameOnBatch)
            arguments
                dCamBoresightZ_Frame 
                dCamDirY_Frame 
                dSigmaDegRotAboutBoresight 
                bSameOnBatch (1,1) {islogical} = false;
            end
            % Sampling functions for boresight roll

                % Sample rotation angle in [rad]
                assert(dSigmaDegRotAboutBoresight > 0, 'ERROR: a variance cannot be negative!');

                if bSameOnBatch
                    ui32NumOfSamples = 1;
                else
                    ui32NumOfSamples = size(dCamBoresightZ_Frame, 2);
                end

                dScatterBoresightAngle = deg2rad(dSigmaDegRotAboutBoresight) * randn(1, ui32NumOfSamples);
                dCamDirY_Frame = Rot3dVecAboutDir(dCamBoresightZ_Frame, dCamDirY_Frame, dScatterBoresightAngle);
        end


        function [dCamDirY_Frame] = DefineYaxisFromConstraint(dCameraPosition_Frame, dCamBoresightZ_Frame, kwargs, options)
            arguments (Input)
                dCameraPosition_Frame (3, :) double {mustBeNumeric}
                dCamBoresightZ_Frame  (3, :) double {mustBeNumeric}
            end
            arguments (Input)
                kwargs.dSunPosition_Frame   (3, :) double = zeros(3,1);
                kwargs.dVelocity_Frame      (3, :) double = zeros(3,1);
                kwargs.dAuxiliaryAxis       (3, :) double = zeros(3,1);
            end
            arguments (Input)
                options.enumConstraintType      (1,:) string {mustBeMember(options.enumConstraintType, ["YorthogonalSun", "trackLVLH", "auxiliaryAxis"])} = "YorthogonalSun"
            end
            % Function defining the Y axis of the camera frame according to the constraint mode

            switch options.enumConstraintType
                case "YorthogonalSun"
                    assert( all(vecnorm(kwargs.dSunPosition_Frame, 2, 1) ~= 0 ), "Invalid input data: Sun position cannot be zero for YorthogonalSun contraint type." );

                    % Construct Y axis to satisfy Sun orthogonality constraint
                    dCamPosFromSun_Frame = kwargs.dSunPosition_Frame - dCameraPosition_Frame;
                    dCamDirY_Frame = cross(dCamBoresightZ_Frame, dCamPosFromSun_Frame./vecnorm(dCamPosFromSun_Frame, 2, 1), 1);
                
                    dCamDirYnorms = vecnorm(dCamDirY_Frame, 2, 1);

                    assert( all(dCamDirYnorms > eps , 'all' ), ['ERROR: camera boresight and position of ' ...
                        'the camera from light are parallel within machine precision. Cannot attitude pointing due to ambiguity!'])

                    dCamDirY_Frame = dCamDirY_Frame./dCamDirYnorms;

                    return

                case "trackLVLH"
                    assert( all(vecnorm(kwargs.dVelocity_Frame, 2, 1) ~= 0 ), "Invalid input data: velocity vectors cannot be zero for trackLVLH contraint type." );
                    % Construct Y axis to satisfy trackLVLH constraint (+X along velocity)
                    dCamDirY_Frame = cross(dCamBoresightZ_Frame, kwargs.dVelocity_Frame./vecnorm(kwargs.dVelocity_Frame, 2, 1), 1);
                    dCamDirY_Frame = dCamDirY_Frame./vecnorm(dCamDirY_Frame, 2, 1);
                    return

                case "auxiliaryAxis"
                    assert( all(vecnorm(kwargs.dAuxiliaryAxis, 2, 1) ~= 0 ), "Invalid input data: auxiliary axis cannot be zero for auxiliaryAxis contraint type." );
                    % Construct Y axis to satisfy auxiliary axis constraint (+X axis)
                    dCamDirY_Frame = cross(dCamBoresightZ_Frame, repmat(kwargs.dAuxiliaryAxis./vecnorm(kwargs.dAuxiliaryAxis, 2, 1), 1, size(dCamBoresightZ_Frame,2)), 1);
                    dCamDirY_Frame = dCamDirY_Frame./vecnorm(dCamDirY_Frame, 2, 1);
                    return
            end
        end


        function [dCamBoresightAxis_Frame, dCamLookAtPoint_Frame] = ApplyAxisOffPointing(dCamLookAtPoint_Frame, ...
                                                                                        dReferenceAxis_Frame, ...
                                                                                        kwargs,...
                                                                                        options)
            arguments
                dCamLookAtPoint_Frame   (3,:) double {mustBeNumeric}
                dReferenceAxis_Frame    (3,:) double {mustBeNumeric} = zeros(3,0)
            end
            arguments
                kwargs.dDisplaceOffsetValue         (1,:) double {isscalar, mustBeNumeric} = 0.0 % "Deterministic" offset
                options.enumDisplacementMethod      (1,1) string {mustBeMember(options.enumDisplacementMethod, ["lookAtPoint", "rotate3d"])} = "lookAtPoint";
                options.enumDisplaceDistribution    (1,:) string = "gaussian"
                options.dSigmaOffPointingDegAngle   (1,:) double {isscalar, mustBeNumeric} = 0.0 % Sigma to scatter camera boresight in a random direction
                options.dScaleDistOffPointing       (1,:) double {isscalar, mustBeNumeric} = 0.0
                options.enumOffPointingMode         (1,1) string {mustBeMember(options.enumOffPointingMode, ["randomAxis", "refAxisOutOfPlane", "refAxisInPlane"])} = "randomAxis";
            end
            
            % Verify inputs
            bConditionAngle = any(options.dSigmaOffPointingDegAngle > 0);
            bConditionScale = any(options.dScaleDistOffPointing > 0);
           
            % Compute default outputs
            dCamBoresightAxis_Frame = dCamLookAtPoint_Frame./vecnorm(dCamLookAtPoint_Frame, 2, 1);

            if strcmpi(options.enumDisplacementMethod, "lookAtPoint") && ( max( abs(options.dScaleDistOffPointing) ) - min(vecnorm(dCamLookAtPoint_Frame, 2, 1) ) > 1e-2 ) 
                warning("Displacement method is lookAtPoint, but dScaleDistOffPointing and dCamLookAtPoint_Frame are similar in magnitude. This may be unintended and will cause large off-pointing values!");
            end

            if not(bConditionAngle || bConditionScale)
                warning('No applied off-pointing: dSigmaOffPointingDegAngle and dScaleDistOffPointing both set to zero value.')
                return
            end

            % Check if both provided, override to angle by default
            if bConditionAngle && bConditionScale
                warning('Both dSigmaOffPointingDegAngle and dScaleDistOffPointing provided: using dSigmaOffPointingDegAngle by default.')
                bConditionAngle = true;
                options.enumDisplacementMethod = "rotate3d";
            end

            if bConditionAngle % Generate off-pointing from angle
                options.enumDisplacementMethod = "rotate3d";
                dDisplaceSigma = deg2rad(options.dSigmaOffPointingDegAngle);

            else % Generate of-pointing from reference distance
                options.enumDisplacementMethod = "lookAtPoint";
                dDisplaceSigma = options.dScaleDistOffPointing;

            end

            if strcmpi(options.enumOffPointingMode, "randomAxis")
                dReferenceAxis_Frame = randn(3,size(dCamLookAtPoint_Frame, 2)); % Default is random axis
                dReferenceAxis_Frame = dReferenceAxis_Frame./vecnorm(dReferenceAxis_Frame, 2, 1);
            else
                assert(size(dReferenceAxis_Frame, 2) >= 1, 'ERROR: reference axes must be provided for "refAxisOutOfPlane", "refAxisInPlane" off-pointing modes')
            end

            % Compute displaced boresight axis
            [dCamBoresightAxis_Frame(1:3, :), dCamLookAtPoint_Frame(1:3,:)] = CAttitudePointingGenerator.ComputeDisplacedBoresight(dCamLookAtPoint_Frame, ...
                                                                                                    dReferenceAxis_Frame, ...
                                                                                                    kwargs.dDisplaceOffsetValue, ...
                                                                                                    "dDisplaceSigma", dDisplaceSigma, ...
                                                                                                    "enumDisplacementMode", options.enumDisplacementMethod, ...
                                                                                                    "enumDisplaceDistribution", options.enumDisplaceDistribution);




        end

        function [dBoresightUnitVec_Frame, dNewLookAtPoint_Frame] = ComputeDisplacedBoresight(dLookAtPoint_Frame, ...
                                                                                                dReferenceAxis_Frame, ...
                                                                                                dDisplaceValue, ...
                                                                                                settings)
            arguments
                dLookAtPoint_Frame      (3,:) double {mustBeNumeric}
                dReferenceAxis_Frame    (3,:) double {mustBeNumeric} = randn(3,size(dLookAtPoint_Frame, 2)); % Default is random axis
                dDisplaceValue          (1,:) double {mustBeNumeric} = 0; % Interpreted as angle or distance depending on enumDisplacementMode
            end
            arguments
                settings.enumDisplacementMode               (1,1) string {mustBeMember(settings.enumDisplacementMode, ["lookAtPoint", "rotate3d"])} = "lookAtPoint";
                settings.dDisplaceSigma                     (1,:) double {mustBeGreaterThanOrEqual(settings.dDisplaceSigma, 0)} = 0;
                settings.bDisplaceOrthogonalToRefAxisPlane  (1,1) logical  = false
                settings.enumDisplaceDistribution           (1,:) string {mustBeMember(settings.enumDisplaceDistribution, ...
                                                            ["uniform", "gaussian", "time_correlation", "gaussian_same_on_batch", "uniform_same_on_batch"])} = "gaussian";
            end
            %% SIGNATURE
            % [dCamBoresightUnitVec_Frame, dNewLookAtPoint_Frame] = ComputeDisplacedBoresight(dLookAtPoint_Frame, ...
            %                                                                                     dReferenceAxis_Frame, ...
            %                                                                                     dDisplaceValue, ...
            %                                                                                     settings)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % This function computes the displaced boresight unit vector and the new look-at point based on the provided parameters.
            % It supports two modes of displacement: "lookAtPoint" and "rotate3d". The displacement can be defined as an angle or a distance.
            % The function can also apply a randomization of the displacement value based on a Gaussian or uniform distribution.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % arguments
            %     dLookAtPoint_Frame      (3,:) double {mustBeNumeric}
            %     dReferenceAxis_Frame    (3,:) double {mustBeNumeric} = randn(3,size(dLookAtPoint_Frame, 2)); % Default is random axis
            %     dDisplaceValue          (1,:) double {mustBeNumeric} = 0; % Interpreted as angle or distance depending on enumDisplacementMode
            % end
            % arguments
            %     settings.enumDisplacementMode               (1,1) string {mustBeMember(settings.enumDisplacementMode, ["lookAtPoint", "rotate3d"])} = "lookAtPoint";
            %     settings.dDisplaceSigma                     (1,1) double {mustBeGreaterThanOrEqual(settings.dDisplaceSigma, 0)} = 0;
            %     settings.bDisplaceOrthogonalToRefAxisPlane  (1,1) logical {islogical, isscalar} = false
            %     settings.enumDisplaceDistribution           (1,:) string {mustBeMember(settings.enumDisplaceDistribution, ["uniform", "gaussian"])} = "gaussian";
            % end
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dBoresightUnitVec_Frame
            % dNewLookAtPoint_Frame
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 21-03-2025    Pietro Califano     Implement from previous function code, update for vect.
            % 22-03-2025    Pietro Califano     [MAJOR] Reworking, provide implementation for two displacement modes
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Future upgrades
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% Function code
            ui32NumOfSamples = size(dLookAtPoint_Frame, 2);
            assert( isscalar(settings.dDisplaceSigma) || length(settings.dDisplaceSigma) == ui32NumOfSamples);
            assert( isscalar(dDisplaceValue) || length(dDisplaceValue) == ui32NumOfSamples);

            dBoresightUnitVec_Frame = zeros(3, ui32NumOfSamples);
            dNewLookAtPoint_Frame   = zeros(3, ui32NumOfSamples);
            
            % Ensure reference axis are unit vectors
            dReferenceAxis_Frame = dReferenceAxis_Frame./vecnorm(dReferenceAxis_Frame, 2, 1);

            % Apply randomization of displacement value (angle or distance)
            if settings.dDisplaceSigma > 0
            
                switch settings.enumDisplaceDistribution

                    case "gaussian"
                        % Sample Gaussian distribution
                        dDisplaceValue = dDisplaceValue + settings.dDisplaceSigma .* randn(1, ui32NumOfSamples);
                    case "uniform"
                        % Compute uniform distribution interval and sample
                        dUniformRange = transpose(0.5 * sqrt( 12 * (settings.dDisplaceSigma).^2 ));
                        dInterval = [-dUniformRange, dUniformRange]; 

                        % Repeat value ui32NumOfSamples times
                        if isscalar(dUniformRange)
                            dInterval = repmat(dInterval, ui32NumOfSamples, 1);
                        end

                        dDisplaceValue = dDisplaceValue + uniformScatter(dInterval(:,1), dInterval(:,2), 1);


                    case "time_correlation"
                        error('Not implemented yet')
                        % Run simulation of FOGM stochastic process dynamics
                        % TODO
                    case "guassian_same_on_batch"
                        assert(isscalar(settings.dDisplaceSigma), 'ERROR: you must provide a scalar displace value for this options.');
                        % Sample 1 value from Gaussian distribution
                        dDisplaceValue = dDisplaceValue + settings.dDisplaceSigma .* randn(1, 1);
                        dDisplaceValue = repmat(dDisplaceValue, 1, ui32NumOfSamples);

                        % Override reference axis by selecting one only
                        dReferenceAxis_Frame = repmat( dReferenceAxis_Frame(1:3, randperm(size(dReferenceAxis_Frame,2), 1)), 1, ui32NumOfSamples);

                    case "uniform_same_on_batch"
                        % Sample 1 value from uniform distribution
                        assert(isscalar(settings.dDisplaceSigma), 'ERROR: you must provide a scalar displace value for this options.');
                        dUniformRange = 0.5 * sqrt( 12 * (settings.dDisplaceSigma).^2 );
                        dInterval = [-dUniformRange, dUniformRange];

                        dDisplaceValue = dDisplaceValue + uniformScatter(dInterval(1), dInterval(2), 1);

                        dDisplaceValue = repmat(dDisplaceValue, 1, ui32NumOfSamples);

                        % Override reference axis by selecting one only
                        dReferenceAxis_Frame = repmat( dReferenceAxis_Frame(1:3, randperm(size(dReferenceAxis_Frame,2), 1)), 1, ui32NumOfSamples);

                    otherwise
                        error('Invalid or not yet implemented case.')
                end

            end

            % Quick return case (do nothing!)
            if all(settings.dDisplaceSigma == 0) && all(settings.dDisplaceValue == 0)
                warning('No displacement or displacement scattering was provided: no displacement applied, returning input values.')
                return
            end


            % Apply displacement using selected displacement mode
            switch settings.enumDisplacementMode
                case "lookAtPoint"
                    [dBoresightUnitVec_Frame(1:3,:), dNewLookAtPoint_Frame(1:3,:)] = CAttitudePointingGenerator.ComputeDisplacedBoresight_LookAtPoint_(dLookAtPoint_Frame, ...
                                                                                                                                    dReferenceAxis_Frame, ...
                                                                                                                                    dDisplaceValue, ...
                                                                                                                                    "bDisplaceOrthogonalToRefAxisPlane", settings.bDisplaceOrthogonalToRefAxisPlane);
                case "rotate3d"
                     [dBoresightUnitVec_Frame(1:3,:)] = CAttitudePointingGenerator.ComputeDisplacedBoresight_Rotate3D_(dLookAtPoint_Frame, ...
                                                                                                                    dReferenceAxis_Frame, ...
                                                                                                                    dDisplaceValue, ...
                                                                                                                    "bDisplaceOrthogonalToRefAxisPlane", settings.bDisplaceOrthogonalToRefAxisPlane);
                otherwise
                    error('Invalid displacement mode.')
            end


        end

        function [dNewLookAtPoint, dLookAtPointUnitVec] = ComputeDisplacedBoresight_LookAtPoint_(dLookAtPoint, dReferenceAxis, dDisplacementNorms, options)
            arguments
                dLookAtPoint        (3,:) double {mustBeNumeric}
                dReferenceAxis      (3,:) double {mustBeNumeric}
                dDisplacementNorms  (1,:) double {mustBeNumeric} 
                options.bDisplaceOrthogonalToRefAxisPlane (1,1) logical {islogical, isscalar} = false
            end
            
            % Compute directions from dLookAtPoint
            dLookAtPointNorms   = vecnorm(dLookAtPoint, 2, 1);
            dLookAtPointUnitVec = dLookAtPoint./dLookAtPointNorms;
            
            ui32NumOfSamples = length(dLookAtPointNorms);
            dDisplaceUnitVec = zeros(3, ui32NumOfSamples);

            if options.bDisplaceOrthogonalToRefAxisPlane
                %% Displace in-plane toward reference axis direction

                % Compute component of dReferenceAxis orthogonal to dLookAtPointUnitVec, in plane
                dDisplaceUnitVec(:,:) = dReferenceAxis - dot(dLookAtPointUnitVec, dReferenceAxis, 1) * dLookAtPointUnitVec; 

            else
                %% Displace orthogonal to plane of reference axis and lookAtPoint

                % Compute component of dReferenceAxis orthogonal to dLookAtPointUnitVec, orthogonal to plane
                dDisplaceUnitVec(:,:) = cross(dReferenceAxis, dLookAtPointUnitVec, 1);

            end
        
            % Normalize to unit vector
            dDisplaceUnitVec = dDisplaceUnitVec./vecnorm(dDisplaceUnitVec, 2, 1);

            % Compute new lookAtPoint in Frame 
            dNewLookAtPoint = dLookAtPoint - (dDisplacementNorms .* dDisplaceUnitVec);

            if nargout > 1
                dLookAtPointUnitVec = dNewLookAtPoint./vecnorm(dNewLookAtPoint, 2, 1);
            end

        end

        function [dBoresightUnitVec_Frame] = ComputeDisplacedBoresight_Rotate3D_(dBoresightVector, dReferenceAxis, dRotDisplaceAngle, options)
            arguments
                dBoresightVector                            (3,:) double {mustBeNumeric}
                dReferenceAxis                              (3,:) double {mustBeNumeric}
                dRotDisplaceAngle                           (1,:) double {mustBeNumeric}
                options.bDisplaceOrthogonalToRefAxisPlane   (1,1) logical {islogical, isscalar} = false
            end

            % Compute directions from dLookAtPoint
            dAuxNorms               = vecnorm(dBoresightVector, 2, 1);
            dBoresightUnitVec_Frame = dBoresightVector./dAuxNorms;

            ui32NumOfSamples = length(dAuxNorms);
            dRotationAxisUnitVec = zeros(3, ui32NumOfSamples);

            if options.bDisplaceOrthogonalToRefAxisPlane
                %% Displace in-plane toward reference axis direction
                % Rotation axis is orthogonal to plane containing reference axis and dBoresightUnitVec_Frame
                dRotationAxisUnitVec(1:3, :) = cross(dReferenceAxis, dBoresightUnitVec_Frame, 1);
                    
            else
                %% Displace orthogonal to plane of reference axis and lookAtPoint
                % Rotation axis is orthogonal component of dReferenceAxis (automatically used by rotation
                % function based on Rodrigues' formula)
                dRotationAxisUnitVec(1:3, :) = dReferenceAxis;
            end

            % Normalize to unit vector
            dRotationAxisUnitVec = dRotationAxisUnitVec./vecnorm(dRotationAxisUnitVec, 2, 1);

            % Rotate boresight vector about dRotationAxisUnitVec
            dBoresightUnitVec_Frame = Rot3dVecAboutDir(dRotationAxisUnitVec, dBoresightUnitVec_Frame, dRotDisplaceAngle);
            

        end

    end

    methods (Access = protected)
        % METHODS
    
   
        % Sanity checks internal functions %TODO
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
