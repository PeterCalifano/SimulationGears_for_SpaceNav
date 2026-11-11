classdef CCameraIntrinsics < cameraIntrinsics
%% DESCRIPTION
% Class to store optical cameras intrinsic parameters (calibration) for pinhole projection model
% Optional detector/radiometric metadata uses native units and does not affect
% projection, feature selection or automatic noise-model configuration.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2024    Pietro Califano     Class implementation as wrapper of cameraIntrinsics
% 09-06-2025    Pietro Califano     Update to add new conveniency methods and attributes
% 16-10-2026    Pietro Califano, Codex gpt-6.1  Retain optional detector metadata.
% 18-10-2026    Pietro Califano, Codex GPT-6  Align FoV with MATLAB [H,W] image order.
% 11-11-2026    Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% cameraIntrinsics, SCameraSensorMetadata.
% -------------------------------------------------------------------------------------------------------------
    properties (SetAccess = protected, GetAccess = public)
        bDefaultConstructed logical     = true;
        dPixelSize          double      = zeros(2,1); % Legacy unpopulated helper [micrometers].
        dFovHW              double      = zeros(2,1); % [vertical, horizontal] rad, matching [H,W].
        dMeanIFovInRad      double      = zeros(2,1); % [rad]
        % TODO remove inheritance from cameraIntrinsics and add fields here
    end

    properties (SetAccess = public, GetAccess = public)
        ui32NumOfChannels   uint32  = 1;
        % Optional physical pitch/response data live here, independently of geometry.
        objSensorMetadata (1,1) SCameraSensorMetadata = SCameraSensorMetadata()
    end

    methods
        %% CONSTRUCTOR
        function self = CCameraIntrinsics(focalLength_uv, principalPoint_uv, imageSizeHW, ui32NumOfChannels)
            % Build the camera from focal lengths [fx,fy], principal point [u,v] and image
            % size [H,W], all in pixels; FoV and mean IFOV need all three.
            arguments (Input)
                focalLength_uv    (1,2) double = ones(1, 2) % Automatically broadcasts to [1,2] if scalar
                principalPoint_uv (1,2) double = ones(1, 2)
                imageSizeHW       (1,2) double = ones(1, 2)
                ui32NumOfChannels (1,1) uint32 = 1;
            end

            self = self@cameraIntrinsics(focalLength_uv, principalPoint_uv, imageSizeHW);

            if nargin > 2
                % Keep FoV in MATLAB [H,W] order while focal length remains [fx,fy].
                self.dFovHW(1) = 2*atan(0.5 * imageSizeHW(1) / focalLength_uv(2));
                self.dFovHW(2) = 2*atan(0.5 * imageSizeHW(2) / focalLength_uv(1));
                self.bDefaultConstructed = false;

                % Compute mean IFOV
                self.dMeanIFovInRad = self.dFovHW ./ imageSizeHW(:);

            elseif nargin > 0 && nargin < 3
                warning('You should specify all the intrinsic parameters for the instance to be a valid one. Make sure to do so before using it.')
            end

            % Assign default number of channels (Grayscale)
            self.ui32NumOfChannels = ui32NumOfChannels;
        end

        %% GETTERS
        function dFovHW = GetFovHW(self)
            self.assertValidity_()
            dFovHW = self.dFovHW;
        end

    end

    methods (Access = protected)
        function assertValidity_(self)
            assert( all( self.dFovHW ~= [0;0]           , 'all'), 'ERROR: Attempt to use default initialized instance (empty)!' )
            assert( all( self.FocalLength ~= [1;1]      , 'all'), 'ERROR: Attempt to use default initialized instance (empty)!' )
            assert( all( self.PrincipalPoint ~= [1;1]   , 'all'), 'ERROR: Attempt to use default initialized instance (empty)!' )
            assert( all( self.ImageSize ~= [1;1]        , 'all'), 'ERROR: Attempt to use default initialized instance (empty)!' )
        end
    end

    methods (Static, Access = public)

        function self = fromCameraIntrinsics(objCameraIntrinsics, ui32NumOfChannels)
            % fromCameraIntrinsics  Create CCameraIntrinsics from cameraIntrinsics
            % Inputs:
            %   objCameraIntrinsics (1,1) cameraIntrinsics
            %   ui32NumOfChannels   (1,1) uint32 = 1
            arguments (Input)
                objCameraIntrinsics (1,1) {mustBeA(objCameraIntrinsics, "cameraIntrinsics")}
                ui32NumOfChannels   (1,1) uint32 = 1
            end

            % Normalize shapes to 1x2 row doubles
            dFocalLength_uv    = reshape(double(objCameraIntrinsics.FocalLength),    1, 2);
            dPrincipalPoint_uv = reshape(double(objCameraIntrinsics.PrincipalPoint), 1, 2);
            dImageSizeHW       = reshape(double(objCameraIntrinsics.ImageSize),      1, 2);

            % Construct derived object (FoV and IFOV computed in ctor)
            self = CCameraIntrinsics(dFocalLength_uv, dPrincipalPoint_uv, dImageSizeHW, ui32NumOfChannels);

            % Copy the sensor metadata from a CCameraIntrinsics source.
            % A plain toolbox camera carries geometry only, so its metadata stays unknown.
            if isa(objCameraIntrinsics, 'CCameraIntrinsics')
                self.objSensorMetadata = objCameraIntrinsics.objSensorMetadata;
            end

            % Best-effort copy of optional calibration fields (if available)
            % Safe if read-only in superclass; errors ignored by try blocks
            if isprop(objCameraIntrinsics, 'RadialDistortion') && isprop(self, 'RadialDistortion')
                try
                    self.RadialDistortion = objCameraIntrinsics.RadialDistortion;
                catch
                end
            end
            if isprop(objCameraIntrinsics, 'TangentialDistortion') && isprop(self, 'TangentialDistortion')
                try
                    self.TangentialDistortion = objCameraIntrinsics.TangentialDistortion;
                catch
                end
            end
            if isprop(objCameraIntrinsics, 'Skew') && isprop(self, 'Skew')
                try
                    self.Skew = objCameraIntrinsics.Skew;
                catch
                end
            end
        end

        function [dFocalLengthInPix] = computeFocalLenghInPix(dFov, dSensorSize, enumInputUnit)
            % Compute the focal length (x,y) in pixels from sensor size (x,y) and field of view (x,y)
            arguments (Input)
                dFov        (:,1) double
                dSensorSize (:,1) double
                enumInputUnit (1,:) string {mustBeMember(enumInputUnit, ["deg", "rad"])} = "rad"
            end

            if strcmpi(enumInputUnit, "deg")
                dFov = deg2rad(dFov);
            end

            dTanHalfFov = tan(0.5*dFov);

            dFocalLengthInPix = zeros(size(dFov, 1), 1);
            dFocalLengthInPix(:) = 0.5 * dSensorSize ./ dTanHalfFov;

        end

        function [dFovInDegrees] = computeFovInDegrees(dImageSizeHW, dFocalLengthInPix)
            % Compute the FoV in degrees from the image size [H,W] and focal lengths [fx,fy] in pixels
            arguments (Input)
                dImageSizeHW      (2,1) double
                dFocalLengthInPix (2,1) double
            end

            % Return [vertical,horizontal] for MATLAB [H,W] and focal lengths [fx,fy].
            dFovInDegrees = zeros(1, 2);
            dFovInDegrees(1) = 2*atand(0.5 * double(dImageSizeHW(1)) / dFocalLengthInPix(2));
            dFovInDegrees(2) = 2*atand(0.5 * double(dImageSizeHW(2)) / dFocalLengthInPix(1));

        end

        function [dMeanIFovInRad] = computeIFovInRad(dFovInRadHW, dImageSizeHW)
            % Compute the mean IFOV per axis in rad/pixel from the FoV [H,W] and the image size [H,W]
            arguments (Input)
                dFovInRadHW       (2,1) double
                dImageSizeHW      (2,1) double
            end

            dMeanIFovInRad = dFovInRadHW ./ dImageSizeHW;
        end

    end
end
