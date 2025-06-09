classdef CCameraIntrinsics < cameraIntrinsics
%% DESCRIPTION
% Class to store optical cameras intrinsic parameters (calibration) for pinhole projection model
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2024    Pietro Califano     Class implementation as wrapper of cameraIntrinsics
% 09-06-2025    Pietro Califano     Update to add new conveniency methods and attributes
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Implement code to remove dependency from Image Processing toolbox
% -------------------------------------------------------------------------------------------------------------
    
    properties (SetAccess = protected, GetAccess = public)
        bDefaultConstructed logical = true;
        dFovHW              double  = zeros(2,1); % [rad]
        dMeanIFovInRad          double  = zeros(2,1); % [rad]
        % TODO remove inheritance from cameraIntrinsics and add fields here
    end

    properties (SetAccess = public, GetAccess = public)
        ui32NumOfChannels   uint32  = 1;
    end

    methods
        %% CONSTRUCTOR
        function self = CCameraIntrinsics(focalLength_uv, principalPoint_uv, imageSizeHW, ui32NumOfChannels)
            arguments
                focalLength_uv    (1,2) double = ones(1, 2) % Automatically broadcasts to [1,2] if scalar
                principalPoint_uv (1,2) double = ones(1, 2)
                imageSizeHW       (1,2) double = ones(1, 2) 
                ui32NumOfChannels (1,1) uint32 = 1;
            end

            self = self@cameraIntrinsics(focalLength_uv, principalPoint_uv, imageSizeHW);

            if nargin > 2
                % Compute FoV from intrinsics 
                self.dFovHW(1) =  2*atan(0.5 * imageSizeHW(1) / focalLength_uv(1));
                self.dFovHW(2) =  2*atan(0.5 * imageSizeHW(2) / focalLength_uv(2));
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

        function [dFocalLengthInPix] = computeFocalLenghInPix(dFov, dSensorSize, enumInputUnit)
            arguments
                dFov        (:,1) double
                dSensorSize (:,1) double
                enumInputUnit (1,:) string {mustBeMember(enumInputUnit, ["deg", "rad"])} = "rad"
            end
            % TODO: ensure that input is 
            % Compute the focal length (x,y) in pixels from sensor size (x,y) and field of view (x,y)
        
            if strcmpi(enumInputUnit, "deg")
                dFov = deg2rad(dFov);
            end

            dTanHalfFov = tan(0.5*dFov);

            dFocalLengthInPix = zeros(size(dFov, 1), 1);
            dFocalLengthInPix(:) = 0.5 * dSensorSize ./ dTanHalfFov;
               
        end

        function [dFovInDegrees] = computeFovInDegrees(dImageSizeHW, dFocalLengthInPix)
            arguments
                dImageSizeHW      (2,1) double
                dFocalLengthInPix (2,1) double
            end

            dFovInDegrees(1) =  2*atand(0.5 * double(dImageSizeHW(1)) / dFocalLengthInPix(1));
            dFovInDegrees(2) =  2*atand(0.5 * double(dImageSizeHW(2)) / dFocalLengthInPix(2));

        end

        function [dMeanIFovInRad] = computeIFovInRad(dFovInRadHW, dImageSizeHW)
            arguments
                dFovInRadHW       (2,1) double
                dImageSizeHW      (2,1) double
            end

            dMeanIFovInRad = dFovInRadHW ./ dImageSizeHW;
        end


    end
end
