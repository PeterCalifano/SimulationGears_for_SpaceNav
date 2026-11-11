classdef CBlenderCommManagerCaptureProbe < BlenderPyCommManager
    %% SIGNATURE
    % objRenderer = CBlenderCommManagerCaptureProbe()
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Real BlenderPyCommManager whose transport is replaced by a capture. renderImage runs the
    % class's own argument validation and scene composition; the composed scene vector is recorded
    % and a fixed image is returned, so no Blender server or socket is needed.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % objRenderer  Unconnected probe with a one-channel camera.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Check the text frame name against the real class.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % BlenderPyCommManager (CommManager4MATLAB, opt-in through SIM_GEARS_COMM_MANAGER_DIR).
    % -------------------------------------------------------------------------------------------------------------

    properties
        dSceneDataVector (1,:) double = zeros(1, 0)
        dImage (:,:) double = (128/255)*ones(4, 5)
        ui32InitializeCount (1,1) uint32 = uint32(0)
    end

    methods
        function self = CBlenderCommManagerCaptureProbe()
            % Build the manager without connecting, with a one-channel camera.
            self@BlenderPyCommManager("127.0.0.1", uint32([30001, 51000]), 1, ...
                'bInitInPlace', false, ...
                'objCameraIntrisincs', CCameraIntrinsics([100, 100], [3, 2.5], [4, 5], uint32(1)));
        end

        function self = Initialize(self)
            % Count connection requests without opening a socket.
            self.ui32InitializeCount = self.ui32InitializeCount + 1;
        end

        function [dImg, self] = renderImageFromPQ_(self, dSceneDataVector, options)
            % Record the composed scene vector and return the fixed image.
            arguments (Input)
                self (1,1) CBlenderCommManagerCaptureProbe
                dSceneDataVector (1,:) double
                options.bApplyBayerFilter (1,1) logical = false %#ok<INUSA>
                options.bIsImageRGB (1,1) logical = false
            end

            self.dSceneDataVector = dSceneDataVector;
            dImg = self.dImage;
        end
    end
end
