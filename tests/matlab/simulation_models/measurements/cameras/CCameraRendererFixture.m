classdef CCameraRendererFixture < handle
    %% SIGNATURE
    % objRenderer = CCameraRendererFixture()
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Deterministic in-memory Spectra protocol fixture; no renderer dependency.
    % It records the last light position, camera pose and timestamp ID, counts renders
    % and returns a fixed 4-by-5 packed RGBA image of gray level 128.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % objRenderer  Handle fixture implementing the Spectra camera protocol.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header and method comments.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    properties
        dCameraPosition (3,1) double = zeros(3, 1)
        dSunPosition (3,1) double = zeros(3, 1)
        dCameraRotation (3,3) double = eye(3)
        ui32TimestampID (1,1) uint32 = uint32(0)
        ui32RenderCount (1,1) uint32 = uint32(0)
        ui32Image = repmat(uint32(hex2dec('FF808080')), 4, 5)
        strSurfaceLabels (1,1) struct = struct('status', uint32(ones(4, 5)))
        strCaptureInfo (1,1) struct = struct()
    end

    methods
        function setLightPosition(self, dPosition)
            % Record the light position in the target-body frame.
            self.dSunPosition = dPosition;
        end

        function setCameraPose(self, dPosition, dRotation)
            % Record the camera position and attitude in the target-body frame.
            self.dCameraPosition = dPosition;
            self.dCameraRotation = dRotation;
        end

        function renderFrame(self, bSave, ui32TimestampID)
            % Record one acquisition; the fixture never writes images to disk.
            assert(~bSave, 'The owner, not the renderer, persists test images.');
            self.ui32TimestampID = ui32TimestampID;
            self.ui32RenderCount = self.ui32RenderCount + 1;
        end

        function strLabels = renderFrameWithLabels(self, strCaptureInfo, ui32TimestampID)
            %% SIGNATURE
            % strLabels = renderFrameWithLabels(self, strCaptureInfo, ui32TimestampID)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return fixture labels while recording one same-acquisition render.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self, strCaptureInfo, ui32TimestampID  Acquisition label request.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strLabels  Deterministic dense label fixture.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-10-2026  Pietro Califano, Codex GPT-6  Support same-render label output tests.
            % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % renderFrame
            % ---------------------------------------------------------------------------------------------------------
            self.renderFrame(false, ui32TimestampID);
            self.strCaptureInfo = strCaptureInfo;
            strLabels = self.strSurfaceLabels;
        end

        function ui32Image = getImageRGBA(self)
            % Return the configured packed RGBA words unchanged.
            ui32Image = self.ui32Image;
        end
    end
end
