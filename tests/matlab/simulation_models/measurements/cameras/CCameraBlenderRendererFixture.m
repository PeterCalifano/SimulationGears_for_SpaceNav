classdef CCameraBlenderRendererFixture < handle
    %% SIGNATURE
    % objRenderer = CCameraBlenderRendererFixture()
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Deterministic Blender protocol fixture for host-side camera tests.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % objRenderer  Handle fixture that records one configured grayscale acquisition.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 19-10-2026  Pietro Califano, Codex GPT-6  Add host-only render fixture.
    % 19-10-2026  Pietro Califano, Codex GPT-6  Allow typed integer and float render fixtures.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Record the rendering frame as text.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % RenderCameraFrame
    % -------------------------------------------------------------------------------------------------------------

    properties
        dImage (:,:) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative} = ...
            (128/255)*ones(4, 5)
        dSunPosition_W (3,1) double = zeros(3, 1)
        dCameraPosition_W (3,1) double = zeros(3, 1)
        dDCM_WfromCAM (3,3) double = eye(3)
        dTargetPosition_W (3,1) double = zeros(3, 1)
        dDCM_WfromTB (3,3) double = eye(3)
        ui32RenderCount (1,1) uint32 = uint32(0)
        strOptions (1,1) struct = struct()
    end

    methods
        function dImage = renderImage(self, dSunPosition_W, dCameraPosition_W, ...
                dDCM_WfromCAM, dTargetPosition_W, dDCM_WfromTB, options)
            %% SIGNATURE
            % dImage = renderImage(self, dSunPosition_W, dCameraPosition_W, ...)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Record one Blender-style acquisition and return configured fixture pixels.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self, geometry values, and renderer options from RenderCameraFrame.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dImage  Configured numeric grayscale pixels, returned without conversion.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 19-10-2026  Pietro Califano, Codex GPT-6  Record renderer arguments.
            % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) CCameraBlenderRendererFixture
                dSunPosition_W (3,1) double
                dCameraPosition_W (3,1) double
                dDCM_WfromCAM (3,3) double
                dTargetPosition_W (3,1) double
                dDCM_WfromTB (3,3) double
                options.ui32TargetPort (1,1) uint32 = uint32(0)
                options.enumRenderingFrame (1,:) char ...
                    {mustBeMember(options.enumRenderingFrame, {'TARGET_BODY', 'CAMERA', 'CUSTOM_FRAME'})} = 'TARGET_BODY'
                options.bConvertCamQuatToBlenderQuat (1,1) logical = true
            end

            % Record the acquisition geometry and options
            self.dSunPosition_W = dSunPosition_W;
            self.dCameraPosition_W = dCameraPosition_W;
            self.dDCM_WfromCAM = dDCM_WfromCAM;
            self.dTargetPosition_W = dTargetPosition_W;
            self.dDCM_WfromTB = dDCM_WfromTB;
            self.strOptions = struct('ui32TargetPort', options.ui32TargetPort, ...
                'enumRenderingFrame', options.enumRenderingFrame, ...
                'bConvertCamQuatToBlenderQuat', options.bConvertCamQuatToBlenderQuat);

            % Count the render and return the configured pixels
            self.ui32RenderCount = self.ui32RenderCount + 1;
            dImage = self.dImage;
        end
    end
end
