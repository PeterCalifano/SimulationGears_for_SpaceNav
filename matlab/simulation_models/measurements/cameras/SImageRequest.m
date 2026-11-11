classdef SImageRequest
    %% SIGNATURE
    % self = SImageRequest(ui32TimestampID, i32FrameID, dTimestamp, ...
    %     dSunPosition_W, dCameraPosition_W, dDCM_WfromCAM, ...
    %     dTargetPosition_W, dDCM_WfromTB)
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Immutable camera acquisition geometry. All positions share one world frame
    % and length unit, matching the injected renderer's mesh unit. Declare its
    % metre scale with dLengthUnit_m when requesting surface queries. Rotations are
    % active camera/body-to-world DCMs. The renderer adapter performs conversion
    % to its target-body frame; no frontend, navigation, or truth object is stored.
    % Host-side contract, not a deployable numerical kernel.
    % Retain supplied DCMs and check rotation residuals with single-precision epsilon in double arithmetic.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % Timestamp/frame IDs identify acquisition; dTimestamp is finite seconds.
    % Positions are finite 3-vectors; DCMs satisfy proper-rotation checks within single-precision epsilon.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % self  Validated request, independent of Nav-Frontend image observations.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 12-10-2026  Pietro Califano, Codex gpt-6.1  Define the owned image boundary.
    % 18-10-2026  Pietro Califano, Codex GPT-6  Declare acquisition position units for surface queries.
    % 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Use single-precision epsilon for rotation checks.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % MATLAB only. See RunCameraFrameExample for representative use/output.
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = immutable)
        ui32TimestampID (1,1) uint32
        i32FrameID (1,1) int32
        dTimestamp (1,1) double
        dSunPosition_W (3,1) double
        dCameraPosition_W (3,1) double
        dDCM_WfromCAM (3,3) double
        dTargetPosition_W (3,1) double
        dDCM_WfromTB (3,3) double
        objImageErrorSettings (1,1) SImageErrorsPipelineSettings
        dLengthUnit_m (1,1) double
    end

    methods
        function self = SImageRequest(ui32TimestampID, i32FrameID, dTimestamp, ...
                dSunPosition_W, dCameraPosition_W, dDCM_WfromCAM, ...
                dTargetPosition_W, dDCM_WfromTB, options)
            % Validate world geometry once before any stateful renderer call.
            % Retain supplied DCMs; evaluate rotation residuals in double precision.
            arguments (Input)
                ui32TimestampID (1,1) uint32
                i32FrameID (1,1) int32
                dTimestamp (1,1) double {mustBeFinite}
                dSunPosition_W (3,1) double {mustBeFinite, mustBeReal}
                dCameraPosition_W (3,1) double {mustBeFinite, mustBeReal}
                dDCM_WfromCAM (3,3) double {mustBeFinite, mustBeReal}
                dTargetPosition_W (3,1) double {mustBeFinite, mustBeReal}
                dDCM_WfromTB (3,3) double {mustBeFinite, mustBeReal}
                options.objImageErrorSettings (1,1) SImageErrorsPipelineSettings = SImageErrorsPipelineSettings()
                options.dLengthUnit_m (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1
            end

            % Bound rotation residuals without casting or projecting the supplied DCMs.
            % Each loop iteration takes one DCM as a 9-by-1 column and reshapes it to 3-by-3.
            dRotationTolerance = double(eps('single'));
            for dRotationColumn = reshape([dDCM_WfromCAM(:), dDCM_WfromTB(:)], 9, 2)
                dRotation = reshape(dRotationColumn, 3, 3);
                if norm(dRotation' * dRotation - eye(3), 'fro') > dRotationTolerance || ...
                        abs(det(dRotation) - 1) > dRotationTolerance
                    error('SImageRequest:InvalidRotation', ...
                        'Camera and target DCMs must satisfy proper-rotation checks within single-precision epsilon.');
                end
            end

            % Reject a camera or Sun placed at the target origin
            if norm(dCameraPosition_W - dTargetPosition_W) <= eps || ...
                    norm(dSunPosition_W - dTargetPosition_W) <= eps
                error('SImageRequest:DegenerateGeometry', ...
                    'Camera and Sun positions must differ from the target origin.');
            end

            % Store the validated acquisition
            self.ui32TimestampID = ui32TimestampID;
            self.i32FrameID = i32FrameID;
            self.dTimestamp = dTimestamp;
            self.dSunPosition_W = dSunPosition_W;
            self.dCameraPosition_W = dCameraPosition_W;
            self.dDCM_WfromCAM = dDCM_WfromCAM;
            self.dTargetPosition_W = dTargetPosition_W;
            self.dDCM_WfromTB = dDCM_WfromTB;
            self.objImageErrorSettings = options.objImageErrorSettings;
            self.dLengthUnit_m = options.dLengthUnit_m;
        end
    end
end
