function strHits = QueryCameraSurface(objRequest, objRenderer, objCameraIntrinsics, dPixels_px)
%% SIGNATURE
% strHits = QueryCameraSurface(objRequest, objRenderer, objCameraIntrinsics, dPixels_px)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Intersect MATLAB image coordinates with the target-body geometry used for an acquisition.
% Restore the acquisition camera pose before querying. The caller must retain the same rigid
% scene geometry and externally serialize renderer calls; this function does not retain a scene.
% Return renderer statuses and exact labels unchanged. Positions and ranges are in metres in
% the target-body frame, regardless of the mesh unit declared by objRequest.dLengthUnit_m.
% This host adapter does not store tracks or require prepared-mesh provenance.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objRequest          Immutable acquisition geometry and explicit mesh length unit.
% objRenderer         Spectra queryPixels/setCameraPose provider; mutated synchronously.
% objCameraIntrinsics Modern MATLAB calibration; only an undistorted pinhole is supported.
% dPixels_px          2-by-N coordinates with the first pixel centre at (1,1).
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strHits             Renderer columnar hit record, in input order, including class_id/validity.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2026  Pietro Califano, Codex GPT-6  Query initial feature references outside tracking code.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% spectra_rt.CSpectralRayTracerWrapper.queryPixels; SImageRequest; CCameraIntrinsics.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    objRequest (1,1) SImageRequest
    objRenderer (1,1)
    objCameraIntrinsics (1,1) CCameraIntrinsics
    dPixels_px (2,:) double {mustBeReal}
end
arguments (Output)
    strHits (1,1) struct
end

% Require the pixel-query interface and an undistorted pinhole calibration
if ~ismethod(objRenderer, 'queryPixels') || ~ismethod(objRenderer, 'setCameraPose')
    error('QueryCameraSurface:UnsupportedRenderer', ...
        'Surface references require the Spectra pixel-query interface.');
end

if any(objCameraIntrinsics.RadialDistortion ~= 0) || ...
        any(objCameraIntrinsics.TangentialDistortion ~= 0) || objCameraIntrinsics.Skew ~= 0
    error('QueryCameraSurface:UnsupportedCalibration', ...
        'Surface queries require undistorted, zero-skew pinhole coordinates.');
end

% Match RenderCameraFrame's target-body frame; do not use a later navigation estimate.
dDCM_TBfromW = objRequest.dDCM_WfromTB';
dCameraPosition_TB = dDCM_TBfromW * ...
    (objRequest.dCameraPosition_W - objRequest.dTargetPosition_W);
objRenderer.setCameraPose(dCameraPosition_TB, dDCM_TBfromW * objRequest.dDCM_WfromCAM);

% Describe the acquisition epoch, frame and mesh unit of the query
strQueryInfo = struct('scene_revision', uint64(0), ...
    'epoch_s', objRequest.dTimestamp, ...
    'epoch_reference', 'simulation time', ...
    'world_frame', 'TARGET_BODY', ...
    'world_unit_m', objRequest.dLengthUnit_m);

% Pass K and the pixels in the MATLAB convention; the renderer converts both and checks K against its viewport.
objQuery = objRenderer.queryPixels(strQueryInfo, dPixels_px', objCameraIntrinsics.K, ...
    'matlab_intrinsic');
strHits = objQuery.getData();
end
