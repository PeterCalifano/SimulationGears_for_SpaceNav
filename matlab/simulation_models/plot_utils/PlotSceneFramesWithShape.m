function [] = PlotSceneFramesWithShape(dSunVector_NavFrame, ...
                                       dCameraOrigin_NavFrame, ...
                                       dCameraAttDCM_NavframeFromOF, ...
                                       dBodiesOrigin_NavFrame, ...
                                       dBodiesAttDCM_NavFrameFromOF, ...
                                       kwargs)
arguments (Input) % TODO: update inputs
    dSunVector_NavFrame              (3,:)   double {isvector, isnumeric}
    dCameraOrigin_NavFrame           (3,:)   double {isvector, isnumeric}
    dCameraAttDCM_NavframeFromOF     (3,3,:) double {ismatrix, isnumeric}
    dBodiesOrigin_NavFrame           (3,:,:)   double {ismatrix, isnumeric} = zeroes(3,1)
    dBodiesAttDCM_NavFrameFromOF     (3,3,:,:) double {ismatrix, isnumeric} = eye(3)
end
arguments (Input)
    kwargs.ui32TargetPort                  (1,1) uint32 {isscalar, isnumeric} = 0
    kwargs.charOutputDatatype              (1,:) string {isa(kwargs.charOutputDatatype, 'string')} = "uint8"
    kwargs.ui32NumOfBodies                 (1,1) uint32 {isnumeric, isscalar} = 1
    kwargs.objCameraIntrinsics             (1,1) {mustBeA(kwargs.objCameraIntrinsics, "CCameraIntrinsics")} = CCameraIntrinsics()
    kwargs.enumRenderingFrame              (1,1) EnumRenderingFrame {isa(kwargs.enumRenderingFrame, 'EnumRenderingFrame')} = EnumRenderingFrame.CUSTOM_FRAME % TARGET_BODY, CAMERA, CUSTOM_FRAME
    kwargs.bEnableFramesPlot               (1,1) logical {islogical} = false;
    kwargs.bConvertCamQuatToBlenderQuat    (1,1) logical {isscalar, islogical} = true;
    kwargs.bDisplayImage                   (1,1) logical {islogical} = false;
end


%% Plot shape object
% TODO


%% Plot objects and camera frames
if kwargs.bEnableFramesPlot
    fprintf("\nProducing requested visualization of scene frames to render...\n")

    % Convert DCMs to quaternion
    dSceneEntityQuatArray_RenderFrameFromOF = transpose( dcm2quat(dBodiesAttDCM_NavFrameFromOF) );
    dCameraQuat_RenderFrameFromCam          = transpose( dcm2quat(dCameraAttDCM_NavframeFromOF) );

    % if kwargs.bConvertCamQuatToBlenderQuat
    % DEVNOTE: removed because plot function operates using the same convention as
    % this function, contrarily to Blender. Assuming that the plot is correct, the
    % downstream operations should be correct too.
    %     dCameraQuat_RenderFrameFromCam = BlenderPyCommManager.convertCamQuatToBlenderQuatStatic(dCameraQuat_RenderFrameFromCam);
    % end

    % Construct figure with plot
    [objSceneFigs(idImg)] = PlotSceneFrames_Quat(dBodiesOrigin_NavFrame, ...
        dSceneEntityQuatArray_RenderFrameFromOF, ...
        dCameraOrigin_NavFrame, ...
        dCameraQuat_RenderFrameFromCam, 'bUseBlackBackground', true, ...
        "charFigTitle", "Visualization with Blender camera quaternion");
end

end

