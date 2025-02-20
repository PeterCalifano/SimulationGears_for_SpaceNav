function [objSceneFig, cellPlotObjs] = PlotSceneFramesWithShape(strShapeModel, ...
                                                                dSunPosition_NavFrame, ...
                                                                dCameraOrigin_NavFrame, ...
                                                                dCameraAttDCM_NavframeFromOF, ...
                                                                dBodyOrigin_NavFrame, ...
                                                                dBodyAttDCM_NavFrameFromOF, ...
                                                                kwargs)
arguments (Input) 
    strShapeModel                  {isstruct}
    dSunPosition_NavFrame          (3,1)   double {isnumeric}
    dCameraOrigin_NavFrame         (3,1)   double {isvector, isnumeric}
    dCameraAttDCM_NavframeFromOF   (3,3)   double {ismatrix, isnumeric}
    dBodyOrigin_NavFrame           (3,:)   double {ismatrix, isnumeric} = zeroes(3,1)
    dBodyAttDCM_NavFrameFromOF     (3,3,:) double {ismatrix, isnumeric} = eye(3)
end
arguments (Input)
    kwargs.bUseBlackBackground          (1,1) logical {islogical, isscalar} = false;
    kwargs.charDistanceUnit             (1,:) string {mustBeMember(kwargs.charDistanceUnit, ["m", "km"])} = 'm'
    kwargs.cellPlotColors               (1,:) cell = {};
    kwargs.cellPlotNames                (1,:) cell = {};
    kwargs.objSceneFig                  (1,1) {isscalar, mustBeA(kwargs.objSceneFig, ["double", "matlab.ui.Figure"])} = 0;
    kwargs.charFigTitle                 (1,:) string {mustBeA(kwargs.charFigTitle, ["string", "char"])} = "Scene frames and shape visualization"
    kwargs.bUsePhysicalPosition         (1,1) logical {islogical, isscalar} = false;
    kwargs.dPointsPositions_NavFrame    (3,:) double = [];
    kwargs.bConvertToBlenderFrame       (1,1) logical {islogical, isscalar} = false;
end


% Get figure and properties
if kwargs.objSceneFig == 0
    objSceneFig = figure('Renderer', 'opengl');
    kwargs.bEnforcePlotOpts = true; % No figure provided, enable plot opts
    [~, charTextColor, ~] = DefaultPlotOpts(objSceneFig, "charRenderer", "opengl", "bUseBlackBackground", kwargs.bUseBlackBackground);
else
    objSceneFig = kwargs.objSceneFig;
    charTextColor = objSceneFig.Color;
end

% Check that strShapeModel contains necessary fields
if ~isfield(strShapeModel, 'dVerticesPos') || ~isfield(strShapeModel, 'ui32triangVertexPtr')
    error('strShapeModel must contain fields: dVerticesPos and ui32triangVertexPtr');
end

% Get scale from maximum radius of shape model
dAxisScale = 1.25 * max(vecnorm(strShapeModel.dVerticesPos, 2, 1), [], 'all');


%% Plot shape object with point cloud
[objSceneFig, cellPlotObjs] = Visualize3dShapeModelWithPC(strShapeModel, ...
                                                          dCameraOrigin_NavFrame, ...
                                                          dSunPosition_NavFrame, ...
                                                          dBodyAttDCM_NavFrameFromOF, ...
                                                          "charDistanceUnit", kwargs.charDistanceUnit, ...
                                                          "bUseBlackBackground", kwargs.bUseBlackBackground, ...
                                                          "objFig", objSceneFig, ...
                                                          "bEnableLegend", false, ...
                                                          "dPointsPositions_NavFrame", kwargs.dPointsPositions_NavFrame);


%% Plot objects and camera frames
% Convert DCMs to quaternion
dSceneEntityQuatArray_RenderFrameFromOF = transpose( dcm2quat(dBodyAttDCM_NavFrameFromOF) );

if kwargs.bConvertToBlenderFrame
    % Perform operations to convert to Blender quaternion
    dCameraQuat_RenderFrameFromCam  = transpose( dcm2quat( transpose(dCameraAttDCM_NavframeFromOF )) );
    dCameraQuat_RenderFrameFromCam  = transpose( quatinv( quatmultiply( dCameraQuat_RenderFrameFromCam' , [0,1,0,0]) ) );
    % (Blender takes this quaternion inverted for it to be correctly pointing to target)

else
    % Use matrix with standard camera frame convention
    dCameraQuat_RenderFrameFromCam          = transpose( dcm2quat(dCameraAttDCM_NavframeFromOF) );
end


% Construct figure with plot
[~, cellPlotObjs_SceneFrames] = PlotSceneFrames_Quat(dBodyOrigin_NavFrame, ...
                                                    dSceneEntityQuatArray_RenderFrameFromOF, ...
                                                    dCameraOrigin_NavFrame, ...
                                                    dCameraQuat_RenderFrameFromCam, ...
                                                    'bUseBlackBackground', true, ...
                                                    "objFig", objSceneFig, ...
                                                    "charFigTitle", kwargs.charFigTitle, ...
                                                    "dAxisScale", dAxisScale, ...
                                                    "bEnableLegend", false, ...
                                                    "bUsePhysicalPosition", kwargs.bUsePhysicalPosition, ...
                                                    "cellPlotColors", kwargs.cellPlotColors);



%% Handle legend
% Define cell for global legend
cellPlotObjs = [cellPlotObjs(:)', cellPlotObjs_SceneFrames(:)'];

% Add legend
legend([cellPlotObjs{:}], 'TextColor', charTextColor)
axis equal
grid off


end

