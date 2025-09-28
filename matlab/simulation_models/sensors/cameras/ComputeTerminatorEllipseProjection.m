function [dProjectedEllipsoid_TerminatorPlane, dEllipsoidMatrix_TerminatorPlane, ...
           dTerminatorPlaneUnitVector_Frame, dHomTerminatorEllipse_uv, dNormTerminatorEllipse_uv] = ComputeTerminatorEllipseProjection(dEllipsoidMatrix_TF, ...
                                                                                                    dEllipsoidCentre_Frame, ...
                                                                                                    dSunDirection_Frame, ...
                                                                                                    dDCM_CamFromFrame, ...
                                                                                                    dCameraPosition_Frame, ...
                                                                                                    dDCM_TFfromFrame, ...
                                                                                                    dKcam) %#codegen
arguments
    dEllipsoidMatrix_TF     (3,3) double {ismatrix, isnumeric}
    dEllipsoidCentre_Frame  (3,1) double {isvector, isnumeric}
    dSunDirection_Frame     (3,1) double {isvector, isnumeric}
    dDCM_CamFromFrame       (3,3) double {ismatrix, isnumeric}
    dCameraPosition_Frame   (3,1) double {isvector, isnumeric}
    dDCM_TFfromFrame        (3,3) double {ismatrix, isnumeric} = eye(3);
    dKcam                   (3,3) double {ismatrix, isnumeric} = eye(3);
end
%% SIGNATURE
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 04-03-2025        Pietro Califano         First prototype version implemented.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Construct reference frame of terminator plane (+Z toward the Sun)
dTerminatorPlaneUnitVector_Frame = dSunDirection_Frame - dEllipsoidCentre_Frame;
dTerminatorPlaneUnitVector_Frame = dTerminatorPlaneUnitVector_Frame.norm(dTerminatorPlaneUnitVector_Frame);

dDCM_TerminatorPlaneFromFrame = zeros(3,3); % TODO


% Rotate ellipsoid from TF to Terminator Plane
dDCM_TerminatorPlaneFromTF = dDCM_TerminatorPlaneFromFrame * transpose(dDCM_TFfromFrame);
dEllipsoidMatrix_TerminatorPlane = dDCM_TerminatorPlaneFromTF * dEllipsoidMatrix_TF * transpose(dDCM_TerminatorPlaneFromTF);

% Project 3D Ellipsoid onto Terminator plane (equivalent to zero out Z component)
% NOTE: projection is orthographic in this case. Note that the projection operation has the transposed on
% the left side of the matrix. TBC.
dOrthogonalProjectionMatrix = eye(3) - dTerminatorPlaneUnitVector_Frame * transpose(dTerminatorPlaneUnitVector_Frame);
% NOTE: dProjectedEllipsoid_TerminatorPlane is a 2D quadric in 3D Euclidean space
dProjectedEllipsoid_TerminatorPlane = transpose(dOrthogonalProjectionMatrix) * dEllipsoidMatrix_TerminatorPlane * dOrthogonalProjectionMatrix;

% Expand matrix to projective space P3
dHomProjectedEllipsoid_TerminatorPlane = zeros(4,4);
dHomProjectedEllipsoid_TerminatorPlane(1:3, 1:3) = dProjectedEllipsoid_TerminatorPlane;
dHomProjectedEllipsoid_TerminatorPlane(4,4) = 1.0;

% Project terminator ellipse to camera frame
% DEVNOTE TODO: this step  may be done using homography matrices from terminator plane to camera plane
% directly. Probably better way than the attempt below.

dCameraPose_FromTerminatorPlane = zeros(4,4);
dCameraPose_FromTerminatorPlane(4,4) = 1.0;

dCameraPose_FromTerminatorPlane(1:3, 4) = dDCM_TerminatorPlaneFromFrame * (dCameraPosition_Frame - dEllipsoidCentre_Frame);
dCameraPose_FromTerminatorPlane(1:3, 1:3) = dDCM_CamFromFrame * transpose(dDCM_TerminatorPlaneFromFrame); 

dHomTerminatorEllipse_CAM = dCameraPose_FromTerminatorPlane * dHomProjectedEllipsoid_TerminatorPlane * transpose(dCameraPose_FromTerminatorPlane);

% Project to image plane using camera intrinsic parameters
dHomTerminatorEllipse_uv = dKcam * dHomTerminatorEllipse_CAM * transpose(dKcam);

% Normalize to get 2D ellipse matrix with F = 1
dNormTerminatorEllipse_uv = dHomTerminatorEllipse_uv./dHomTerminatorEllipse_uv(end, end);

end
