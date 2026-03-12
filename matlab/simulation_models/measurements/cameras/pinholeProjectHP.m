function [dPixCoord, dPosPix] = pinholeProjectHP(dKcam, ...
                                                dDCM_CAMfromIN, ...
                                                dCameraPos_IN, ...
                                                dPointPos_IN) %#codegen
arguments
    dKcam           (3,3) double {ismatrix}
    dDCM_CAMfromIN  (3,3) double {ismatrix}
    dCameraPos_IN   (3,1) double {isvector}
    dPointPos_IN    (3,:) double {isnumeric}
end
%% PROTOTYPE
% [dPixCoord, dPosPix] = pinholeProjectHP(dKcam, dDCM_CAMfromIN, drCam_IN, dPosPoint_IN) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Pinhole projection model (no distorsion) for Line of Sight observables
% (e.g. for optical navigation) using pixel location [u, v] in the image
% plane as measurements. SINGLE POINT ONLY.
% NOTE for use: dPosPoint_IN may be employed to add bias term to the 
% observed line of sight as [2x1] input providing the displacement vector 
% of the los in the CAM frame (orthogonal to the boresight, i.e. z=0).
% REFERENCE:
% 1) Multiple view geometry in computer vision 2nd edition, 
%    Hartley and Zisserman, Eq. 6.7  
% 2) Hera GNC Design Definition and Justification file (not public)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dKcam           (3,3) double {ismatrix}   Camera intrinsic (calibration) parameters (Normalized)
% dDCM_CAMfromIN  (3,3) double {ismatrix}   Attitude matrix converting from Inertial frame to Camera frame
% dCameraPos_IN   (3,1) double {isvector}   Position vector of Cam origin in IN frame
% dPointPos_IN    (3,:) double {isnumeric}   Position vector of point to project in IN frame
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPixCoord: [2xN]  Pixel coordinates of projected points in image plane
% dPosPix:   [3x1]  Projected point (u,v,w) in CAM frame 
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-09-2023    Pietro Califano     Coded, not validated.
% 13-09-2023    Pietro Califano     Modification to compute the DCM once,
%                                   given as second output.
% 15-09-2023    Pietro Califano     Validated against Hera GNC filter models.
% 10-01-2025    Pietro Califano     Update of naming and variables convention.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------

% Uninitialized memory allocation for codegen
dPosPix     = coder.nullcopy(zeros(3, 1));

% Compute "position vector" of pixel in CAM frame
% adCameraMatrix = d_Kcam * d_DCM_fromINtoCAM * [eye(3), -d_RSC_IN];
dPosPix(1:3) = (dKcam * dDCM_CAMfromIN * [eye(3), -dCameraPos_IN]) * [dPointPos_IN; 1];

% Compute pixel coordinates in image plane (normalization)
dPixCoord = [dPosPix(1)/dPosPix(3); dPosPix(2)/dPosPix(3)];

end
