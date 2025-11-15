function [dUVpixCoord] = pinholeProjectArrayHP_DCM(dKcam, dDCM_CAMfromIN, dCameraPos_IN, dPointPos_IN) %#codegen
arguments
    dKcam           (3,3) double {ismatrix}
    dDCM_CAMfromIN  (3,3) double {ismatrix}
    dCameraPos_IN   (3,1) double {isvector}
    dPointPos_IN    (3,:) double {isnumeric}
end
%% PROTOTYPE
% [dUVpixCoord] = pinholeProjectArrayHP_DCM(dKcam, dDCM_CAMfromIN, dCameraPos_IN, dPointPos_IN)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Pinhole projection model (no distorsion) for Line of Sight observables
% (e.g. for optical navigation) using pixel location [u, v] in the image
% plane as measurements. The camera model is given by the calibration
% matrix Kcam. The attitude of the camera is given by the attitude matrix
% dDCM_CAMfromIN. The position vector d_RSC_IN from the IN frame origin to the
% Camera centre in the Inertial frame must be known.
% The position d_PosPoint_IN of the point to project is optional. It can be
% used as a bias term  added to the observed line of sight as [2x1] input
% providing the displacement vector of the los in the CAM frame (orthogonal
% to the boresight, i.e. z=0), initialized to zero in MATLAB (pointing at
% the origin of the IN frame). Datatype of the inputs/outputs are
% specified by the first letter of the nomenclature.
% REFERENCE:
% 1) Multiple view geometry in computer vision 2nd edition,
%    Hartley and Zisserman, Eq. 6.7
% 2) Hera GNC Design Definition and Justification file (not public)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dKcam           (3,3) double {ismatrix}   Camera intrinsic (calibration) parameters (Normalized)
% dDCM_CAMfromIN  (3,3) double {ismatrix}   Rotation matrix from IN to CAMinput quaternion
% dCameraPos_IN   (3,1) double {isvector}   Position vector of SC in IN frame
% dPointPos_IN    (3,:) double {isnumeric}  Position vector (array) in IN of points to project
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dUVpixCoord:      [2xN] Pixel coordinates of projected points in image plane
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-11-2024    Pietro Califano     Improved parallelized version from pinholeProjectArrayHP legacy code
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Add validity check for UV pixels inside detector
% -------------------------------------------------------------------------------------------------------------


%% Function code
% Get size of input (how many points to project)
i32Npoints = int32(size(dPointPos_IN, 2));

% Uninitialized memory allocation for codegen
dUpixCoord = coder.nullcopy(zeros(1, i32Npoints, 'double'));
dVpixCoord = coder.nullcopy(zeros(1, i32Npoints, 'double'));

dTmpXpoint_IN = dPointPos_IN(1,:);
dTmpYpoint_IN = dPointPos_IN(2,:);
dTmpZpoint_IN = dPointPos_IN(3,:);

% Compute pixel coordinates in parallel
parfor idP = 1:i32Npoints

    % Project vector
    tmpUVpixCoord = pinholeProjectPoint(dKcam, dDCM_CAMfromIN, dCameraPos_IN, ...
        [dTmpXpoint_IN(idP); ...
        dTmpYpoint_IN(idP); ...
        dTmpZpoint_IN(idP)]);

    dUpixCoord(idP) = tmpUVpixCoord(1);
    dVpixCoord(idP) = tmpUVpixCoord(2);

end

dUVpixCoord = [dUpixCoord; dVpixCoord];

end

%% Single point projection
function [dUVpixCoord, dPosPix] = pinholeProjectPoint(dKcam, dDCM_fromINtoCAM, dRSC_IN, dPosPoint_IN) %#codegen
% Pinhole projection model (no distorsion) for Line of Sight observables
% (e.g. for optical navigation) using pixel location [u, v] in the image
% plane as measurements. SINGLE POINT ONLY.

% Uninitialized memory allocation for codegen
dPosPix     = coder.nullcopy(zeros(3, 1));

% Compute "position vector" of pixel in CAM frame
% adCameraMatrix = d_Kcam * d_DCM_fromINtoCAM * [eye(3), -d_RSC_IN];
dPosPix(:) = (dKcam * dDCM_fromINtoCAM * [eye(3), -dRSC_IN]) * [dPosPoint_IN; 1];

% Compute pixel coordinates in image plane (normalization)
dUVpixCoord = [dPosPix(1)/dPosPix(3); dPosPix(2)/dPosPix(3)];

end
