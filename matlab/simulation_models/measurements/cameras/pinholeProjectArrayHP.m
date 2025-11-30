function [o_dUVpixCoord, o_dDCM_fromINtoCAM] = pinholeProjectArrayHP(i_dKcam, i_dqCAMwrtIN, i_drSC_IN, i_dPosPoint_IN) %#codegen
%% PROTOTYPE
% [o_dUVpixCoord, o_dDCM_fromINtoCAM] = pinholeProjectArrayHP(i_dKcam, i_dqCAMwrtIN, i_dRSC_IN, i_dPosPoint_IN)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Pinhole projection model (no distorsion) for Line of Sight observables
% (e.g. for optical navigation) using pixel location [u, v] in the image 
% plane as measurements. The camera model is given by the calibration
% matrix Kcam. The attitude of the camera is given by the quaternion
% qCAMwrtIN. The position vector d_RSC_IN from the IN frame origin to the 
% Camera centre in the Inertial frame must be known.
% The position d_PosPoint_IN of the point to project is optional. It can be
% used as a bias term  added to the observed line of sight as [2x1] input 
% providing the displacement vector of the los in the CAM frame (orthogonal 
% to the boresight, i.e. z=0), initialized to zero in MATLAB (pointing at 
% the origin of the IN frame). Datatype of the inputs/outputs are 
% specified by the first letter of the nomenclature. 
% NOTE: quaternion adopted convention: JPL (qv, qs). Hardcoded convention
% flag inside (can be changed).
% REFERENCE:
% 1) Multiple view geometry in computer vision 2nd edition, 
%    Hartley and Zisserman, Eq. 6.7  
% 2) Hera GNC Design Definition and Justification file (not public)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_dKcam:        [3x3] Camera intrinsic (calibration) parameters (Normalized)
% i_dqCAMwrtIN:   [4x1] Attitude quaternion of CAM frame wrt IN frame
% i_dRSC_IN:      [3x1] Position vector of SC in IN frame
% i_dPosPoint_IN: [3xN] Position vector (array) in IN of points to project
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dUVpixCoord:      [2xN] Pixel coordinates of projected points in image plane
% o_dDCM_fromINtoCAM: [3x3] Rotation matrix from IN to CAM computed from
%                          input quaternion
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-09-2023    Pietro Califano     Coded, not validated.
% 13-09-2023    Pietro Califano     Modification to compute the DCM once,
%                                   given as second output.
% 15-09-2023    Pietro Califano     Validated against Hera GNC filter models.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Add validity check for UV pixels inside detector 
% -------------------------------------------------------------------------------------------------------------


%% Function code
% Get size of input (how many points to project)
Npoints = uint16(size(i_dPosPoint_IN, 2));

% Uninitialized memory allocation for codegen
o_dUVpixCoord = coder.nullcopy(zeros(2, Npoints));
d_PosPointTmp_IN = coder.nullcopy(zeros(3, 1));

% Convert attitude from quaternion to DCM
o_dDCM_fromINtoCAM = transpose(Quat2DCM(i_dqCAMwrtIN, true));

% Compute pixel coordinates
for idP = 1:Npoints

    % Extract position vector of point to project
    d_PosPointTmp_IN(1:3) = i_dPosPoint_IN(1:3, idP);
    % Project vector
    o_dUVpixCoord(1:2, idP) = pinholeProjectPoint(i_dKcam, o_dDCM_fromINtoCAM, i_drSC_IN, d_PosPointTmp_IN);

end

%% LOCAL FUNCTIONS
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
    %% Quaternion to DCM conversion
    function DCM = Quat2DCM(dQuatRot, bIS_JPL_CONV) %#codegen

        % DCM initialization
        DCM = zeros(3,3);

        % Get quaternion components depending on quaternion order
        if bIS_JPL_CONV == false
            % Hamilton convention (false)
            qs  = dQuatRot(1);
            qv1 = dQuatRot(2);
            qv2 = dQuatRot(3);
            qv3 = dQuatRot(4);
        else
            % JPL convention (true)
            qv1 = dQuatRot(1);
            qv2 = dQuatRot(2);
            qv3 = dQuatRot(3);
            qs  = dQuatRot(4);
        end

        % Convert to DCM (Not numerically optimized according to input Quat)
        DCM(1,1) = qs^2 + qv1^2 - qv2^2 - qv3^2;
        DCM(2,1) = 2*(qv1*qv2 - qv3*qs);
        DCM(3,1) = 2*(qv1*qv3 + qv2*qs);

        DCM(1,2) = 2*(qv1*qv2 + qv3*qs);
        DCM(2,2) = qs^2 - qv1^2 + qv2^2 - qv3^2;
        DCM(3,2) = 2*(qv2*qv3 - qv1*qs);

        DCM(1,3) = 2*(qv1*qv3 - qv2*qs);
        DCM(2,3) = 2*(qv2*qv3 + qv1*qs);
        DCM(3,3) = qs^2 - qv1^2 - qv2^2 + qv3^2;

    end
end
