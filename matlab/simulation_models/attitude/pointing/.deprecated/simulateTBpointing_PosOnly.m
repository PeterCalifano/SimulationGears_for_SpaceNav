function o_dq_INfromCAM = simulateTBpointing_PosOnly(i_drCam, ...
    i_drTargetBody, i_bIS_VSRPplus, i_bINVERSE_Z_AXIS, i_bIS_QLEFT_HANDED)%#codegen
arguments
    i_drCam           (3,1) double
    i_drTargetBody    (3,1) double
    i_bIS_VSRPplus    (1,1) logical
    i_bINVERSE_Z_AXIS (1,1) logical
    i_bIS_QLEFT_HANDED   (1,1) logical
end
%% PROTOTYPE
% o_dq_fromCAMtonIN = simulateTBpointing_PosOnly(i_drCam, i_drTargetBody, i_bIS_JPL_CONV,
% i_bINVERSE_Z_AXIS, i_bIS_QLEFT_HANDED)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute camera pointing to Target Body assumed at the origin. Orientation of X-Y assuming X to be
% orthogonal to Target body frame Z axis and Camera boresight Z axis.
% NOTE on quaternion convention:
% (SV) Scalar first, Vector last
% (P) Passive 
% (R) Successive coordinate transformations have the unmodified quaternion chain on the Right side of
%     the triple product.
% (plus) Right-Handed Rule for the imaginary numbers i, j, k. (aka Hamilton)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_drCam          
% i_drTargetBody
% i_bIS_JPL_CONV
% i_bINVERSE_Z_AXIS
% i_bIS_QLEFT_HANDED % Required by Blender
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dq_fromCAMonIN
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-03-2024        Pietro Califano         First version coded and verified.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

error('DEPRECATED: use static function in CAttitudePointingGenerator instead.');

assert(all(size(i_drCam) == [3,1],"all"))
assert(all(size(i_drTargetBody) == [3,1],"all"))

% Camera pointing direction (Z axis)
if i_bINVERSE_Z_AXIS
    CamBoresightZ = i_drCam - i_drTargetBody;
else
    CamBoresightZ = i_drTargetBody - i_drCam;
end

CamBoresightZ = CamBoresightZ./norm(CamBoresightZ);

% Camera X axis (Right)
CamDirX = cross([0; 0; 1], CamBoresightZ);
CamDirX = CamDirX./norm(CamDirX);

% Camera Y axis (Left)
CamDirY = cross(CamBoresightZ, CamDirX);
CamDirY = CamDirY./norm(CamDirY);

% Compute attitude rotation matrix
i_dDCM_fromCAMtoIN = [CamDirX, CamDirY, CamBoresightZ];

if i_bIS_QLEFT_HANDED
    i_dDCM_fromCAMtoIN = transpose(i_dDCM_fromCAMtoIN);
end

% Get Quaternion corresponding to the DCM
o_dq_INfromCAM = DCM2quat(i_dDCM_fromCAMtoIN, i_bIS_VSRPplus);

end
