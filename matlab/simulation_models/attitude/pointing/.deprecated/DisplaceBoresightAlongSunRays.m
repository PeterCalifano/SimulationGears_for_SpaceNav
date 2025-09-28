function [dCameraPositionToPoint_Frame, dCamBoresight_Frame] = DisplaceBoresightAlongSunRays(dCameraPosition_Frame, ...
                                                                                            dSunPosition_Frame, ...
                                                                                            dReferenceDistance, ...
                                                                                            settings)
arguments
    dCameraPosition_Frame (3,1) double {isvector, isnumeric}   
    dSunPosition_Frame    (3,1) double {isvector, isnumeric}  
    dReferenceDistance    (1,1) double {isscalar, isnumeric} 
end
arguments
    settings.dScaleSigma (1,1) double {isscalar, mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
end
%% SIGNATURE
% [dCameraPositionToPoint_Frame, dCamBoresight_Frame] = DisplaceBoresightAlongSunRays(dCameraPosition_Frame, ...
%     dSunPosition_Frame, dReferenceDistance, settings)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dCameraPosition_Frame (3,1) double {isvector, isnumeric}
% dSunPosition_Frame    (3,1) double {isvector, isnumeric}
% dReferenceDistance    (1,1) double {isscalar, isnumeric}
% settings.dScaleSigma  (1,1) double {isscalar, mustBeGreaterThan(settings.dScaleSigma, 0)} = 0;
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dCameraPositionToPoint_Frame
% dCamBoresight_Frame
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-12-2024        Pietro Califano         Implementation from previously validated code
% 30-01-2025        Pietro Califano         Modify with new strategy for off-pointing and minor changes
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

error('DEPRECATED: use static function in CAttitudePointingGenerator instead.');
dDeltaDir = zeros(3,1);

% Compute direction delta unit vector
% TODO: verify whether the commented code is functionally equivalent
% dDeltaDir(1:3) = dot(dCameraPosition_Frame, dCameraPosition_Frame) * dSunPosition_Frame ...
%                 - dot(dCameraPosition_Frame, dSunPosition_Frame) * dCameraPosition_Frame; 
% 
% dDeltaDir(1:3) = dDeltaDir./norm(dDeltaDir); % Displacement along a perpendicular to position vector in the plane defined by this and the sun directions
                
% Direction orthogonal to camera position and Sun position plane
dAuxDir1 = cross(dSunPosition_Frame(idP, :)', -dCameraPosition_Frame(idP, :)');
dAuxDir1 = dAuxDir1./norm(dAuxDir1);

% Direction orthogonal to plane formed by camera position and AuxDir1, toward the Sun
dDeltaDir(:) = cross(-dCameraPosition_Frame(idP, :)', dAuxDir1);
dDeltaDir(:) = dDeltaDir./norm(dDeltaDir);

% Set scale of change
if settings.dScaleSigma > 0
    dScaleScatterValue = randn(0, settings.dScaleSigma);
else
    dScaleScatterValue = 0;
end

% Compute displacement scale and add random coefficient to randomize pointing "across" the limb 
dScaleModulus = dReferenceDistance + dScaleScatterValue; 

% Compute new camera position in Frame from look_at point
dCameraPositionToPoint_Frame = dCameraPosition_Frame - (dScaleModulus * dDeltaDir);

if nargout > 1
    dCamBoresight_Frame = dCameraPositionToPoint_Frame./vecnorm(dCameraPositionToPoint_Frame, 2, 1);
end


end
