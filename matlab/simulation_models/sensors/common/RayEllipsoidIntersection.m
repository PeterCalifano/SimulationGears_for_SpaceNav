function [bIntersectFlag, dIntersectDistance, bFailureFlag, dIntersectPoint] = RayEllipsoidIntersection(dRayOrigin, ...
                                                                                                         dRayDirection, ...
                                                                                                         dEllipsoidCentre, ...
                                                                                                         dEllipsoidInvDiagShapeCoeffs, ...
                                                                                                         dDCM_TFfromFrame) %#codegen
arguments
    dRayOrigin                      (3,1) double
    dRayDirection                   (3,1) double
    dEllipsoidCentre                (3,1) double
    dEllipsoidInvDiagShapeCoeffs    (:,1) double % [1/a^2; 1/b^2; 1/c^2]
    dDCM_TFfromFrame                (3,3) double = eye(3)
end
%% SIGNATURE
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to compute intersection of a 3D ellipsoid with a ray (line in 3D).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRayOrigin                      (3,1) double
% dRayDirection                   (3,1) double
% dEllipsoidCentre                (3,1) double
% dEllipsoidInvDiagShapeCoeffs    (:,1) double % [1/a^2; 1/b^2; 1/c^2]
% dDCM_TFfromFrame                (3,3) double = eye(3)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-02-2025        Pietro Califano         First version implemented.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code
% Data pre-processing
if not(all(dDCM_TFfromFrame == eye(3)))
    % Convert ray origin, direction and target position to target fixed frame
    dRayOrigin          = dDCM_TFfromFrame * dRayOrigin;
    dRayDirection       = dDCM_TFfromFrame * dRayDirection;
    if any(abs(dEllipsoidCentre) > 0)
        dEllipsoidCentre    = dDCM_TFfromFrame * dEllipsoidCentre;
    end
end

% Form ellipsoid shape matrix
dEllipsoidMatrix = diag(dEllipsoidInvDiagShapeCoeffs); % TODO check this is correct

% Initialize output
bIntersectFlag      = false;
bFailureFlag        = false;
dIntersectPoint     = zeros(3, 1);
dIntersectDistance  = zeros(1, 1);

% Compute 2nd order intersection equation
% Equation: at^2 + 2bt + c = 0. Note that the b computed here is twice the B coefficient of a generic 
% quadratic equation. This is why the Delta and the solution looks slightly strange.

dRayOriginFromEllipsCentre = dRayOrigin - dEllipsoidCentre;
dAuxMatrix0 = dRayDirection' * dEllipsoidMatrix;

% Compute a coefficient 
% NOTE: this can be avoided in case of a sphere and set to 1, replacing the 1 with r^2 in C)
daCoeff = dAuxMatrix0 * dRayDirection;
% Compute b coefficient
dbCoeff = dAuxMatrix0 * dRayOriginFromEllipsCentre;
% Compute c coefficient
dcCoeff = dRayOriginFromEllipsCentre' * dEllipsoidMatrix * dRayOriginFromEllipsCentre - 1;

% Intersection equation discriminant
dDelta = dbCoeff^2 - daCoeff*dcCoeff;

% Evaluate intersection test
if dDelta < 0
    return
end

bIntersectFlag = true;
dAuxTerm0 = 1 / daCoeff; 
dAuxTerm1 = sqrt(dDelta); 

dtParam0 = dAuxTerm0 * ( - dbCoeff + dAuxTerm1 );
dtParam1 = dAuxTerm0 * ( - dbCoeff - dAuxTerm1 ); 

if dtParam0 >= eps && dtParam1 >= eps
    dIntersectDistance(:) = min(dtParam0, dtParam1);
else
    bIntersectFlag = false;
    bFailureFlag = true;
    return % Test failure
end

% Compute intersection point from ray equation if required
dIntersectPoint(:) = dRayOrigin + dRayDirection * dIntersectDistance;

end
