function [bIntersectFlag, dIntersectDistance, bFailureFlag, dIntersectPoint, ...
    dJacIntersectDistance_RayOrigin, dJacIntersectDistance_TargetAttErr] = ...
    RayEllipsoidIntersection(dRayOrigin_Frame, dRayDirection_Frame, dEllipsoidCentre_Frame, ...
                             dEllipsoidInvDiagShapeCoeffs, dDCM_TFfromFrame, ...
                             dDCM_EstTFfromFrame, bEvaluateJacobians) %#codegen
%% SIGNATURE
% [bIntersectFlag, dIntersectDistance, bFailureFlag, dIntersectPoint, ...
%     dJacIntersectDistance_RayOrigin, dJacIntersectDistance_TargetAttErr] = ...
%     RayEllipsoidIntersection(dRayOrigin_Frame, dRayDirection_Frame, dEllipsoidCentre_Frame, ...
%                              dEllipsoidInvDiagShapeCoeffs, dDCM_TFfromFrame, ...
%                              dDCM_EstTFfromFrame, bEvaluateJacobians)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Return the nearest positive intersection of a unit ray with an ellipsoid.
% Transform the ray and centre with dDCM_EstTFfromFrame before evaluating the
% axis-aligned shape. Reject near-tangent intersections using the discriminant.
% Both Jacobians are evaluated at this same estimated rotation. The attitude
% derivative uses R(delta) = Exp(skew(delta)) * R_EstTFfromFrame, with delta in
% local TF axes [rad]. A passive rotation-vector bias needs a separate chain rule.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRayOrigin_Frame              Ray origin in the input frame [length].
% dRayDirection_Frame           Unit ray direction in the input frame.
% dEllipsoidCentre_Frame        Ellipsoid centre in the input frame [length].
% dEllipsoidInvDiagShapeCoeffs   Inverse squared semiaxes [1/a^2; 1/b^2; 1/c^2].
% dDCM_TFfromFrame              Default rotation if the estimated one is omitted.
% dDCM_EstTFfromFrame           Rotation into ellipsoid principal-axis coordinates.
% bEvaluateJacobians            Independent [origin, attitude] derivative flags.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bIntersectFlag                       True for an accepted forward intersection.
% dIntersectDistance                   Distance along the ray [length]; zero on miss.
% bFailureFlag                         True for invalid or near-tangent geometry.
% dIntersectPoint                      Intersection point in TF coordinates [length].
% dJacIntersectDistance_RayOrigin       Distance derivative w.r.t. input-frame origin.
% dJacIntersectDistance_TargetAttErr    Distance derivative w.r.t. local TF rotation
%                                      [length/rad]. Unrequested derivatives are zero.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-03-2025        Pietro Califano         First version of intersection test implemented.
% 04-03-2025        Pietro Califano         Implement jacobian evaluation wrt ray origin and target attitude.
% 14-05-2025        Pietro Califano         Add flag to require/skip evaluation of jacobians.
% 30-11-2025        Pietro Califano         Improve checks for numerical robustness; debug of jacobians
% 09-09-2026  Pietro Califano, Codex gpt-6    Differentiate the surface at the estimated intersection.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% skewSymm.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dRayOrigin_Frame           (3,1) double {mustBeNumeric}
    dRayDirection_Frame        (3,1) double {mustBeNumeric}
    dEllipsoidCentre_Frame     (3,1) double {mustBeNumeric}
    dEllipsoidInvDiagShapeCoeffs (3,1) double {mustBeNumeric, mustBeFinite, mustBePositive}
    dDCM_TFfromFrame           (3,3) double {mustBeNumeric} = eye(3)
    dDCM_EstTFfromFrame        (3,3) double {mustBeNumeric} = dDCM_TFfromFrame
    bEvaluateJacobians         (1,2) logical = [true, true]
end
arguments (Output)
    bIntersectFlag                     (1,1) logical
    dIntersectDistance                 (1,1) double
    bFailureFlag                       (1,1) logical
    dIntersectPoint                    (3,1) double
    dJacIntersectDistance_RayOrigin    (1,3) double
    dJacIntersectDistance_TargetAttErr (1,3) double
end

%% Function code
if not(all(dDCM_EstTFfromFrame == eye(3), 'all'))
    % Convert IN-PLACE ray origin, direction and target position to target fixed frame
    dRayOrigin_Frame          = dDCM_EstTFfromFrame * dRayOrigin_Frame;
    dRayDirection_Frame       = dDCM_EstTFfromFrame * dRayDirection_Frame;
    
    if any(abs(dEllipsoidCentre_Frame) > 0)
        dEllipsoidCentre_Frame    = dDCM_EstTFfromFrame * dEllipsoidCentre_Frame;
    end
end

% NOTE: the ellipsoid is assumed diagonal in its principal-axis frame.
dEllipsoidMatrix = diag(dEllipsoidInvDiagShapeCoeffs);

% Initialize output
bIntersectFlag                      = false;
bFailureFlag                        = false;
dIntersectPoint                     = zeros(3, 1);
dIntersectDistance                  = zeros(1, 1);
dJacIntersectDistance_RayOrigin     = zeros(1, 3);
dJacIntersectDistance_TargetAttErr  = zeros(1, 3);

% Form the quadratic a*t^2 + 2*b*t + c = 0 for distance along the ray.
dRayOriginFromEllipsCentre = dRayOrigin_Frame - dEllipsoidCentre_Frame; % In Target fixed
dAuxMatrix0 = dRayDirection_Frame' * dEllipsoidMatrix;
dDirectionNorm = norm(dRayDirection_Frame);

% Guard against degenerate direction or singular ellipsoid parameters
if dDirectionNorm < eps('single')
    bFailureFlag = true;
    if coder.target('MATLAB') || coder.target('MEX')
        warning('Ray direction norm is zero at machine precision. Invalid input.');
    end
    return
elseif dDirectionNorm > 1.0 + 10 * eps('single') || dDirectionNorm < 1.0 - eps('single')

    if abs(dDirectionNorm - 1.0) < 10 * eps('single')
        % Acceptable numerical error, normalize
        dRayDirection_Frame = dRayDirection_Frame ./ dDirectionNorm;
    else
        % Call failure
        bFailureFlag = true;
        if coder.target('MATLAB') || coder.target('MEX')
            warning('Ray direction is not incorrect or not normalized. Invalid input.');
        end
        return
    end

end

% Compute a coefficient 
% DEVNOTE: this can be avoided in case of a sphere and set to 1, replacing the 1 with r^2 in C)
daCoeff = dAuxMatrix0 * dRayDirection_Frame;
% Compute b coefficient
dbCoeff = dAuxMatrix0 * dRayOriginFromEllipsCentre;
% Compute c coefficient
dcCoeff = dRayOriginFromEllipsCentre' * dEllipsoidMatrix * dRayOriginFromEllipsCentre - 1;

% Intersection equation discriminant
dDelta = dbCoeff^2 - daCoeff*dcCoeff;

% Parallel/degenerate configuration
if abs(daCoeff) < eps || ~isfinite(dDelta)
    bFailureFlag = true;
    return
end

% Evaluate intersection test
if dDelta < -eps
    return
end

bIntersectFlag = true;
dDelta = max(dDelta, 0); % Clamp tiny negative values due to numerical noise

% Near-tangency makes the derivatives ill-conditioned; leave all numeric outputs zero.
if dDelta <= coder.const(sqrt(eps))
    bIntersectFlag = false;
    bFailureFlag = true;
    if coder.target('MATLAB') || coder.target('MEX')
        warning('Intersection is near-tangent; jacobians are ill-conditioned and will not be computed.');
    end
    return
end

dInvAcoeff = 1 / daCoeff;
dSqrtDelta = sqrt(dDelta);

dtParam0 = dInvAcoeff * ( - dbCoeff + dSqrtDelta );
dtParam1 = dInvAcoeff * ( - dbCoeff - dSqrtDelta ); 

% Get the smallest positive intersection distance
if dtParam0 >= eps && dtParam1 >= eps
    % Both positive --> exterior intersect
    dIntersectDistance(:) = min([dtParam0, dtParam1]);

elseif dtParam0 >= eps || dtParam1 >= eps
    % One root positive, one negative --> interior intersect
    
    % Select the positive intersect
    if dtParam0 >= eps
        dIntersectDistance(:) = dtParam0;
    else
        dIntersectDistance(:) = dtParam1;
    end

else
    bIntersectFlag = false;
    bFailureFlag = false;
    return % Missed intersection (not a failure)
end

% Compute intersection point from ray equation if required
dIntersectPoint(:) = dRayOrigin_Frame + dRayDirection_Frame * dIntersectDistance;

%% Jacobian evaluation
if coder.const(nargout > 4) && (bEvaluateJacobians(1) || ...
        (coder.const(nargout > 5) && bEvaluateJacobians(2)))

    % Differentiate y' D y = 1 at the selected root. The signed normal/ray
    % product handles both entry and exit intersections.
    dPointFromCentre_TF = dRayOriginFromEllipsCentre + ...
        dRayDirection_Frame * dIntersectDistance;
    dSurfaceNormal_TF = dEllipsoidMatrix * dPointFromCentre_TF;
    dNormalDotRay = dSurfaceNormal_TF' * dRayDirection_Frame;

    if bEvaluateJacobians(1)
        dJacIntersectDistance_RayOrigin(:,:) = ...
            -(dSurfaceNormal_TF' * dDCM_EstTFfromFrame) / dNormalDotRay;
    end

    if coder.const(nargout > 5) && bEvaluateJacobians(2)
        % A positive left rotation gives dy = -skew(y)*delta at fixed range.
        dJacIntersectDistance_TargetAttErr(:,:) = ...
            (dSurfaceNormal_TF' * skewSymm(dPointFromCentre_TF)) / dNormalDotRay;
    end
end

end
