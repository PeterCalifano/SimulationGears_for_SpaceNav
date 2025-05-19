function [bIntersectFlag, dIntersectDistance, bFailureFlag, dIntersectPoint, ...
            dJacIntersectDistance_RayOrigin, dJacIntersectDistance_TargetAttErr] = RayEllipsoidIntersection(dRayOrigin_Frame, ...
                                                                                                             dRayDirection_Frame, ...
                                                                                                             dEllipsoidCentre_Frame, ...
                                                                                                             dEllipsoidInvDiagShapeCoeffs, ...
                                                                                                             dDCM_TFfromFrame, ...
                                                                                                             dDCM_EstTFfromFrame, ...
                                                                                                             bEvaluateJacobians) %#codegen
arguments (Input)
    dRayOrigin_Frame                    (3,1) double {isvector, isnumeric}
    dRayDirection_Frame                 (3,1) double {isvector, isnumeric}
    dEllipsoidCentre_Frame              (3,1) double {ismatrix, isnumeric}
    dEllipsoidInvDiagShapeCoeffs        (:,1) double {ismatrix, isnumeric} % [1/a^2; 1/b^2; 1/c^2]
    dDCM_TFfromFrame                    (3,3) double {ismatrix, isnumeric} = eye(3)           % Required for jacobians
    dDCM_EstTFfromFrame                 (3,3) double {ismatrix, isnumeric} = dDCM_TFfromFrame % Rotation including attitude error estimate
    bEvaluateJacobians                  (1,2) {islogical, isscalar} = [true, true]; 
end
arguments (Output)
    bIntersectFlag
    dIntersectDistance
    bFailureFlag
    dIntersectPoint
    dJacIntersectDistance_RayOrigin
    dJacIntersectDistance_TargetAttErr
end
%% SIGNATURE
% [bIntersectFlag, dIntersectDistance, bFailureFlag, dIntersectPoint, ...
%             dJacIntersectDistance_RayOrigin, dJacIntersectDistance_TargetAttErr] = RayEllipsoidIntersection(dRayOrigin_Frame, ...
%                                                                                                              dRayDirection_Frame, ...
%                                                                                                              dEllipsoidCentre_Frame, ...
%                                                                                                              dEllipsoidInvDiagShapeCoeffs, ...
%                                                                                                              dDCM_TFfromFrame, ...
%                                                                                                              dDCM_EstTFfromFrame) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to compute intersection of a 3D ellipsoid with a ray (line in 3D). Jacobians are evaluated if
% required by the user and flag is set to true.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRayOrigin_Frame                    (3,1) double {isvector, isnumeric}
% dRayDirection_Frame                 (3,1) double {isvector, isnumeric}
% dEllipsoidCentre_Frame              (3,1) double {ismatrix, isnumeric}
% dEllipsoidInvDiagShapeCoeffs        (:,1) double {ismatrix, isnumeric} % [1/a^2; 1/b^2; 1/c^2]
% dDCM_TFfromFrame                    (3,3) double {ismatrix, isnumeric} = eye(3)           % Required for jacobians
% dDCM_EstTFfromFrame                 (3,3) double {ismatrix, isnumeric} = dDCM_TFfromFrame % Rotation including attitude error estimate
% bEvaluateJacobians                  (1,2) {islogical, isscalar} = [true, true];
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bIntersectFlag
% dIntersectDistance
% bFailureFlag
% dIntersectPoint
% dJacIntersectDistance_RayOrigin
% dJacIntersectDistance_TargetAttErr
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-03-2025        Pietro Califano         First version of intersection test implemented.
% 04-03-2025        Pietro Califano         Implement jacobian evaluation wrt ray origin and target attitude
% 14-05-2025        Pietro Califano         Add flag to require/skip evaluation of jacobians
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code
% Data pre-processing
if nargout > 4
    dRayDirection_RefTF = dDCM_TFfromFrame * dRayDirection_Frame;
    dEllipsoidCentre_FramePreConv = dRayOrigin_Frame - dEllipsoidCentre_Frame;
end

if not(all(dDCM_EstTFfromFrame == eye(3)))
    % Convert IN-PLACE ray origin, direction and target position to target fixed frame
    dRayOrigin_Frame          = dDCM_EstTFfromFrame * dRayOrigin_Frame;
    dRayDirection_Frame       = dDCM_EstTFfromFrame * dRayDirection_Frame;
    
    if any(abs(dEllipsoidCentre_Frame) > 0)
        dEllipsoidCentre_Frame    = dDCM_EstTFfromFrame * dEllipsoidCentre_Frame;
    end
end

% Form ellipsoid shape matrix
dEllipsoidMatrix = diag(dEllipsoidInvDiagShapeCoeffs); % TODO check this is correct

% Initialize output
bIntersectFlag                      = false;
bFailureFlag                        = false;
dIntersectPoint                     = zeros(3, 1);
dIntersectDistance                  = zeros(1, 1);
dJacIntersectDistance_RayOrigin     = zeros(1, 3);
dJacIntersectDistance_TargetAttErr  = zeros(1, 3);

% Compute 2nd order intersection equation
% Equation: at^2 + 2bt + c = 0. Note that the b computed here is twice the B coefficient of a generic 
% quadratic equation. This is why the Delta and the solution looks slightly strange.

dRayOriginFromEllipsCentre = dRayOrigin_Frame - dEllipsoidCentre_Frame; % In Target fixed
dAuxMatrix0 = dRayDirection_Frame' * dEllipsoidMatrix;

% Compute a coefficient 
% NOTE: this can be avoided in case of a sphere and set to 1, replacing the 1 with r^2 in C)
daCoeff = dAuxMatrix0 * dRayDirection_Frame;
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
dInvAcoeff = 1 / daCoeff; 
dSqrtDelta = sqrt(dDelta); 

dtParam0 = dInvAcoeff * ( - dbCoeff + dSqrtDelta );
dtParam1 = dInvAcoeff * ( - dbCoeff - dSqrtDelta ); 

if dtParam0 >= eps && dtParam1 >= eps
    [dIntersectDistance(:), dSignSelector] = min([dtParam0, dtParam1]);
else
    bIntersectFlag = false;
    bFailureFlag = true;
    return % Test failure
end

% Compute intersection point from ray equation if required
dIntersectPoint(:) = dRayOrigin_Frame + dRayDirection_Frame * dIntersectDistance;


%% Jacobian evaluation
% DEVNOTE TODO: can be optimized both in terms of memory and computations
if nargout > 4 && bEvaluateJacobians(1)
    % Pre-compute shared quantities
    dInvSqrtDelta   = 1/dSqrtDelta;
    dJac_cCoeff_RayOriginInTF = dRayOriginFromEllipsCentre' * ( dEllipsoidMatrix + transpose(dEllipsoidMatrix) ); % * eye(3);
    
    dSign = 0.0;
    if dSignSelector == 1
        dSign = 1.0;
    elseif dSignSelector == 2
        dSign = -1.0;
    end

    if coder.target('MATLAB') || coder.target('MEX')
        assert(abs(dSign) > 0, 'ERROR: dSign variable cannot be zero.')
    end
    
    % Compute jacobian of intersection distance wrt ray origin in input Frame 
    dJacIntersectDist_RayOriginInTF = - dRayDirection_Frame' * dEllipsoidMatrix + ...
             (-dSign * dInvSqrtDelta * (2 * dbCoeff * dRayDirection_Frame' * dEllipsoidMatrix - daCoeff * dJac_cCoeff_RayOriginInTF) );

    dJacIntersectDistance_RayOrigin(:,:) = dInvAcoeff * dJacIntersectDist_RayOriginInTF * dDCM_EstTFfromFrame;

    if nargout > 5 && bEvaluateJacobians(2)
        % Compute auxiliary quantities
        dCameraPosFromCentre_FramePreConv = dDCM_TFfromFrame * (dEllipsoidCentre_FramePreConv);

        % Derivative of a coefficient wrt target attitude error
        dJac_aCoeff_AttErr = transpose(dRayDirection_Frame) * ( dEllipsoidMatrix + transpose(dEllipsoidMatrix) ) * skewSymm(dRayDirection_RefTF); 
        
        % Derivative of b coefficient wrt target attitude error
        % FIXME, first entry is likely wrong (become scalar!), while second cannot be multiplied
        dJac_bCoeff_AttErr = transpose( transpose( skewSymm(dRayDirection_RefTF) ) * (dEllipsoidMatrix * dCameraPosFromCentre_FramePreConv) )...
                              + transpose(dRayDirection_Frame) * dEllipsoidMatrix * (- skewSymm(dCameraPosFromCentre_FramePreConv) );

        % Derivative of c coefficient wrt target attitude error
        dJac_cCoeff_AttErr = dJac_cCoeff_RayOriginInTF * (- skewSymm(dCameraPosFromCentre_FramePreConv) );

        % Compute jacobian of intersection distance wrt small target attitude error
        dAuxJac0 = - dInvAcoeff^2 * dJac_aCoeff_AttErr * (dbCoeff + dSign * dSqrtDelta);
        dAuxJac1 = dInvAcoeff * (- dJac_bCoeff_AttErr + (dSign * dInvSqrtDelta * ...
                        (2*dbCoeff * dJac_bCoeff_AttErr  - (dJac_aCoeff_AttErr * dcCoeff + daCoeff * dJac_cCoeff_AttErr) )) );

        dJacIntersectDistance_TargetAttErr(:,:) = dAuxJac0 + dAuxJac1;
    end
end

end
