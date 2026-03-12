function [dIntersectPointUV, dProjectSunDir_UV] = RayEllipseIntersection(dDCM_fromTFtoCAM, ...
                                                                        dTargetPosVec_CAM, ...
                                                                        dShapeMatrix_TF, ...
                                                                        dKcam, ...
                                                                        dSunDirection_CAM, ...
                                                                        dDCM_CAMfromUVplane) %#codegen
arguments
    dDCM_fromTFtoCAM    (3,3) {ismatrix, isnumeric}
    dTargetPosVec_CAM   (3,1) {isvector, isnumeric}
    dShapeMatrix_TF     (3,3) {ismatrix, isnumeric}
    dKcam               (3,3) {ismatrix, isnumeric}
    dSunDirection_CAM   (3,1) {isvector, isnumeric}
    dDCM_CAMfromUVplane (3,3) {ismatrix, isnumeric} = eye(3);
end

% Compute R considering projected ellipse and sun direction
% px = cam.res(1)/2+0.5;
% py = cam.res(2)/2+0.5;

% Define output variables
dIntersectPointUV = zeros(2,1);
dProjectSunDir_UV = zeros(3,1);

dEllipseCentreX = dKcam(1,3) + 0.5;
dEllipseCentreY = dKcam(2,3) + 0.5;

% Project ellipsoid onto image plane
% P = K_c*[rotSC, tSC];
% Ae_proj = inv(P*(Ae_h\P'));

dProjectedEllipseMatrix = ProjectEllipsoidOnImagePlane(dKcam, ...
                                                       dShapeMatrix_TF, ...
                                                       dDCM_fromTFtoCAM, ...
                                                       dTargetPosVec_CAM, ...
                                                       true);

% Project Light direction onto image plane
dProjectSunDir_UV(:) = transpose(dDCM_CAMfromUVplane) * dSunDirection_CAM;
dProjectSunDir_UV = dProjectSunDir_UV / norm(dProjectSunDir_UV); % TODO, check if this is correct? Should it be / dProjectSunDir_UV(3)?

% Extract the coefficients of the conic matrix
dA = dProjectedEllipseMatrix(1, 1);
dB = 2*dProjectedEllipseMatrix(1, 2); % B/2 is stored in the matrix
dC = dProjectedEllipseMatrix(2, 2);
dD = 2*dProjectedEllipseMatrix(1, 3); % D/2 is stored in the matrix
dE = 2*dProjectedEllipseMatrix(2, 3); % E/2 is stored in the matrix
dF = dProjectedEllipseMatrix(3, 3);

% Sun LOS projection components
dDirX = dProjectSunDir_UV(1);
dDirY = dProjectSunDir_UV(2);

% Compute quadratic coefficients
% dAlpha: 2nd order terms
% dBeta: 1st order terms
% dGamma: 0th order terms
dAlphaCoeff = dA * dDirX^2 + dB * dDirX*dDirY + dC *dDirY^2;

dBetaCoeff = 2 * dA * dEllipseCentreX *dDirX + dB* (dEllipseCentreX*dDirY + dEllipseCentreY*dDirX) + ...
                  2*dC * dEllipseCentreY * dDirY + dD*dDirX + dE*dDirY;

dGammaCoeff = dA * dEllipseCentreX^2 + dB * (dEllipseCentreX * dEllipseCentreY) + ...
                 dC * dEllipseCentreY^2 + dD * dEllipseCentreX + dE * dEllipseCentreY + dF;

% Compute solution discriminant
dDelta = dBetaCoeff^2 - 4 * dAlphaCoeff * dGammaCoeff;

if dDelta < 0
    % No solution
    return;
end

% Compute first intersection point
dDeltaSqrt = sqrt(dDelta);
dInvAlfa = 1 / (2*dAlphaCoeff);
dtParam1 = (- dBetaCoeff + dDeltaSqrt ) * dInvAlfa;

dTmpIntersectPoint = [dtParam1 * dDirX; 
                       dtParam1 * dDirY];

dTmpNormIntersectPoint1 = dTmpIntersectPoint./norm(dTmpIntersectPoint);

% Check first point is the one toward the ray direction
if (dTmpNormIntersectPoint1' * dProjectSunDir_UV(1:2)) > 0
    dIntersectPointUV(:) = dTmpIntersectPoint + [dEllipseCentreX; dEllipseCentreY];
    return
end

% Else the other point must be the right one
dtParam2 = (- dBetaCoeff - dDeltaSqrt ) * dInvAlfa;

dTmpIntersectPoint = [dEllipseCentreX + dtParam2 * dDirX;
                       dEllipseCentreY + dtParam2 * dDirY];

dIntersectPointUV(:) = dTmpIntersectPoint + [dEllipseCentreX; dEllipseCentreY];


end
