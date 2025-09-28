function dTightConeLocusImageMatrix = ProjectEllipsoidOnImagePlane(dKcam, ...
                                                                    dShapeMatrix_TF, ...
                                                                    dDCM_fromTFtoCAM, ...
                                                                    dBodyPosVec_CAM, ...
                                                                    bNormalizeMatrix) %#codegen
arguments
    dKcam            (3,3) double
    dShapeMatrix_TF  (3,3) double
    dDCM_fromTFtoCAM (3,3) double
    dBodyPosVec_CAM  (3,1) double
    bNormalizeMatrix (1,1) logical {islogical} = true
end

% Rotate ellipsoid matrix to Camera frame
dShapeMatrix_CAM = dDCM_fromTFtoCAM * dShapeMatrix_TF * dDCM_fromTFtoCAM';

% Compute inverse of camera intrinsics matrix
invKcam = eye(3)/dKcam; % TODO replace with analytical formula!

% Project ellipsoid matrix onto image plane
dTightConeLocusImageMatrix = transpose(invKcam) * ( (dShapeMatrix_CAM * dBodyPosVec_CAM) * (dBodyPosVec_CAM' * dShapeMatrix_CAM) ...
    - (dBodyPosVec_CAM' * dShapeMatrix_CAM * dBodyPosVec_CAM - 1.0) * dShapeMatrix_CAM) * invKcam;

% Perform normalization to get matrix in Projection Space P^2
if bNormalizeMatrix
    dTightConeLocusImageMatrix = dTightConeLocusImageMatrix./dTightConeLocusImageMatrix(3,3);
end

% Remove numerical zeros
dTightConeLocusImageMatrix(abs(dTightConeLocusImageMatrix) < eps) = 0;

end
