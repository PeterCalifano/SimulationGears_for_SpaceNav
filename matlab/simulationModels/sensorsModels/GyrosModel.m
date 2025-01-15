function [o_dMeasAngRate, dNewBiasValue] = GyrosModel(i_dTrueAngRate, i_dScaleFactor, ...
    i_dOrthogErrMatrix, i_dConfigMat_IMUfromSCB, i_dSigmaRRW, i_dSigmaARW, ...
    i_dPrevBiasValue, i_dSamplingTime)%#codegen

arguments
    i_dTrueAngRate          (3,1) double    
    i_dScaleFactor          (3,1) double
    i_dOrthogErrMatrix      (3,3) double
    i_dConfigMat_IMUfromSCB (3,3) double
    i_dSigmaRRW             (1,1) double
    i_dSigmaARW             (1,1) double
    i_dPrevBiasValue        (3,1) double
    i_dSamplingTime         (1,1) double
end 

% Variables declaration
dNoiseInputRRW = zeros(3,1);
dNewBiasValue  = zeros(3,1);
dNoiseInputARW = zeros(3,1);
o_dMeasAngRate = zeros(3,1);

% Simulate White noise and random walk bias values
dNoiseInputRRW(:) = 1/sqrt(2*i_dSamplingTime) * mvnrnd([0;0;0], i_dSigmaRRW * eye(3));
dNoiseInputARW(:) = 1/sqrt(2*i_dSamplingTime) * mvnrnd([0;0;0], i_dSigmaARW * eye(3));

dNewBiasValue(:) = i_dPrevBiasValue + dNoiseInputRRW;


o_dMeasAngRate(:) = ((eye(3) + diag(i_dScaleFactor)) *  (i_dOrthogErrMatrix * i_dConfigMat_IMUfromSCB) * i_dTrueAngRate) + ...
                                                        dNewBiasValue + dNoiseInputARW;

end
