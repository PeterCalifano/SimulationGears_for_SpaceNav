function strVerification = testSrpLutAcceleration()
%% SIGNATURE
% strVerification = testSrpLutAcceleration()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify SI position, mass and right body-attitude partials against independent
% constant-cannonball formulas and finite perturbations. Include explicit
% attitude-position derivatives in both scalar/full modes.
% Example: strVerification = testSrpLutAcceleration();
% Output: Passing physical chain-rule evidence.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Run SetupSimGears and add this test directory first.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification   Maximum position/attitude/mass derivative discrepancies.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-09-2026  Pietro Califano, Codex gpt-6  Verify physical SRP chain rules.
% 01-10-2026  Pietro Califano, Codex gpt-6  Remove unused partial-eclipse cases.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpLutTestFixture, ComputeSrpLutAcceleration.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

arguments (Output)
    strVerification (1, 1) struct
end

% Recover the independent constant-cannonball force and physical derivatives.
dPosSCtoSun_IN = [1300;370;280];
dPressure = 4e-6;
dMass = 12;
strConstant = BuildSrpLutTestFixture(true);
[dAccel, dJacPosition, dJacAttitude, dJacMass] = ComputeSrpLutAcceleration( ...
    dPosSCtoSun_IN, eye(3), dMass, dPressure, strConstant, false, true);
dSunRange = norm(dPosSCtoSun_IN);
dSunDir_IN = dPosSCtoSun_IN/dSunRange;
dExpected = dPressure/dMass/dSunRange*(eye(3)-3*(dSunDir_IN*dSunDir_IN.'));
assert(norm(dAccel+dPressure/dMass*dSunDir_IN) < 1e-20 && ...
    norm(dJacPosition-dExpected, 'fro') < 1e-20 && norm(dJacAttitude, 'fro') < 1e-20 && ...
    norm(dJacMass+dAccel/dMass) < 1e-20);

% Perturb the same pressure and rotation laws used by the chain rule.
strLut = BuildSrpLutTestFixture(false);
dAngle = 0.21;
dDCM_INfromSCB = Rotation_(dAngle);
dAngleGradient = [0.01, -0.02, 0.015];
dJacDCMWrtPos_INfromSCB = zeros(3, 3, 3);
for ui32Axis = uint32(1):uint32(3)
    dJacDCMWrtPos_INfromSCB(:, :, ui32Axis) = dDCM_INfromSCB*[0, -1, 0;1, 0, 0;0, 0, 0]*dAngleGradient(ui32Axis);
end

% Accumulate separate discrepancies for position, attitude and mass partials.
dMaxPosition = 0;
dMaxAttitude = 0;
dMaxMass = 0;
for bTransverse = [false, true]
    [dAccel, dJacPosition, dJacAttitude, dJacMass] = ComputeSrpLutAcceleration( ...
        dPosSCtoSun_IN, dDCM_INfromSCB, dMass, dPressure, strLut, bTransverse, true, ...
        dJacDCMWrtPos_INfromSCB);

    % Preserve each physical output prefix when optional partials are skipped.
    [dPrefixAccel, dPrefixPosition] = ComputeSrpLutAcceleration(dPosSCtoSun_IN, ...
        dDCM_INfromSCB, dMass, dPressure, strLut, bTransverse, true, dJacDCMWrtPos_INfromSCB);
    assert(isequal(dPrefixAccel, dAccel) && isequal(dPrefixPosition, dJacPosition));
    [dPrefixAccel, dPrefixPosition, dPrefixAttitude] = ComputeSrpLutAcceleration( ...
        dPosSCtoSun_IN, dDCM_INfromSCB, dMass, dPressure, strLut, bTransverse, true, ...
        dJacDCMWrtPos_INfromSCB);
    assert(isequal(dPrefixAccel, dAccel) && isequal(dPrefixPosition, dJacPosition) && ...
        isequal(dPrefixAttitude, dJacAttitude));

    % Differentiate position and right body-attitude errors independently.
    dPositionFd = zeros(3, 3);
    dAttitudeFd = zeros(3, 3);
    for ui32Axis = uint32(1):uint32(3)
        dPositionStep = zeros(3, 1);
        dPositionStep(ui32Axis) = 1e-4;
        dPositive = PositionForce_(dPositionStep, dPosSCtoSun_IN, dMass, dPressure, ...
            strLut, bTransverse, dAngle, dAngleGradient);
        dNegative = PositionForce_(-dPositionStep, dPosSCtoSun_IN, dMass, dPressure, ...
            strLut, bTransverse, dAngle, dAngleGradient);
        dPositionFd(:, ui32Axis) = (dPositive-dNegative)/(2e-4);
        dRotationStep = zeros(3, 1);
        dRotationStep(ui32Axis) = 1e-6;
        dRotationStepSkew_SCB = [0, -dRotationStep(3), dRotationStep(2); ...
            dRotationStep(3), 0, -dRotationStep(1);-dRotationStep(2), dRotationStep(1), 0];
        dPositive = ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB*expm(dRotationStepSkew_SCB), ...
            dMass, dPressure, strLut, bTransverse, false);
        dNegative = ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB*expm(-dRotationStepSkew_SCB), ...
            dMass, dPressure, strLut, bTransverse, false);
        dAttitudeFd(:, ui32Axis) = (dPositive-dNegative)/(2e-6);
    end

    % Vary mass without changing pressure or spacecraft geometry.
    dMassFd = (ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB, dMass+1e-4, ...
        dPressure, strLut, bTransverse, false)- ...
        ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB, dMass-1e-4, ...
        dPressure, strLut, bTransverse, false))/(2e-4);
    dMaxPosition = max(dMaxPosition, norm(dJacPosition-dPositionFd, 'fro'));
    dMaxAttitude = max(dMaxAttitude, norm(dJacAttitude-dAttitudeFd, 'fro'));
    dMaxMass = max(dMaxMass, norm(dJacMass-dMassFd));
    assert(dMaxPosition < 2e-15 && dMaxAttitude < 2e-15 && dMaxMass < 2e-15);
    [dInactive, dInactiveJac] = ComputeSrpLutAcceleration([0;0;0], eye(3), dMass, ...
        0, strLut, bTransverse, true);
    assert(all(dInactive == 0) && all(dInactiveJac == 0, 'all'));
    assert(norm(dAccel-ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB, dMass, ...
        dPressure, strLut, bTransverse, true)) < 1e-20);
end
strVerification = struct('bPassed', true, 'bIndependentCannonballPassed', true, ...
    'dMaxPositionError', dMaxPosition, 'dMaxAttitudeError', dMaxAttitude, 'dMaxMassError', dMaxMass);
disp(strVerification);
end

function dForce = PositionForce_(dScPosOffset_IN, dPosSCtoSun_IN, dMass, dPressure, strLut, ...
    bTransverse, dAngle, dAngleGradient)
% Evaluate the independently perturbed physical inputs for a position difference.
arguments (Input)
    dScPosOffset_IN (3, 1) double
    dPosSCtoSun_IN (3, 1) double
    dMass (1, 1) double
    dPressure (1, 1) double
    strLut (1, 1) struct
    bTransverse (1, 1) logical
    dAngle (1, 1) double
    dAngleGradient (1, 3) double
end

arguments (Output)
    dForce (3, 1) double
end

% Perturb pressure and pointing consistently with the spacecraft position offset.
dPerturbedSunPos_IN = dPosSCtoSun_IN-dScPosOffset_IN;
dCurrentPressure = dPressure*dot(dPosSCtoSun_IN, dPosSCtoSun_IN)/dot(dPerturbedSunPos_IN, dPerturbedSunPos_IN);
dDCM_INfromSCB = Rotation_(dAngle+dAngleGradient*dScPosOffset_IN);
dForce = ComputeSrpLutAcceleration(dPerturbedSunPos_IN, dDCM_INfromSCB, dMass, dCurrentPressure, strLut, ...
    bTransverse, true);
end

function dDCM_INfromSCB = Rotation_(dAngle)
% Rotate about body Z for a reproducible state-dependent attitude fixture.
arguments (Input)
    dAngle (1, 1) double
end

arguments (Output)
    dDCM_INfromSCB (3, 3) double
end
dDCM_INfromSCB = [cos(dAngle), -sin(dAngle), 0;sin(dAngle), cos(dAngle), 0;0, 0, 1];
end
