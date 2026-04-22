function [dSRPaccel_IN, dSRPtorque_SCB, dQuadsCosSunPhaseAngle] = ComputeQuadsModelSRP(dDirSCtoSun_SCB, ...
                                                                                        dqSCBwrtIN, ...
                                                                                        dMassSC, ...
                                                                                        dCoMpos_SCB, ...
                                                                                        dSolarPressure, ...
                                                                                        dSCquadsArea, ...
                                                                                        dDiffSpecQuadsCoeffs, ...
                                                                                        dQuadsNormals_SCB, ...
                                                                                        dQuadsPressCentre_SCB) %#codegen
arguments (Input)
    dDirSCtoSun_SCB        (3,1) double {mustBeFinite}
    dqSCBwrtIN             (4,1) double {mustBeFinite}
    dMassSC                (1,1) double {mustBeFinite, mustBePositive}
    dCoMpos_SCB            (3,1) double {mustBeFinite}
    dSolarPressure         (1,1) double {mustBeFinite, mustBeNonnegative}
    dSCquadsArea                    {mustBeVector, mustBeFinite, mustBeNonnegative}
    dDiffSpecQuadsCoeffs   (:,2) double {mustBeFinite, mustBeNonnegative}
    dQuadsNormals_SCB      (3,:) double {mustBeFinite}
    dQuadsPressCentre_SCB  (3,:) double {mustBeFinite}
end
arguments (Output)
    dSRPaccel_IN           (3,1) double
    dSRPtorque_SCB         (3,1) double
    dQuadsCosSunPhaseAngle (:,1) double
end
%% PROTOTYPE
% [dSRPaccel_IN, dSRPtorque_SCB, dQuadsCosSunPhaseAngle] = ComputeQuadsModelSRP(dDirSCtoSun_SCB, ...
%     dqSCBwrtIN, dMassSC, dCoMpos_SCB, dSolarPressure, dSCquadsArea, dDiffSpecQuadsCoeffs, ...
%     dQuadsNormals_SCB, dQuadsPressCentre_SCB) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes solar-radiation-pressure acceleration and torque by integrating the flat-plate SRP law over
% a set of illuminated quadrilateral surface elements expressed in the spacecraft body frame.
% Optical coefficients are arranged row-wise as [diffuse, specular].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dDirSCtoSun_SCB        (3,1) double   Spacecraft-to-Sun direction in SC body frame [-]
% dqSCBwrtIN             (4,1) double   Quaternion rotating SC body vectors into inertial frame (SVRP+, scalar first)
% dMassSC                (1,1) double   Spacecraft mass [kg]
% dCoMpos_SCB            (3,1) double   Spacecraft center-of-mass position in SC body frame [m]
% dSolarPressure         (1,1) double   Solar radiation pressure at spacecraft distance [N/m^2]
% dSCquadsArea           (N,1) double   Surface-element areas [m^2]
% dDiffSpecQuadsCoeffs   (N,2) double   [diffuse, specular] optical coefficients per surface element [-]
% dQuadsNormals_SCB      (3,N) double   Surface outward normals in SC body frame [-]
% dQuadsPressCentre_SCB  (3,N) double   Surface pressure-center positions in SC body frame [m]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSRPaccel_IN           (3,1) double   SRP acceleration in inertial frame [m/s^2]
% dSRPtorque_SCB         (3,1) double   SRP torque in spacecraft body frame [N m]
% dQuadsCosSunPhaseAngle (N,1) double   Cosine of the Sun phase angle for each surface element [-]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-12-2023    Pietro Califano                 First prototype as PanelModelSRP.
% 22-04-2026    Pietro Califano, Codex 5.4      Modernized naming, argument validation, normalization, 
%                                               and validation-ready header.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Quat2DCM()   [MathCore_for_SpaceNav]
% -------------------------------------------------------------------------------------------------------------

dSCquadsArea = dSCquadsArea(:);
nQuads = size(dQuadsNormals_SCB, 2);

assert(numel(dSCquadsArea) == nQuads, ...
    'ComputeQuadsModelSRP:AreaSizeMismatch', ...
    'The areas vector must contain one value per quadrilateral.');
assert(size(dDiffSpecQuadsCoeffs, 1) == nQuads, ...
    'ComputeQuadsModelSRP:CoeffSizeMismatch', ...
    'The optical-coefficients matrix must have one row per quadrilateral.');
assert(size(dQuadsPressCentre_SCB, 2) == nQuads, ...
    'ComputeQuadsModelSRP:PressCentreSizeMismatch', ...
    'The pressure-center matrix must have one column per quadrilateral.');
assert(all(dDiffSpecQuadsCoeffs(:) <= 1), ...
    'ComputeQuadsModelSRP:CoeffRange', ...
    'Diffuse and specular coefficients must lie in the [0, 1] interval.');

dSunDir_SCB = NormalizeVector(dDirSCtoSun_SCB, 'Sun direction must be non-zero.');
dqSCBwrtIN = NormalizeVector(dqSCBwrtIN, 'Input quaternion must be non-zero.');

dQuadNormalNorm = sqrt(sum(dQuadsNormals_SCB.^2, 1));
assert(all(dQuadNormalNorm > 0), ...
    'ComputeQuadsModelSRP:ZeroNormal', ...
    'Quadrilateral normals must be non-zero.');
dQuadsNormals_SCB = dQuadsNormals_SCB ./ dQuadNormalNorm;

dSRPtorque_SCB = zeros(3, 1);
dQuadsCosSunPhaseAngle = zeros(nQuads, 1);
dForcePerQuad_SCB = zeros(3, nQuads);

for idQ = 1:nQuads
    dQuadsCosSunPhaseAngle(idQ) = dot(dQuadsNormals_SCB(:, idQ), dSunDir_SCB);

    if dQuadsCosSunPhaseAngle(idQ) <= 0 || dSCquadsArea(idQ) == 0
        continue;
    end

    dForcePerQuad_SCB(:, idQ) = -dSolarPressure * dSCquadsArea(idQ) * dQuadsCosSunPhaseAngle(idQ) * ...
        (2 * (dDiffSpecQuadsCoeffs(idQ, 1) / 3 + dDiffSpecQuadsCoeffs(idQ, 2) * dQuadsCosSunPhaseAngle(idQ)) * dQuadsNormals_SCB(:, idQ) + ...
        (1 - dDiffSpecQuadsCoeffs(idQ, 2)) * dSunDir_SCB);

    dSRPtorque_SCB = dSRPtorque_SCB + cross(dQuadsPressCentre_SCB(:, idQ) - dCoMpos_SCB, dForcePerQuad_SCB(:, idQ));
end

% Compute accelerations
dSRPaccel_SCB = sum(dForcePerQuad_SCB, 2) / dMassSC;
dDCM_INfromSCB = Quat2DCM(dqSCBwrtIN);
dSRPaccel_IN = dDCM_INfromSCB * dSRPaccel_SCB;

end

%% Internal helpers
function dUnitVector = NormalizeVector(dVector, charErrorMessage)
coder.inline('always')
dNorm = norm(dVector);
assert(dNorm > 0, 'ComputeQuadsModelSRP:ZeroVector', charErrorMessage);
dUnitVector = dVector / dNorm;
end
