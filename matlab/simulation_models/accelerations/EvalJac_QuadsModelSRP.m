function [dJacAccSRP_IN, dJacAccSRPdir_IN, dJacAccSRPpressure_IN] = EvalJac_QuadsModelSRP(dPosSCtoSun_IN, ...
                                                                                           dQuat_INfromSCB, ...
                                                                                           dMassSC, ...
                                                                                           dSolarPressure, ...
                                                                                           dSpacecraftQuadsArea, ...
                                                                                           dDiffSpecQuadsCoeffs, ...
                                                                                           dQuadsNormals_SCB, ...
                                                                                           bIsInEclipse, ...
                                                                                           bRecomputePressureFromDistance) %#codegen
arguments (Input)
    dPosSCtoSun_IN                 (3,1) double {mustBeFinite}
    dQuat_INfromSCB                (4,1) double {mustBeFinite}
    dMassSC                        (1,1) double {mustBeFinite, mustBePositive}
    dSolarPressure                 (1,1) double {mustBeFinite, mustBeNonnegative}
    dSpacecraftQuadsArea           {mustBeVector, mustBeFinite, mustBeNonnegative}
    dDiffSpecQuadsCoeffs           (:,2) double {mustBeFinite, mustBeNonnegative}
    dQuadsNormals_SCB              (3,:) double {mustBeFinite}
    bIsInEclipse                   (1,1) logical = false
    bRecomputePressureFromDistance (1,1) logical = false
end
arguments (Output)
    dJacAccSRP_IN       (3,3) double
    dJacAccSRPdir_IN    (3,3) double
    dJacAccSRPpressure_IN (3,3) double
end
%% PROTOTYPE
% [dJacAccSRP_IN, dJacAccSRPdir_IN, dJacAccSRPpressure_IN] = EvalJac_QuadsModelSRP(dPosSCtoSun_IN, ...
%     dQuat_INfromSCB, dMassSC, dSolarPressure, dSpacecraftQuadsArea, dDiffSpecQuadsCoeffs, ...
%     dQuadsNormals_SCB, bIsInEclipse, bRecomputePressureFromDistance) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical position Jacobian of the flat-panel SRP acceleration returned by ComputeQuadsModelSRP.
% The derivative is taken with respect to spacecraft inertial position, with Sun position, spacecraft attitude, mass, panel geometry, and optical coefficients frozen at the linearization point.
% The panel illumination and eclipse tests are hard switches in the force model. This Jacobian therefore uses
% the active illuminated panel set at the linearization point and does not differentiate the switches themselves. At exact shadow or terminator boundaries the physical model is discontinuous, so this function returns the frozen-active-set derivative implied by the current branch.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_IN                 (3,1) double   Spacecraft-to-Sun vector in inertial frame [LU].
% dQuat_INfromSCB                (4,1) double   Quaternion rotating SC body vectors into inertial frame.
% dMassSC                        (1,1) double   Spacecraft mass [kg].
% dSolarPressure                 (1,1) double   Current solar pressure at spacecraft range [force/area].
% dSpacecraftQuadsArea           (N,1) double   Surface-element areas.
% dDiffSpecQuadsCoeffs           (N,2) double   [diffuse, specular] optical coefficients per panel.
% dQuadsNormals_SCB              (3,N) double   Surface outward normals in spacecraft body frame.
% bIsInEclipse                   (1,1) logical  If true, SRP Jacobian is zero.
% bRecomputePressureFromDistance (1,1) logical  If true, include derivative of inverse-square pressure.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacAccSRP_IN                  (3,3) double   Total d(a_SRP)/d(r_SC) in inertial coordinates.
% dJacAccSRPdir_IN               (3,3) double   Contribution from Sun-direction variation.
% dJacAccSRPpressure_IN          (3,3) double   Contribution from inverse-square pressure variation.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-07-2026    Pietro Califano, Codex 5.5      Add analytical flat-panel SRP acceleration Jacobian.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Quat2DCM()   [MathCore_for_SpaceNav]
% -------------------------------------------------------------------------------------------------------------

%% Function code
dSpacecraftQuadsArea = dSpacecraftQuadsArea(:);
dNumQuads = size(dQuadsNormals_SCB, 2);

% Initial checks
assert(numel(dSpacecraftQuadsArea) == dNumQuads, ...
    'EvalJac_QuadsModelSRP:AreaSizeMismatch', ...
    'The areas vector must contain one value per quadrilateral.');
assert(size(dDiffSpecQuadsCoeffs, 1) == dNumQuads, ...
    'EvalJac_QuadsModelSRP:CoeffSizeMismatch', ...
    'The optical-coefficients matrix must have one row per quadrilateral.');
assert(all(dDiffSpecQuadsCoeffs(:) <= 1), ...
    'EvalJac_QuadsModelSRP:CoeffRange', ...
    'Diffuse and specular coefficients must lie in the [0, 1] interval.');

dDistSCtoSun = norm(dPosSCtoSun_IN);
assert(dDistSCtoSun > 0.0, ...
    'EvalJac_QuadsModelSRP:ZeroSunSpacecraftDistance', ...
    'Sun-spacecraft distance must be positive.');

% Allocate outputs
dJacAccSRP_IN = zeros(3, 3);
dJacAccSRPdir_IN = zeros(3, 3);
dJacAccSRPpressure_IN = zeros(3, 3);

% Match ComputeQuadsModelSRP: normalize the quaternion and panel normals before applying the panel law.
dQuat_INfromSCB = dQuat_INfromSCB / max(norm(dQuat_INfromSCB), eps);
dDCM_INfromSCB = Quat2DCM(dQuat_INfromSCB);
dSunDir_IN = dPosSCtoSun_IN / dDistSCtoSun;
dSunDir_SCB = dDCM_INfromSCB.' * dSunDir_IN;

dQuadNormalNorm = sqrt(sum(dQuadsNormals_SCB.^2, 1));
assert(all(dQuadNormalNorm > 0), ...
    'EvalJac_QuadsModelSRP:ZeroNormal', ...
    'Quadrilateral normals must be non-zero.');
dQuadsNormals_SCB = dQuadsNormals_SCB ./ dQuadNormalNorm;

if bIsInEclipse || dSolarPressure == 0.0
    return;
end

dJacAccelWrtSunDir_SCB = zeros(3, 3);
dAccelSRP_SCB = zeros(3, 1);
dEye3 = eye(3);

% Evaluate Jacobian for each quadrilateral surface element
for idQ = 1:dNumQuads

    dArea = dSpacecraftQuadsArea(idQ);
    dNormal_SCB = dQuadsNormals_SCB(:, idQ);
    dDiffuseCoeff = dDiffSpecQuadsCoeffs(idQ, 1);
    dSpecularCoeff = dDiffSpecQuadsCoeffs(idQ, 2);
    dAbsorbCoeff = 1.0 - dSpecularCoeff;
    dMu = dot(dNormal_SCB, dSunDir_SCB);

    % The SRP force is piecewise smooth. Keep the active illuminated set frozen at the linearization point.
    if dMu <= 0.0 || dArea == 0.0
        continue;
    end

    % Panel acceleration:
    % a_i = -(P A_i / m) * mu * ([2(Cd/3 + Cs mu)] n_i + (1 - Cs) s)
    dPanelScale = -dSolarPressure * dArea / dMassSC;
    dNormalForceCoeff = 2.0 * (dDiffuseCoeff / 3.0 + dSpecularCoeff * dMu);
    dAccelSRP_SCB = dAccelSRP_SCB + ...
        dPanelScale * dMu * (dNormalForceCoeff * dNormal_SCB + dAbsorbCoeff * dSunDir_SCB);

    % With mu = n_i' s, the smooth active-panel derivative is:
    % d/ds {mu [2(Cd/3 + Cs mu)n + (1 - Cs)s]}
    %   = (2Cd/3 + 4Cs mu) n n' + (1 - Cs)(s n' + mu I)
    dJacPanelWrtSunDir_SCB = dPanelScale * ( ...
        (2.0 * dDiffuseCoeff / 3.0 + 4.0 * dSpecularCoeff * dMu) * (dNormal_SCB * dNormal_SCB.') + ...
        dAbsorbCoeff * (dSunDir_SCB * dNormal_SCB.' + dMu * dEye3));

    % Sum contribution of ith panel
    dJacAccelWrtSunDir_SCB = dJacAccelWrtSunDir_SCB + dJacPanelWrtSunDir_SCB;
end

% Direction chain rule. Since q = r_Sun - r_SC and s = q / ||q||:
% ds_IN / dr_SC = -(I - s_IN s_IN') / ||q||.
dSunDirProjector_IN = dEye3 - dSunDir_IN * dSunDir_IN.';
dSunDirWrtScPos_IN = -dSunDirProjector_IN / dDistSCtoSun;
dJacAccSRPdir_IN = dDCM_INfromSCB * dJacAccelWrtSunDir_SCB * dDCM_INfromSCB.' * dSunDirWrtScPos_IN;

if bRecomputePressureFromDistance
    % Pressure is already the current value. For P = P0 R0^2 / ||q||^2,
    % dP/dr_SC = 2 P s_IN / ||q||, so d(a)/dr adds (a/P) dP/dr.
    dAccelSRP_IN = dDCM_INfromSCB * dAccelSRP_SCB;
    dJacAccSRPpressure_IN = dAccelSRP_IN * (2.0 * dSunDir_IN / dDistSCtoSun).';
end

dJacAccSRP_IN = dJacAccSRPdir_IN + dJacAccSRPpressure_IN;

end
