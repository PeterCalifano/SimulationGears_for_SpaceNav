function [dForcePerPressure, dResponseJacobian, dTorquePerPressure] = ...
    ComputePanelSrpResponse(dDirections, strPanel, bSelfShadowing) %#codegen
%% SIGNATURE
% [dForce, dJacobian, dTorque] = ComputePanelSrpResponse(dDirections, strPanel, bSelfShadowing)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate prepared panel SRP response for one or several independent directions.
% Return body-frame force divided by pressure, with optional direction partials
% and torque about the body origin. Keep pressure, mass, external eclipse and
% attitude outside this geometry/optics response. Differentiate normalization
% and the active panel law while holding sampled visibility fixed.
% Example: dForce = ComputePanelSrpResponse([1;0;0],strPanel,true);
% Output: A 3-by-1 body force/pressure response in m^2.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dDirections       (3,K) nonzero spacecraft-to-Sun vectors in body coordinates.
% strPanel          Numeric areas [m^2], optics [diffuse,specular], normals,
%                   pressure centres [m] and optional prepared strShadowData.
% bSelfShadowing    Apply opaque two-sided sampled visibility; default true.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dForcePerPressure (3,K) body force/pressure [m^2].
% dResponseJacobian (3,3,K) partials with respect to each supplied direction;
%                   visibility and illuminated active set remain frozen.
% dTorquePerPressure (3,K) torque/pressure about the body origin [m^3].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Share the batched panel force, Jacobian and torque response.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePreparedPanelVisibility.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dDirections (3, :) double
    strPanel (1, 1) struct
    bSelfShadowing (1, 1) logical = true
end

arguments (Output)
    dForcePerPressure (3, :) double
    dResponseJacobian (3, 3, :) double
    dTorquePerPressure (3, :) double
end

% Allocate only the requested derivative and torque paths.
ui32DirectionCount = uint32(size(dDirections, 2));
ui32FaceCount = uint32(size(strPanel.dQuadsNormals_SCB, 2));
dForcePerPressure = zeros(3, ui32DirectionCount);
if nargout >= 2
    dResponseJacobian = zeros(3, 3, ui32DirectionCount);
else
    dResponseJacobian = zeros(3, 3, 0);
end
if nargout >= 3
    dTorquePerPressure = zeros(3, ui32DirectionCount);
else
    dTorquePerPressure = zeros(3, 0);
end

% Match the established force law's normal normalization once per batch.
dNormalNorms = sqrt(sum(strPanel.dQuadsNormals_SCB.^2, 1));
dNormals = strPanel.dQuadsNormals_SCB ./ dNormalNorms;
for ui32Direction = uint32(1):ui32DirectionCount
    dDirectionNorm = norm(dDirections(:, ui32Direction));
    assert(isfinite(dDirectionNorm) && dDirectionNorm > eps, 'ComputePanelSrpResponse:ZeroDirection', ...
        'Supply nonzero Sun directions.');
    dSunDirection = dDirections(:, ui32Direction) / dDirectionNorm;
    dVisibility = ones(double(ui32FaceCount), 1);
    if bSelfShadowing && isfield(strPanel, 'strShadowData')
        strShadow = strPanel.strShadowData;
        if isfield(strShadow, 'strRayData')
            dVisibility = ComputePreparedPanelVisibility( ...
                dSunDirection, strPanel.dQuadsNormals_SCB, strShadow);
        else
            % Preserve legacy prepared samples while adopting cached geometry explicitly.
            dVisibility = ComputePanelSunVisibility(dSunDirection, ...
                strPanel.dQuadsNormals_SCB, strShadow.dSamplePoints_SCB, ...
                strShadow.dFaceVertices_SCB, strShadow.dRayOffset);
        end
    end

    dDirectionJacobian = zeros(3, 3);
    for ui32Face = uint32(1):ui32FaceCount
        dNormal = dNormals(:, ui32Face);
        dCosine = dot(dNormal, dSunDirection);
        dArea = strPanel.dSCquadsArea(ui32Face) * dVisibility(ui32Face);
        if dCosine <= 0 || dArea == 0
            continue
        end
        dDiffuse = strPanel.dDiffSpecQuadsCoeffs(ui32Face, 1);
        dSpecular = strPanel.dDiffSpecQuadsCoeffs(ui32Face, 2);
        dAbsorption = 1 - dSpecular;

        % Accumulate the same optical law without storing every face force.
        dFaceForce = -dArea * dCosine * ...
            (2 * (dDiffuse / 3 + dSpecular * dCosine)*dNormal + dAbsorption * dSunDirection);
        dForcePerPressure(:, ui32Direction) = ...
            dForcePerPressure(:, ui32Direction) + dFaceForce;
        if nargout >= 2
            dDirectionJacobian = dDirectionJacobian - dArea * ( ...
                (2*dDiffuse / 3 + 4*dSpecular * dCosine)*(dNormal * dNormal.') + ...
                dAbsorption * (dSunDirection * dNormal.' + dCosine * eye(3)));
        end
        if nargout >= 3
            dTorquePerPressure(:, ui32Direction) = dTorquePerPressure(:, ui32Direction) + ...
                cross(strPanel.dQuadsPressCentre_SCB(:, ui32Face), dFaceForce);
        end
    end
    if nargout >= 2
        % Include the supplied vector's normalization while freezing ray visibility.
        dResponseJacobian(:, :, ui32Direction) = dDirectionJacobian * ...
            (eye(3) - dSunDirection * dSunDirection.') / dDirectionNorm;
    end
end
end
