function dVisibleAreas = ComputePanelVisibleAreas(dAreas, dSunDir_SCB, strPanel) %#codegen
%% SIGNATURE
% dVisibleAreas = ComputePanelVisibleAreas(dAreas, dSunDir_SCB, strPanel)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply prepared equal-area self-shadow fractions to SI panel areas. Reuse this
% step in force and position-Jacobian evaluation so both use the same active
% optical faces. Retain unshadowed areas when the optional numeric shadow
% section is absent. Keep its geometry and ray offset in one length unit;
% visibility fractions are independent of that unit.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dAreas         (N,1) Panel areas normalized to SI [m^2].
% dSunDir_SCB    (3,1) Spacecraft-to-Sun direction in the panel frame [-].
% strPanel       Panel data with optional strShadowData containing
%                dFaceVertices_SCB (3,3,N), dSamplePoints_SCB (3,Q,N), and
%                a positive dRayOffset, all in the geometry's length unit.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVisibleAreas  (N,1) Directly illuminated effective areas [m^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 04-10-2026  Pietro Califano, Codex GPT-6  Share prepared shadowing in truth dynamics.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePanelSunVisibility.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dAreas double {mustBeVector}
    dSunDir_SCB (3, 1) double
    strPanel (1, 1) struct
end

arguments (Output)
    dVisibleAreas (:, 1) double
end

% Specialize legacy callers without transporting or constructing shadow data.
dVisibleAreas = dAreas(:);
if coder.const(isfield(strPanel, 'strShadowData'))
    strShadow = strPanel.strShadowData;
    dVisibility = ComputePanelSunVisibility(dSunDir_SCB, strPanel.dQuadsNormals_SCB, ...
        strShadow.dSamplePoints_SCB, strShadow.dFaceVertices_SCB, strShadow.dRayOffset);
    dVisibleAreas = dVisibleAreas .* dVisibility;
end

end
