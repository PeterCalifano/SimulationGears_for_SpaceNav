function [dForce, dVisibility, dTorque] = ComputeSrpLutGrid( ...
    dDirections, strPanel, bSelfShadowing, bReuseVisibility, dVisibility) %#codegen
%% SIGNATURE
% [dForce, dVisibility, dTorque] = ComputeSrpLutGrid(dDirections, strPanel, ...
%     bSelfShadowing, bReuseVisibility, dVisibility)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate a fixed batch of LUT directions with the owning panel law. Return
% visibility separately so optical-only samples can reuse geometric ray work.
% Keep validation, hashing, file access and packing in the host builder.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dDirections     (3,K) Nonzero spacecraft-to-Sun directions.
% strPanel        Prepared numeric SI panel and shadow geometry.
% bSelfShadowing  Select the panel model's shadow policy.
% bReuseVisibility Reuse the supplied fractions rather than tracing rays.
% dVisibility     (N,K) Input workspace and optional cached visibility.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dForce          (3,K) Force divided by solar pressure [m^2].
% dVisibility     (N,K) Direct visibility fractions.
% dTorque         (3,K) Optional torque/pressure about the body origin [m^3].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Share complete MATLAB/MEX numerical LUT construction.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePreparedPanelVisibility, ComputePanelSrpResponse.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dDirections (3,:) double
    strPanel (1,1) struct
    bSelfShadowing (1,1) logical
    bReuseVisibility (1,1) logical
    dVisibility (:,:) double
end
arguments (Output)
    dForce (3,:) double
    dVisibility (:,:) double
    dTorque (3,:) double
end

% Populate the reusable geometry response only on a cold geometry preparation.
if ~bReuseVisibility
    if bSelfShadowing
        for ui32Direction = uint32(1):uint32(size(dDirections,2))
            dVisibility(:,ui32Direction) = ComputePreparedPanelVisibility( ...
                dDirections(:,ui32Direction),strPanel.dQuadsNormals_SCB,strPanel.strShadowData);
        end
    else
        dVisibility(:) = 1;
    end
end

% The normal LUT path requests force alone; truth can additionally preserve torque.
if nargout >= 3
    [dForce,~,dTorque] = ComputePanelSrpResponse( ...
        dDirections,strPanel,false,dVisibility,false);
else
    dForce = ComputePanelSrpResponse(dDirections,strPanel,false,dVisibility);
    dTorque = zeros(3,0);
end
end
