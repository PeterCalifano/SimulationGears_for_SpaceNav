function [dDxDt, strAccelInfo, dDynMatrix] = ComputeRefDynFcn(dStateTimetag, ...
                                                               dxState, ...
                                                               strDynParams) %#codegen
arguments
    dStateTimetag   (1,1) double {mustBeNumeric, mustBeNonnegative}
    dxState         (:,1) double {mustBeNumeric}
    strDynParams    (1,1) struct
end
%% PROTOTYPE
% [dDxDt, strAccelInfo, dDynMatrix] = ComputeRefDynFcn(dStateTimetag, dxState, strDynParams) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Reference orbit dynamics entry point for simulations and ground-truth propagation. The first six state
% entries are inertial position and velocity. If dxState also carries a 6x6 STM flattened after the state
% (42 states total), the function propagates the STM with the same Jacobian returned as optional output 3.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dStateTimetag   (1,1) double   Evaluation time.
% dxState         (:,1) double   [r; v] or [r; v; Phi(:)].
% strDynParams    (1,1) struct   Dynamics environment assembled by DefineEnvironmentProperties or equivalent.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDxDt           (:,1) double   State derivative, with STM derivative appended when input carries STM.
% strAccelInfo    (1,1) struct   Acceleration components and cached SRP/polyhedron quantities.
% dDynMatrix      (6,6) double   Optional orbit-state Jacobian.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5      Upgrade reference dynamics entry point with optional
%                                               Jacobian/STM and polyhedron perturbation support.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% evalRHS_InertialDynMaxFidelity()
% evalJac_InertialDynMaxFidelity()
% -------------------------------------------------------------------------------------------------------------

%% Function code
assert(numel(dxState) == 6 || numel(dxState) == 42, ...
    'ComputeRefDynFcn:InvalidStateSize', ...
    'dxState must contain either a 6-state orbit state or a 6-state orbit state plus a flattened 6x6 STM.');

bPropagateSTM = numel(dxState) == 42;
dxOrbitState = dxState(1:6);

[dDxDtOrbit, strAccelInfo] = evalRHS_InertialDynMaxFidelity(dStateTimetag, dxOrbitState, strDynParams);

dDynMatrix = zeros(6, 6);
if nargout > 2 || bPropagateSTM
    dDynMatrix = evalJac_InertialDynMaxFidelity(dStateTimetag, dxOrbitState, strDynParams, struct(), strAccelInfo);
end

if bPropagateSTM
    dPhi = reshape(dxState(7:42), 6, 6);
    dDxDt = [dDxDtOrbit; reshape(dDynMatrix * dPhi, 36, 1)];
else
    dDxDt = dDxDtOrbit;
end

end
