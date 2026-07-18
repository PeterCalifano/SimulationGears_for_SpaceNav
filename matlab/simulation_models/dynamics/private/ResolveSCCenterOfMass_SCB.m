function dCoMpos_SCB = ResolveSCCenterOfMass_SCB(strDynParams) %#codegen
arguments
    strDynParams (1,1) struct
end
%% PROTOTYPE
% dCoMpos_SCB = ResolveSCCenterOfMass_SCB(strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Return the spacecraft center-of-mass position in spacecraft body coordinates.
% The origin is used when the spacecraft data does not provide dCoMpos_SCB.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strDynParams:  (1,1) struct   Dynamics payload with optional strSCdata.dCoMpos_SCB.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dCoMpos_SCB:   (3,1) double   Spacecraft center-of-mass position in SC body frame [panel length unit].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract shared spacecraft center-of-mass resolver.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
dCoMpos_SCB = zeros(3, 1);

if coder.const(isfield(strDynParams.strSCdata, 'dCoMpos_SCB'))
    dCoMpos_SCB = strDynParams.strSCdata.dCoMpos_SCB(:);
end

end
