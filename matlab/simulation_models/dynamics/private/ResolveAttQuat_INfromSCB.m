function dQuat_INfromSCB = ResolveAttQuat_INfromSCB(strDynParams) %#codegen
arguments
    strDynParams (1,1) struct
end
%% PROTOTYPE
% dQuat_INfromSCB = ResolveAttQuat_INfromSCB(strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Return the spacecraft body-to-inertial attitude quaternion stored in the dynamics payload.
% The identity attitude is used when the spacecraft data does not provide dQuat_INfromSCB.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strDynParams: (1,1) struct  Dynamics payload with optional strSCdata.dQuat_INfromSCB.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dQuat_INfromSCB:   (4,1) double  Quaternion rotating SC body vectors into inertial frame (scalar first).
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract shared spacecraft attitude resolver.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
dQuat_INfromSCB = [1; 0; 0; 0];

if coder.const(isfield(strDynParams.strSCdata, 'dQuat_INfromSCB'))
    dQuat_INfromSCB = strDynParams.strSCdata.dQuat_INfromSCB(:);
end

end
