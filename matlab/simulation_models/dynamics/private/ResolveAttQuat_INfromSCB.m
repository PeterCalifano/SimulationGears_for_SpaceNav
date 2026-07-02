function dqSCBwrtIN = ResolveAttQuat_INfromSCB(strDynParams) %#codegen
arguments
    strDynParams (1,1) struct
end
%% PROTOTYPE
% dqSCBwrtIN = ResolveAttQuat_INfromSCB(strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Return the spacecraft body-to-inertial attitude quaternion stored in the dynamics payload.
% The identity attitude is used when the spacecraft data does not provide dqSCBwrtIN.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strDynParams: (1,1) struct  Dynamics payload with optional strSCdata.dqSCBwrtIN.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dqSCBwrtIN:   (4,1) double  Quaternion rotating SC body vectors into inertial frame (scalar first).
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract shared spacecraft attitude resolver.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
dqSCBwrtIN = [1; 0; 0; 0];

if coder.const(isfield(strDynParams.strSCdata, 'dqSCBwrtIN'))
    dqSCBwrtIN = strDynParams.strSCdata.dqSCBwrtIN(:);
end

end
