function [o_dXSC_ref, o_dTimestamps, o_dqCAMwrtIN_ref, o_dqCAMwrtIN_est, objOdeSolution] = GenerateReferenceTraj(strScenConfig, bDEBUG_MODE)
arguments
    strScenConfig (1,1) {isstruct}
    bDEBUG_MODE    (1,1) logical = false
end
%% PROTOTYPE
% [o_dXSC_ref, o_dTimestamps, o_dqCAMwrtIN_ref, o_dqCAMwrtIN_est] = GenerateReferenceTraj(strScenConfig, bDEBUG_MODE);
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% NOTE: all the parameters the dynamics require must be "pre-loaded" in the function handle
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-05-2024        Pietro Califano         Function adapted from script.
% 08-07-2024        Pietro Califano         Update to support more complex dynamics
% 19-07-2024        Pietro Califano         Update of interfaces and processing
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

if not(isfield(strScenConfig, 'strAttitudeData'))
    existAttitudeData = false;
    warning("No attitudeData in strScenConfig: using default settings.")
end

% Convenience assignments
if isfield(strScenConfig.strInitConditions, 'dInitKeplParams') && not(isfield(strScenConfig.strInitConditions, 'xState0'))
    i_dNominalKeplParams = strScenConfig.strInitConditions.dInitKeplParams;
    i_dGravParamMain = strScenConfig.strInitConditions.dGravParam;

    assert(length(i_dNominalKeplParams) == 6);
    assert(length(i_dGravParamMain) == 1);

    % i_dNominalKeplParams = [SMA, ECC, INCL, RAAN, OMEGA, THETA];
    xSCref0 = kepl2rv(i_dNominalKeplParams, i_dGravParamMain);

elseif isfield(strScenConfig.strInitConditions, 'xState0')
    
    assert(length(strScenConfig.strInitConditions.xState0) == 6);
    xSCref0 = strScenConfig.strInitConditions.xState0(1:6);

else
    error('Neither Osculating Keplerian elements nor Cartesian coordinates have been provided.')
end

% NOTE: all the parameters the dynamics require must be "pre-loaded" in the function handle
%if isfield(strScenConfig, 'dynParams')
%    params = strScenConfig.dynParams;
%else
%    params = struct(); % TODO: there should a better way to pass parameters to the function?
%end


if isfield(strScenConfig, 'dSimOptions.odeopts') == true
    odeopts = strScenConfig.dSimOptions.odeopts;
else
    odeopts = odeset('RelTol', 1E-13, 'AbsTol', 1E-13);
end

if existAttitudeData && isfield(strScenConfig.dSimOptions, 'dSigmaAKE') == true 
    dSigmaAKE = strScenConfig.dSimOptions.dSigmaAKE;
else
    dSigmaAKE = 0;
end

if existAttitudeData && isfield(strScenConfig.dSimOptions, 'bIS_VSRPplus') == true
    bIS_VSRPplus = strScenConfig.dSimOptions.bIS_VSRPplus;
else
    bIS_VSRPplus = true;
end
if existAttitudeData && isfield(strScenConfig.dSimOptions, 'bIS_QLEFT_HANDED') == true
    bIS_QLEFT_HANDED = strScenConfig.dSimOptions.bIS_QLEFT_HANDED;
else
    bIS_QLEFT_HANDED = false;
end

% Assign minimal set of variables
dynFuncHandle = strScenConfig.dynFuncHandle;

%% ORBIT TRAJECTORY

if not(isfield(strScenConfig, 'dTimestamps'))
    tFinal        = round(strScenConfig.finalTime, 12);
    deltaTime     = strScenConfig.deltaTime;

    tspan = 0:deltaTime:tFinal;

    if tspan(end) ~= tFinal
        tspan(end+1) = tFinal;
    end

else
    tspan = strScenConfig.dTimestamps;
end

% Integrate dynamics
tic
objOdeSolution = ode113(@(time, xState) dynFuncHandle(time, xState), ...
    tspan, xSCref0, odeopts);
toc

o_dXSC_ref = deval(objOdeSolution, tspan)';
o_dTimestamps = tspan;

if bDEBUG_MODE
    % PLOT REFERENCE
    figure;
    plot3(o_dXSC_ref(:, 1), o_dXSC_ref(:, 2), o_dXSC_ref(:, 3), 'k-', 'LineWidth', 1.05);
    hold on;
    axis equal;
    grid on;
end

%% ATTITUDE TRAJECTORY
i_drTargetPoint_IN = [0;0;0];
i_dqCAMwrtSCB = [0; 0; 0; 1];
i_bINVERSE_Z_AXIS = false;

o_dqSCBwrtIN_ref = zeros(length(tspan), 4);
o_dqSCBwrtIN_est = zeros(length(tspan), 4);
o_dqCAMwrtIN_ref = zeros(length(tspan), 4);
o_dqCAMwrtIN_est = zeros(length(tspan), 4);

for idt = 1:length(tspan)

    [o_dqSCBwrtIN_ref(idt, :), o_dqSCBwrtIN_est(idt, :), ...
        o_dqCAMwrtIN_ref(idt, :), o_dqCAMwrtIN_est(idt, :)] = simulateTBPointing(o_dXSC_ref(idt, 1:6)', ...
        i_drTargetPoint_IN, ...
        i_dqCAMwrtSCB, ...
        dSigmaAKE, ...
        bIS_VSRPplus, ...
        i_bINVERSE_Z_AXIS, ...
        bIS_QLEFT_HANDED);

end

% Plot trajectory
% try
%     % TO REWORK
%     plotAttitudeQuat(o_dqCAMwrtIN_ref, ...
%         [0,0,0], ...
%         i_bIS_JPL_CONV, ...
%         i_bPlotFrame, ...
%         i_bAnimateFlag, ...
%         i_dPauseTime);
% catch
%     warning('Function plotAttitudeQuat() not found');
% 
%     figure;
%     plot3(0, 0, 0, 'ko', 'DisplayName', 'Origin');
%     try
%         DefaultPlotOpts();
%     catch
%         warning('Function DefaultPlotOpts() not found');
%     end
%     axis equal;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
% end


end
