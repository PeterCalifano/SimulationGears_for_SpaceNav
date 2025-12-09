function [objFig, objSceneAx, ...
        bEnforcePlotOpts, charTextColor] = CreateOrHandleFig_(objFig, ...
                                                              charFigureRenderer, ...
                                                              bUseBlackBackground)
arguments
    objFig              (1,1) {mustBeA(objFig, ["double", "matlab.ui.Figure"])} = 0;
    charFigureRenderer  (1,:) string  {mustBeA(charFigureRenderer, ["string", "char"]), ...
                                    mustBeMember(charFigureRenderer, ["opengl", "painters"])} = "opengl"
    bUseBlackBackground (1,1) logical = false;
end
%% SIGNATURE
% TODO
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% 
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% 
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% 
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 09-12-2025    Pietro Califano     Implement first version from existing code
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Define defaults
bEnforcePlotOpts = false;

% Construct or handle figure
if objFig == 0
    objFig = figure();
    bEnforcePlotOpts = true; % No figure provided, enable plot opts
    [~, charTextColor, ~] = DefaultPlotOpts(objFig, ...
                            "charRenderer", charFigureRenderer, ...
                            "bUseBlackBackground", bUseBlackBackground);
    % Create new axis
    objSceneAx = axes(objFig);
else
    
    if bUseBlackBackground
        charTextColor = "k";
    else
        charTextColor = "w"; 
    end
    % Get axes
    assert(isvalid(objFig), 'ERROR: figure handle is invalid!')
    objSceneAx = get(objFig, "CurrentAxes");
    
    % If axes is placeholder, create new one
    if not(isvalid(objSceneAx)) || isa(objSceneAx, "matlab.graphics.axis.AxesPlaceholder")
        objSceneAx = axes(objFig);
        bEnforcePlotOpts = true; % No valid axes, enable plot opts
    end
end

end