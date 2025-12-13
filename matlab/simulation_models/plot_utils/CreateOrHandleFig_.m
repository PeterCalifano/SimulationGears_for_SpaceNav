function [objFig, objSceneAx, ...
        bEnforcePlotOpts, charTextColor] = CreateOrHandleFig_(objFig, ...
                                                              charFigureRenderer, ...
                                                              bUseBlackBackground, ...
                                                              ui32FigureSeedID)
arguments
    objFig              {mustBeA(objFig, ["double", "matlab.ui.Figure", "matlab.graphics.Graphics", ...
                                                "matlab.graphics.GraphicsPlaceholder"])} = 0;
    charFigureRenderer  (1,:) string  {mustBeA(charFigureRenderer, ["string", "char"]), ...
                                    mustBeMember(charFigureRenderer, ["opengl", "painters"])} = "opengl"
    bUseBlackBackground (1,1) logical = false;
    ui32FigureSeedID    (1,1) uint32 = 0;
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

% Check validity
bValidFig = not(isempty(objFig)) && not(objFig == 0);

if bValidFig
    % Check validity
    bValidFig = isvalid(objFig) && isa(objFig, "matlab.ui.Figure") && ...
        not(isa(objFig, "matlab.graphics.GraphicsPlaceholder")) && ...
        (objFig.Number == ui32FigureSeedID || ui32FigureSeedID == 0);
end

% Construct or handle figure
if not(bValidFig)

    if nargin > 3
        objFig = figure(double(ui32FigureSeedID));
    else
        objFig = figure();
    end

    bEnforcePlotOpts = true; % No figure provided, enable plot opts

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
    objSceneAx = findall(objFig, 'Type', 'axes');

    % If axes is placeholder, create new one
    if any(isempty(objSceneAx)) || any( not(isvalid(objSceneAx)) ) || any(isa(objSceneAx, "matlab.graphics.GraphicsPlaceholder"))
        objSceneAx = axes(objFig);
        bEnforcePlotOpts = true; % No valid axes, enable plot opts
    end
end

% Set figure options
if nargin > 1 && bEnforcePlotOpts
    [~, charTextColor, ~] = DefaultPlotOpts(objFig, ...
                            "charRenderer", charFigureRenderer, ...
                            "bUseBlackBackground", bUseBlackBackground, ...
                            "bEnableGrid", false);
end

end
