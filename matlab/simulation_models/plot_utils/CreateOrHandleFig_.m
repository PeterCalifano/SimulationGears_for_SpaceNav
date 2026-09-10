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
% [objFig, objSceneAx, bEnforcePlotOpts, charTextColor] = CreateOrHandleFig_( ...
%     objFig, charFigureRenderer, bUseBlackBackground, ui32FigureSeedID)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve a reusable figure and its axes, or create them when no valid figure exists. A numbered figure that already
% exists is resolved without activating or showing it, preserving non-interactive plotting state.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objFig                 Candidate figure handle or graphics placeholder.
% charFigureRenderer     Renderer applied when a new figure is configured.
% bUseBlackBackground    Whether new-figure defaults use a black background.
% ui32FigureSeedID       Preferred figure number; zero leaves numbering to MATLAB when omitted.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objFig                 Resolved or newly created figure.
% objSceneAx             Existing axes in the figure, or a newly created axis.
% bEnforcePlotOpts       True when this function created figure content requiring default plot options.
% charTextColor          Text color selected by the existing or newly applied background policy.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 09-12-2025    Pietro Califano     Implement first version from existing code
% 04-08-2026    Pietro Califano, Codex gpt 5.6     Preserve graphics state when resolving a numbered figure.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% DefaultPlotOpts
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Define defaults
bEnforcePlotOpts = false;

% Validate a caller-supplied figure before considering number-based resolution.
bValidFig = not(isempty(objFig)) && not(objFig == 0);

if bValidFig
    % Check validity
    bValidFig = isvalid(objFig) && isa(objFig, "matlab.ui.Figure") && ...
        not(isa(objFig, "matlab.graphics.GraphicsPlaceholder")) && ...
        (objFig.Number == ui32FigureSeedID || ui32FigureSeedID == 0);
end

% Resolve an existing numbered figure without the activation and visibility side effects of figure(seedID).
if not(bValidFig) && nargin > 3 && ui32FigureSeedID > uint32(0)
    objMatchingFigures = findall(groot, 'Type', 'figure', 'Number', double(ui32FigureSeedID));
    if not(isempty(objMatchingFigures))
        objFig = objMatchingFigures(1);
        bValidFig = true;
    end
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
