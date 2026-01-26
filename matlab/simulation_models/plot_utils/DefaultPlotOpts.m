function [objFig, charTextColor, charBackGroundColor] = DefaultPlotOpts(objFig, kwargs)
arguments
    objFig {mustBeA(objFig, ["matlab.ui.Figure", "double"])} = []
end
arguments
    kwargs.bUseBlackBackground  (1,1) logical = false;
    kwargs.charRenderer         (1,:) string {mustBeA(kwargs.charRenderer, ["string", "char"])} = 'painters'; % 'opengl'
    kwargs.bEnableGrid          (1,1) logical = true
end
% TODO: move this function to EvalAndVisualization toolbox
%% PROTOTYPE
% DefaultPlotOpts(objFig) 
% ---------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% "Void" function applying default plot options for axis width, ticks and grid to the currently active 
% figure if no input is given. The options are applied to the figure given as input if nargin > 0. 
% This is useful specifically when multiple figure objects are handles automatically.
% ---------------------------------------------------------------------------------------------------------
% INPUT
% objFig: [fig object] Optional input. Figure object to which the options
%         are applied. It may be different from the currently active figure. 
% ---------------------------------------------------------------------------------------------------------
% OUTPUT
% [-]
% ---------------------------------------------------------------------------------------------------------
% CHANGELOG
% 18-04-2023    Pietro Califano     Coded and tested
% 11-07-2025    Pietro Califano     Upgrade for usage in Gears frameworks
% 11-12-2025    Pietro Califano     Upgrade to support usage of target axes from handles
% ---------------------------------------------------------------------------------------------------------
% DEPENDENCIES
% [-]
% ---------------------------------------------------------------------------------------------------------

%% Function code

if nargin == 0
    % Get figure and axis handles
    objFig = gcf();
else
    % Assert input type
    mustBeA(objFig, "matlab.ui.Figure");
end

% Set background color based on flag
if nargin == 1 && kwargs.bUseBlackBackground == false
    charBackGroundColor = objFig.Color;

    if all(charBackGroundColor == 0)
        charTextColor       = "w";
    else
        charTextColor       = "k";
    end

elseif kwargs.bUseBlackBackground == true
   
    % charRenderer = 'opengl';
    charTextColor       = 'w'; % White text
    charBackGroundColor = 'k';
    
elseif kwargs.bUseBlackBackground == false

    charTextColor       = 'k'; % Black text
    charBackGroundColor = 'w';

end

objCurrentAxes = findall(objFig, 'Type', 'axes');

for objAx = transpose(objCurrentAxes)
    set(objAx, 'Color', charBackGroundColor); % White background
    set(objFig, 'Color', charBackGroundColor);

    % Apply options
    ApplyOptions(objFig, objAx, kwargs.charRenderer, charTextColor, charBackGroundColor, kwargs.bEnableGrid);
end

end

%% LOCAL FUNCTION
function ApplyOptions(objFig, objCurrentAx, charRenderer, charTextColor, charBackGroundColor, bEnableGrid)
arguments
    objFig
    objCurrentAx
    charRenderer
    charTextColor
    charBackGroundColor
    bEnableGrid (1,1) logical = true;
end

% Set renderer
set(objFig, 'Renderer', charRenderer);

% Apply plot options
if bEnableGrid
    grid(objCurrentAx, "minor")
    axis(objCurrentAx, "auto");
end

objCurrentAx.XAxisLocation = 'bottom';
try
    objCurrentAx.YAxisLocation = 'left';
catch
end

objCurrentAx.XMinorTick = 'on';
objCurrentAx.YMinorTick = 'on';
objCurrentAx.LineWidth = 1.05;
ylim(objCurrentAx, 'tickaligned');
xlim(objCurrentAx, 'tight')

% Set background and text colours
set(objCurrentAx, 'Color', charBackGroundColor); % White background
set(objFig, 'Color', charBackGroundColor);

% Set legend color to required one
objLegends = findobj(objFig, 'Type', 'Legend');

if ~isempty(objLegends)

    for idL = 1:numel(objLegends)
        objLegends(idL).Color     = charBackGroundColor;
        objLegends(idL).EdgeColor = charTextColor;
        objLegends(idL).TextColor = charTextColor;
    end
end

% Set axis colours
objLabelHandle = objCurrentAx.XLabel;
objLabelHandle.Color = charTextColor;

objLabelHandle = objCurrentAx.YLabel;
objLabelHandle.Color = charTextColor;

objLabelHandle = objCurrentAx.ZLabel;
objLabelHandle.Color = charTextColor;

% Set title color
objTitleHandle = objCurrentAx.Title;
objTitleHandle.Color = charTextColor;

set(objCurrentAx, 'XColor', charTextColor, 'YColor', charTextColor, 'ZColor', charTextColor);


end




