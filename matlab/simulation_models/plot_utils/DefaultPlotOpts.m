function [objFig, charTextColor, charBackGroundColor] = DefaultPlotOpts(objFig, kwargs)
arguments
    objFig = ""
end
arguments
    kwargs.bUseBlackBackground  (1,1) logical {islogical, isscalar} = false;
    kwargs.charRenderer         (1,:) string {mustBeA(kwargs.charRenderer, ["string", "char"])} = 'painters'; % 'opengl'
end
% TODO: move this function to SimulationGears_for_SpaceNav!
%% PROTOTYPE
% DefaultPlotOpts(fig_in) 
% -------------------------------------------------------------------------
%% DESCRIPTION
% "Void" function applying default plot options for axis width, ticks and 
% grid to the currently active figure if no input is given. The options are
% applied to the figure given as input if nargin > 0. This is useful
% specifically when multiple figure objects are handles automatically.
% -------------------------------------------------------------------------
% INPUT
% fig_in: [fig object] Optional input. Figure object to which the options
%         are applied. It may be different from the currently active figure. 
% -------------------------------------------------------------------------
% OUTPUT
% [-]
% -------------------------------------------------------------------------
% CHANGELOG
%    18-04-2023    Pietro Califano    Coded and tested
% -------------------------------------------------------------------------
% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------
% Future upgrades
% 1) Check to get currently set options and avoid overwriting
% -------------------------------------------------------------------------

%% Function code

if nargin == 0
    % Get figure and axis handles
    objFig = gcf;
else
    % Assert input type
    mustBeA(objFig, "matlab.ui.Figure");
end

% Set background color based on flag
if nargin == 1 && kwargs.bUseBlackBackground == false

    charBackGroundColor = objFig.Color;

    if all(charBackGroundColor == 0)
        charTextColor       = "w";
    end

elseif kwargs.bUseBlackBackground == true
   
    % charRenderer = 'opengl';
    charTextColor       = 'w'; % White text
    charBackGroundColor = 'k';
    
elseif kwargs.bUseBlackBackground == false

    set(gca, 'Color', 'w'); % White background
    set(gcf, 'Color', 'w');
    
    charTextColor       = 'k'; % Black text
    charBackGroundColor = 'w';

    set(gca, 'Color', 'w'); % White background
    set(gcf, 'Color', 'w');

end

objCurrentAx = gca;
% Apply options
ApplyOptions(objFig, objCurrentAx, kwargs.charRenderer, charTextColor, charBackGroundColor);


end

%% LOCAL FUNCTION
function ApplyOptions(objFig, objCurrentAx, charRenderer, charTextColor, charBackGroundColor)

% Recall figure
figure(objFig);

% Set renderer
set(objFig, 'Renderer', charRenderer);

% Apply plot options
grid minor
axis auto;

objCurrentAx.XAxisLocation = 'bottom';
try
    objCurrentAx.YAxisLocation = 'left';
catch
end

objCurrentAx.XMinorTick = 'on';
objCurrentAx.YMinorTick = 'on';
objCurrentAx.LineWidth = 1.05;
objCurrentAx.LineWidth = 1.05;
ylim('tickaligned');
xlim('tight')

% Set background and text colours
set(objCurrentAx, 'Color', charBackGroundColor); % White background
set(gcf, 'Color', charBackGroundColor);

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




