classdef CBasePlotter < handle
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 13-02-2025    Pietro Califano     First base implementation
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % Method1: Description
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % Property1: Description, dtype, nominal size
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = protected, GetAccess = public)
        objFigureArray {mustBeA(objFigureArray, ["matlab.ui.Figure", "GraphicsPlaceholder"])} = gobjects(1,1)

        % Default plotting options
    end


    methods (Access = public)
        % CONSTRUCTOR
        function self = CBasePlotter()
            arguments
            end


        end

        % GETTERS

        % SETTERS

        % METHODS
    end


    methods (Access = protected)


    end

    % methods (Access=private)
    %
    % end

    % methods (Access=public, Static)
    %
    % end

    % methods (Abstract, Access=public)
    % [x,y] = abstract_function_name(args) NOTE: number of args matter.
    % end

    methods (Static, Access = public)

        function [objFig, charTextColor, charBackGroundColor] = DefaultPlotOpts(objFig, kwargs)
            arguments
                objFig = ""
            end
            arguments
                kwargs.bUseBlackBackground  (1,1) logical {islogical, isscalar} = false;
                kwargs.charRenderer         (1,:) string {mustBeA(kwargs.charRenderer, ["string", "char"])} = 'painters'; % 'opengl'
            end
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

            % Set background color based on flag
            if kwargs.bUseBlackBackground == true
                % Get figure handle

                % charRenderer = 'opengl';
                charTextColor       = 'w'; % White text
                charBackGroundColor = 'k';
            else

                set(gca, 'Color', 'w'); % White background
                set(gcf, 'Color', 'w');
                charTextColor       = 'k'; % Black text
                charBackGroundColor = 'w';

            end

            if nargin == 0
                % Get figure and axis handles
                objFig = gcf;

            else

                % Assert input type
                mustBeA(objFig, "matlab.ui.Figure");

            end

            objCurrentAx = gca;
            % Apply options
            CBasePlotter.ApplyOptions_(objFig, objCurrentAx, kwargs.charRenderer, charTextColor, charBackGroundColor);

        end


        function [dEntitiesColorArray] = getRandomizedColormap(handleColorMapFcn, dNumOfEntries)
            % Generate colors using colormap specified by handleColorMapFcn

            % Generates a colormap with `ui32NumOfEntities` distinct randomized colors
            dEntitiesColorArray = handleColorMapFcn(1e4 * dNumOfEntries);
            dEntitiesColorArray = dEntitiesColorArray(randperm(size(dEntitiesColorArray, 1), 3*dNumOfEntities), :);

        end

        function [cellInput] = appendObjToCell(inputInstance, cellInput)
            arguments
                inputInstance   (1,:) {mustBeA(inputInstance, ["cell", "matlab.graphics.Graphics"])} 
                cellInput       (1,:) cell {mustBeA(cellInput, "cell")} = {}
            end
            
            % Wrap in cell if not already a cell
            if not(isa(inputInstance, "cell"))
                inputInstance = {inputInstance};
            end

            cellInput = [cellInput(:)', inputInstance(:)];

        end

        function [cellPlotObjs] = initializeLegendCell(bEnableLegend)

            % Define cell of plot objects for legend
            if bEnableLegend
                % If legend is enabled, first get objects already inserted
                % TODO: get previous legend entries if any
                warning('Branch bEnableLegend = true not yet completed! cellPlotObjs defaulted to empty {}')
                cellPlotObjs = {};

            else
                % Else, store objects for external use
                cellPlotObjs = {};
            end

        end

        function [objSceneFig, charTextColor, charBackGroundColor] = initializeFigureObj(objSceneFig, kwargs)
            arguments
                objSceneFig           (1,1) {isscalar, mustBeA(objSceneFig, ["double", "matlab.ui.Figure"])} = 0;
                kwargs.charRenderer          (1,:) string {mustBeA(kwargs.charRenderer, ["string", "char"])} = 'opengl'
                kwargs.bUseBlackBackground   (1,1) logical {islogical, isscalar} = false;
            end

            if objSceneFig == 0

                objSceneFig = figure('Renderer', kwargs.charRenderer);
                kwargs.bEnforcePlotOpts = true; % No figure provided, enable plot opts
                [~, charTextColor, charBackGroundColor] = CBasePlotter.DefaultPlotOpts(objSceneFig, ...
                        "charRenderer", kwargs.charRenderer, ...
                        "bUseBlackBackground", kwargs.bUseBlackBackground);

            else
                objSceneFig = kwargs.objSceneFig;
                charTextColor = objSceneFig.Color;
            end
        end

    end


    methods (Static, Access = protected)
        % Internal implementation
        function ApplyOptions_(objFig, objCurrentAx, charRenderer, charTextColor, charBackGroundColor)

            % Recall figure
            figure(objFig);

            % Set renderer
            set(objFig, 'Renderer', charRenderer);

            % Apply plot options
            grid minor
            axis auto;

            objCurrentAx.XAxisLocation = 'bottom';
            objCurrentAx.YAxisLocation = 'left';
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

    end


end


