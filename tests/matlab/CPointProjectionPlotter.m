classdef CPointProjectionPlotter < CBasePlotter
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 13-02-2025    Pietro Califano     First implementation.
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

    end


    methods (Access = public)
        % CONSTRUCTOR
        function self = CPointProjectionPlotter()
            arguments
            end


        end

        % GETTERS

        % SETTERS

        % METHODS
    end


    methods (Access = protected)


    end

    methods (Static, Access = public)
        function [objSceneFig, cellPlotObjs] = PlotProjectedPoints(cellProjectedPointsArray_UV, ...
                objCameraParams, ...
                dSunPosition_CamFrame, ...
                kwargs)

            arguments (Input)
                cellProjectedPointsArray_UV  (1,:) {mustBeA(cellProjectedPointsArray_UV, ["cell", "float"])}
                objCameraParams              (1,1) {mustBeA(objCameraParams, ["CProjectiveCamera", "CCameraIntrinsics", "cameraIntrinsics"])}
                dSunPosition_CamFrame        (3,1) double {mustBeVector, mustBeNumeric} = [0;0;0]
            end
            arguments (Input)
                kwargs.bUseBlackBackground   (1,1) logical {islogical, isscalar} = false;
                kwargs.cellPlotColors        (1,:) cell = {};
                kwargs.cellPlotNames         (1,:) cell = {};
                kwargs.cellMarkersTypes      (1,:) cell = {}; % TODO
                kwargs.objSceneFig           (1,1) {isscalar, mustBeA(kwargs.objSceneFig, ["double", "matlab.ui.Figure"])} = 0;
                kwargs.charFigTitle          (1,:) string {mustBeA(kwargs.charFigTitle, ["string", "char"])} = 'Projected 2D visible points in Image Plane'
                kwargs.charRenderer          (1,:) string {mustBeA(kwargs.charRenderer, ["string", "char"])} = 'opengl'
                kwargs.bEnforcePlotOpts      (1,1) logical {islogical, isscalar} = false
                kwargs.bEnableLegend         (1,1) logical {islogical, isscalar} = true  
            end

            % Get figure and properties
            [objSceneFig, charTextColor, charBackGroundColor] = CBasePlotter.initializeFigureObj(kwargs.objSceneFig, ...
                "bUseBlackBackground", kwargs.bUseBlackBackground, ...
                "charRenderer", kwargs.charRenderer);

            [cellPlotObjs] = CBasePlotter.initializeLegendCell(kwargs.bEnableLegend);

            % Get number of sets to plot
            ui32NumOfSets = length(cellProjectedPointsArray_UV);
            
            % Determine name cells
            % TODO
            if isempty(kwargs.cellPlotNames)
                cellPlotNames = cell(1, ui32NumOfSets);

                % Default names for entries
                for idE = 1:dNumOfEntities
                    cellPlotNames(idE) = { sprintf("Points set %d", idE) };
                end
            else
                assert(length(kwargs.cellPlotNames) == ui32NumOfSets, 'ERROR: you need to specify a name for each set if provided.')
                cellPlotNames = kwargs.cellPlotNames;
            end

            % Determine colors cell % TODO this may be moved to bae plotter and be a static method?
            if not(isempty(kwargs.cellPlotColors)) == true

                assert(length(kwargs.cellPlotColors) == ui32NumOfSets, 'ERROR: you need to specify a color for each set if provided.')
                cellPlotColors = kwargs.cellPlotColors;

            elseif isempty(kwargs.cellPlotColors) && ui32NumOfSets > 3
                % Generate random colors
                [dEntitiesColorArray] = getRandomizedColormap(handleColorMapFcn, double(ui32NumOfSets));

                % Allocate in cell % TODO: this is just for simplicity of input + single interface, but clearly inefficient
                cellPlotColors = cell(ui32NumOfSets, 1);

                for idE = 1:ui32NumOfSets
                    cellPlotColors{idE, 1} = dEntitiesColorArray(idE, :);
                end

            elseif isempty(kwargs.cellPlotColors) && ui32NumOfSets <= 3
                % Set manually defined and nice looking colors
                cellPlotColors = {charTextColor, 'b', '#FFA500'};
            else
                error('Undefined case. There may be some issue with the provided inputs and/or the conditions. Please check code.')
            end

            % Refocus figure
            figure(objSceneFig);
            set(gca, 'YDir', 'reverse');

            % Detector rectangle (sensor boundaries)
            % Draw the detector rectangle using `rectangle()`
            objCamDetector = rectangle('Position', [0, 0, objCameraParams.ImageSize(1), objCameraParams.ImageSize(2)], ...
                                        'EdgeColor', 'r', 'LineWidth', 2); %#ok<NASGU>


            % Plot points set
            for idS = 1:ui32NumOfSets

                dProjectedPointsArray_UV = cellProjectedPointsArray_UV{idS};

                if size(dProjectedPointsArray_UV, 1) ~= 2
                    assert(size(dProjectedPointsArray_UV, 2) == 2, "ERROR: Invalid shape. Either the num. of columns or of the rows must be = 2.")
                    dProjectedPointsArray_UV = dProjectedPointsArray_UV';
                end
                hold on;

                objPlot = plot(dProjectedPointsArray_UV(2, :), dProjectedPointsArray_UV(1, :), ...
                                'Marker', '.', ...
                                'Color', cellPlotColors{idS}, ...
                                'DisplayName', cellPlotNames{idS}, ...
                                'Linestyle', 'none');

                [cellPlotObjs] = CBasePlotter.appendObjToCell(objPlot, cellPlotObjs);
            end

            % Apply options
            xlabel('u [pix]', 'Color', charTextColor); 
            ylabel('v [pix]', 'Color', charTextColor);
            title('Projected 2D Mesh in Image Plane', 'Color', 'w');


            if kwargs.objSceneFig == 0 && kwargs.bEnforcePlotOpts
                objCurrentAx = gca;
                CBasePlotter.ApplyOptions_(objSceneFig, objCurrentAx, kwargs.charRenderer, charTextColor, charBackGroundColor);
            end

            if any(dSunPosition_CamFrame > 0)
                % TODO project and plot Sun line in image plane if provided

            end

            if kwargs.bEnableLegend
                % Add legend if not empty
                legend([cellPlotObjs{:}], 'TextColor', charTextColor);
            end

        end

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
end


