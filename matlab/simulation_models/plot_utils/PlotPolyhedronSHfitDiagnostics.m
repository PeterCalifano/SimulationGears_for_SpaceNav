function strFigureHandles = PlotPolyhedronSHfitDiagnostics( ...
    strShapeModel, strSHgravityData, strDiagnostics, options)
arguments
    strShapeModel                    (1,1) struct
    strSHgravityData                 (1,1) struct
    strDiagnostics                   (1,1) struct
    options.bShowMeshFigure          (1,1) logical = true
    options.bShowConvergenceFigure   (1,1) logical = true
    options.bShowHoldoutFigure       (1,1) logical = true
    options.charFigureRenderer       (1,:) string {mustBeA(options.charFigureRenderer, ["string", "char"])} = "opengl"
    options.charDistanceUnit         (1,:) string {mustBeA(options.charDistanceUnit, ["string", "char"])} = "m"
    options.bUseBlackBackground      (1,1) logical = false
end
%% PROTOTYPE
% strFigureHandles = PlotPolyhedronSHfitDiagnostics( ...
%     strShapeModel, strSHgravityData, strDiagnostics, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Creates diagnostic figures for the polyhedron-to-spherical-harmonics fit:
% 1) input mesh visualization,
% 2) adaptive fit convergence history,
% 3) holdout error visualization against the exact polyhedron field.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strShapeModel:                  struct    Shape struct with fields
%                                           dVerticesPos [3 x Nv] and
%                                           ui32triangVertexPtr [3 x Nf].
% strSHgravityData:               struct    Fitted SH gravity data.
% strDiagnostics:                 struct    Output of
%                                           ComputePolyhedronGravitySHfitDiagnostics().
% options.bShowMeshFigure:        [1]       Toggle mesh figure.
% options.bShowConvergenceFigure: [1]       Toggle convergence history figure.
% options.bShowHoldoutFigure:     [1]       Toggle holdout comparison figure.
% options.charFigureRenderer:     [1]       MATLAB renderer to use.
% options.charDistanceUnit:       [1]       Distance unit label.
% options.bUseBlackBackground:    [1]       Use repo black-background figure style.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strFigureHandles: struct with figure handles for the generated figures.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Add reusable visualization utility for SH-fit diagnostics.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Visualize3dShapeModelWithPC()
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Input validation
strFigureHandles = struct();
strFigureHandles.objMeshFig = gobjects(1,1);
strFigureHandles.objConvergenceFig = gobjects(1,1);
strFigureHandles.objHoldoutFig = gobjects(1,1);

% Show mesh if enabled
if options.bShowMeshFigure
    [objMeshFig, ~] = Visualize3dShapeModelWithPC(strShapeModel, ...
                                                charFigureRenderer=options.charFigureRenderer, ...
                                                charDistanceUnit=options.charDistanceUnit, ...
                                                charPathDisplayName="Input shape model", ...
                                                bShowAsWireframe=false, ...
                                                bEnableLegend=false, ...
                                                dFaceAlpha=0.92, ...
                                                bUseBlackBackground=options.bUseBlackBackground, ...
                                                bEnforcePlotOpts=true);
    objMeshAx = get(objMeshFig, "CurrentAxes");
    title(objMeshAx, sprintf('Input Mesh and Expansion Radius (R_{ref} = %.6g)', ...
        strSHgravityData.dBodyRadiusRef));
    strFigureHandles.objMeshFig = objMeshFig;
end

% Show convergence progression at each iteration and holdout diagnostics if enabled
if options.bShowConvergenceFigure

    objConvergenceFig = figure('Renderer', char(options.charFigureRenderer), ...
        'Name', 'Polyhedron-to-SH Fit Convergence');
    tiledlayout(objConvergenceFig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    dIterations = 1:double(strSHgravityData.strFitStats.ui32NumIterations);

    nexttile;
    semilogy(dIterations, strSHgravityData.strFitStats.dTrainAccRMSrelHistory, 'o-', 'LineWidth', 1.1);
    hold on
    semilogy(dIterations, strSHgravityData.strFitStats.dValidationAccRMSrelHistory, 's-', 'LineWidth', 1.1);
    semilogy(dIterations, strSHgravityData.strFitStats.dTrainPotRMSrelHistory, '^-', 'LineWidth', 1.1);
    semilogy(dIterations, strSHgravityData.strFitStats.dValidationPotRMSrelHistory, 'd-', 'LineWidth', 1.1);
    grid on
    xlabel('Iteration')
    ylabel('Relative RMS error')
    title('Train / Validation RMS History')
    legend({'Train acc', 'Validation acc', 'Train potential', 'Validation potential'}, ...
        'Location', 'best');

    nexttile;
    semilogy(dIterations, max(strSHgravityData.strFitStats.dCoeffRelChangeHistory, eps(1.0)), 'o-', 'LineWidth', 1.1);
    hold on
    semilogy(dIterations, strSHgravityData.strFitStats.dConditionNumberHistory, 's-', 'LineWidth', 1.1);
    grid on
    xlabel('Iteration')
    ylabel('Magnitude')
    title('Coefficient Change / Condition Number')
    legend({'Coeff relative change', 'Condition number'}, 'Location', 'best');

    nexttile;
    semilogy(strSHgravityData.strFitStats.dValidationShellRadii, ...
        strSHgravityData.strFitStats.dValidationShellAccRMSrelHistory.', 'LineWidth', 1.1);
    grid on
    xlabel(sprintf('Holdout radius [%s]', options.charDistanceUnit))
    ylabel('Relative RMS error')
    title('Validation shell acceleration RMS')

    nexttile;
    semilogy(strSHgravityData.strFitStats.dValidationShellRadii, ...
        strSHgravityData.strFitStats.dValidationShellPotRMSrelHistory.', 'LineWidth', 1.1);
    grid on
    xlabel(sprintf('Holdout radius [%s]', options.charDistanceUnit))
    ylabel('Relative RMS error')
    title('Validation shell potential RMS')

    sgtitle(objConvergenceFig, sprintf( ...
        'Adaptive SH fit convergence, degree %u, best iteration %u, converged = %d', ...
        strSHgravityData.ui32MaxDegree, strSHgravityData.strFitStats.ui32BestIteration, ...
        strSHgravityData.strFitStats.bConverged));

    strFigureHandles.objConvergenceFig = objConvergenceFig;
end

% Show holdout diagnostics if enabled
if options.bShowHoldoutFigure

    objHoldoutFig = figure('Renderer', char(options.charFigureRenderer), ...
        'Name', 'Polyhedron-to-SH Holdout Diagnostics');
    tiledlayout(objHoldoutFig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    dVerticesRows = strShapeModel.dVerticesPos.';
    ui32FacesRows = uint32(strShapeModel.ui32triangVertexPtr.');
    dPointColor = log10(max(strDiagnostics.strMetrics.dAccPointRel, eps(1.0e-16)));

    nexttile;
    patch('Vertices', dVerticesRows, ...
        'Faces', ui32FacesRows, ...
        'FaceColor', [0.80, 0.82, 0.85], ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.25);
    hold on
    scatter3(strDiagnostics.dHoldoutPos_TB(1, :), ...
        strDiagnostics.dHoldoutPos_TB(2, :), ...
        strDiagnostics.dHoldoutPos_TB(3, :), ...
        18, dPointColor, 'filled');
    axis equal
    grid on
    xlabel(sprintf('X [%s]', options.charDistanceUnit))
    ylabel(sprintf('Y [%s]', options.charDistanceUnit))
    zlabel(sprintf('Z [%s]', options.charDistanceUnit))
    title('Holdout points colored by log_{10} acc relative error')
    colorbar
    view(35, 24)

    nexttile;
    semilogy(strDiagnostics.dHoldoutShellRadii, strDiagnostics.strMetrics.dAccShellRMSrel, 'o-', 'LineWidth', 1.1);
    hold on
    semilogy(strDiagnostics.dHoldoutShellRadii, strDiagnostics.strMetrics.dPotentialShellRMSrel, 's-', 'LineWidth', 1.1);
    grid on
    xlabel(sprintf('Holdout radius [%s]', options.charDistanceUnit))
    ylabel('Relative RMS error')
    title('Holdout shell RMS errors')
    legend({'Acceleration', 'Potential'}, 'Location', 'best');

    nexttile;
    histogram(log10(max(strDiagnostics.strMetrics.dAccPointRel, eps(1.0e-16))), 24);
    grid on
    xlabel('log_{10}(relative acceleration error)')
    ylabel('Count')
    title('Holdout acceleration error histogram')

    nexttile;
    histogram(log10(max(strDiagnostics.strMetrics.dPotentialPointRel, eps(1.0e-16))), 24);
    grid on
    xlabel('log_{10}(relative potential error)')
    ylabel('Count')
    title('Holdout potential error histogram')

    sgtitle(objHoldoutFig, sprintf( ...
        'Holdout comparison, acc RMS rel = %.3e, potential RMS rel = %.3e', ...
        strDiagnostics.strMetrics.dAccRMSrel, strDiagnostics.strMetrics.dPotentialRMSrel));
    strFigureHandles.objHoldoutFig = objHoldoutFig;
end

end
