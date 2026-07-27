classdef CGeneralPropagator < handle
    %% DESCRIPTION
    % Host-side propagation dispatcher for orbit and generic state dynamics.
    % MATLAB ODE solvers and the shared SimulationGears fixed-step provider use
    % one state-history-first output contract.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 15-03-2025    Pietro Califano     Implemented moving methods from CScenarioGeneration (now a subclass)
    % 27-07-2026    Pietro Califano, Codex     Add shared RK4/RK8 dispatch.
    % 27-07-2026    Pietro Califano, Codex     Bind the explicit shared-provider RHS contract.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % See public methods below or call function: methods <class_name>.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % See list below or call function: properties <class_name>.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = protected, GetAccess = public)
                
        % Initial states
        dPosVelState0_W
        dAttDCM0_W
        
        % Time grid
        dEphemerisTimegrid  
        dRelativeTimegrid

        % Configuration
        bEnablePlots        = false;

        % Handles
        objOdeSolution            {mustBeScalarOrEmpty} = []
        objOrbitDynamicFcnHandle  {mustBeScalarOrEmpty} = []

    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = CGeneralPropagator()
            arguments
            end

        end

        % GETTERS

        % SETTERS

        % METHODS
        function [dxStateTrajectory, dTimegrid] = propagateOrbitTrajectory(self, varargparams, settings)
            arguments
                self
            end
            arguments (Repeating)
                varargparams
            end
            arguments
                settings.dTimestep (1,1) double {isscalar, isnumeric} = 0.0 % Default
                settings.objOdeOpts {isstruct} = odeset('RelTol', 1E-12, 'AbsTol', 1E-12) % Default
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78"])}
            end

            % TODO: integrates equations of motions of attitude kinematics

            % [dxStateTrajectory, dTimegrid] = propagateState(objDynamicFcnHandle, ...
            %                                                 self.dEphemerisTimegrid, ...
            %                                                 self.dPosVelState0, ...
            %                                                 varargparams, ...
            %                                                 "dTimestep", settings.dTimestep, ...
            %                                                 "objOdeOpts", settings.objOdeOpts, ...
            %                                                 "enumOdeFunctioName", settings.enumOdeFunctioName);

            % Unwrap settings to cell
            cellSettings = CScenarioGenerator.unwrapSettings(settings);

            % Call ODE-based propagator
            [dxStateTrajectory, dTimegrid] = CScenarioGenerator.propagateState(self.objOrbitDynamicFcnHandle, ...
                                                                            self.dEphemerisTimegrid, ...
                                                                            self.dPosVelState0_W, ...
                                                                            varargparams, ...
                                                                            cellSettings{:});
        end

        % TODO
        %function [] = propagateAttitudePoitingProfile(self)
        %    % Generate attitude pointing profile from CAttitudeGenerator class
        %end
        
        % TODO
        %function [] = propagateFreeAttitudeProfile(self)
        %    % TODO: integrates equations of motions of attitude kinematics
        %end

        function [] = propagateNavPoseTrajectory(self)
            % TODO: integrates equations of motions of orbit dynamics + attitude kinematics
            error('%s:NotImplemented', class(self), ...
                'propagateNavPoseTrajectory is not implemented yet.');
        end

        function [] = propagatePoseDynamicsTrajectory(self)
            % TODO: integrates equations of motions of orbit and attitude dynamics + kinematics
            error('%s:NotImplemented', class(self), ...
                'propagatePoseDynamicsTrajectory is not implemented yet.');
        end

    end

    methods (Access = protected)

    end

    methods (Access = public, Static)

        function [dxStateTrajectory, dTimegrid] = propagateState(objDynamicFcnHandle, ...
                                                                dTimegrid, ...
                                                                dxState0, ...
                                                                varargparams, ...
                                                                settings)
            %% SIGNATURE
            % [dxStateTrajectory, dTimegrid] = CGeneralPropagator.propagateState( ...
            %     objDynamicFcnHandle, dTimegrid, dxState0, varargin, Name=Value)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Propagate a column state through a MATLAB ODE solver or the
            % shared fixed-step RK4/RK8 provider. Requested grids with more
            % than two timestamps are preserved as output-sample locations.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % objDynamicFcnHandle          (1,1) function_handle state derivative
            % dTimegrid                    (1,N) double interval or requested output grid
            % dxState0                     (Mx1) double initial state
            % varargparams                 repeating derivative parameters
            % settings.dTimestep           (1,1) double fixed-step maximum magnitude
            % settings.objOdeOpts          (1,1) struct MATLAB ODE options
            % settings.enumOdeFunctioName  (1,1) string solver identifier
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % dxStateTrajectory            (NxM) double state history, one state per row
            % dTimegrid                    (Nx1) double state timestamps
            % -------------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-07-2026  Pietro Califano, Codex     Implement RK4/RK8 dispatch.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % EnumFixedStepScheme, PropagateFixedStep
            % -------------------------------------------------------------------------------------------------------------
            arguments
                objDynamicFcnHandle {mustBeA(objDynamicFcnHandle, 'function_handle')}
                dTimegrid           (1,:) double {isvector, isnumeric}
                dxState0            (:,1) double {isvector, isnumeric}
            end
            arguments (Repeating)
                varargparams
            end
            arguments
                settings.dTimestep (1,1) double {isscalar, isnumeric} = 0.0 % Default
                settings.objOdeOpts {isstruct} = odeset('RelTol', 1E-12, 'AbsTol', 1E-12) % Default
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78", "RK4", "RK8"])} = "ode113"
            end

            assert(length(dTimegrid) >= 2, ...
                'CGeneralPropagator:InvalidTimegrid', ...
                'The time grid must contain at least two timestamps.');
            if settings.dTimestep < 0.0
                error('CGeneralPropagator:InvalidTimestep', ...
                    'The fixed-step magnitude must be nonnegative.');
            end

            % Bind legacy caller parameters once while exposing the explicit
            % SimulationGears RHS contract only to the shared fixed-step path.
            fcnOdeStateDerivative = @(dTime, dxState) objDynamicFcnHandle( ...
                dTime, dxState, varargparams{:});
            fcnFixedStateDerivative = @(dTime, dxState, ...
                strDynParams, strModelConfigFlags) objDynamicFcnHandle( ...
                dTime, dxState, varargparams{:});

            switch settings.enumOdeFunctioName
                case "ode113"

                    [dTimegrid, dxStateTrajectory] = ode113( ...
                        fcnOdeStateDerivative, dTimegrid, dxState0, ...
                        settings.objOdeOpts);

                case "ode45"

                    [dTimegrid, dxStateTrajectory] = ode45( ...
                        fcnOdeStateDerivative, dTimegrid, dxState0, ...
                        settings.objOdeOpts);
                    
                case "ode78"

                    [dTimegrid, dxStateTrajectory] = ode78( ...
                        fcnOdeStateDerivative, dTimegrid, dxState0, ...
                        settings.objOdeOpts);

                case "RK4"
                    [dxStateTrajectory, dTimegrid] = PropagateFixedGrid_( ...
                        fcnFixedStateDerivative, dTimegrid, dxState0, ...
                        settings.dTimestep, EnumFixedStepScheme.RK4);

                case "RK8"
                    [dxStateTrajectory, dTimegrid] = PropagateFixedGrid_( ...
                        fcnFixedStateDerivative, dTimegrid, dxState0, ...
                        settings.dTimestep, EnumFixedStepScheme.RK8);

                otherwise
                    error('CGeneralPropagator:UnsupportedSolver', ...
                        'Unsupported propagation solver.');
            end
        end

        function [] = propagatedStateAutoDiff(objDynamicFcnHandle, ...
                                                dTimegrid, ...
                                                dxState0, ...
                                                varargparams, ...
                                                settings)
            arguments
                objDynamicFcnHandle {mustBeA(objDynamicFcnHandle, 'function_handle')}
                dTimegrid           (1,:) double {isvector, isnumeric}
                dxState0            (:,1) double {isvector, isnumeric}
            end
            arguments (Repeating)
                varargparams
            end
            arguments
                settings.dTimestep (1,1) double {isscalar, isnumeric} = 0.0 % Default
                settings.objOdeOpts {isstruct} = odeset('RelTol', 1E-12, 'AbsTol', 1E-12) % Default
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78", "RK4", "RK8"])} = "ode113"
            end

            % TODO implement integrator manager as in IntegratorStepRKX functions
            

        end

        function cellSettings = unwrapSettings(settings)
            arguments
                settings (1,1) {isstruct}
            end

            cellFieldNames = fieldnames(settings);
            cellSettings = cell(1, length(cellFieldNames));

            ui32AllocCounter = 1;

            for idF = 1:length(cellSettings)

                % Store key
                cellSettings{ui32AllocCounter} = cellFieldNames{idF};
                ui32AllocCounter = ui32AllocCounter + 1;

                % Store value
                cellSettings{ui32AllocCounter} = settings.(cellFieldNames{idF});
                ui32AllocCounter = ui32AllocCounter + 1;

            end
        end

        %%% Additional tools from "simulationUtils" repository (private)
        function [y, tInt,te, ye, ie] = PropagateOrbitTrajectoryHF_Static(xi, t_vect, data, kwargs, options)
            arguments
                xi
                t_vect
                data
                kwargs
                options
            end

            error('Not implemented yet')
        end

    end

    % methods (Access=private)
    % 
    % end

    % methods (Abstract, Access=public)
    % [x,y] = abstract_function_name(args) NOTE: number of args matter.
    % end
end

function [dxStateTrajectory, dOutputTimegrid] = PropagateFixedGrid_( ...
    fcnStateDerivative, dRequestedTimegrid, dxState0, dMaximumStep, ...
    enumFixedStepScheme)
% Propagate a requested fixed-step interval or preserve an explicit sample grid.
if numel(dRequestedTimegrid) == 2
    dIntervalDuration = abs(dRequestedTimegrid(2) - dRequestedTimegrid(1));
    if dMaximumStep == 0.0
        dMaximumStep = max(dIntervalDuration, 1.0);
    end

    [dxStateTrajectory, dOutputTimegrid] = PropagateFixedStep( ...
        fcnStateDerivative, dRequestedTimegrid, dxState0, dMaximumStep, ...
        struct(), struct(), enumFixedStepScheme);
    return;
end

% For an explicit output grid, integrate each adjacent interval internally
% and retain only its endpoint so the caller's sampling contract is unchanged.
dOutputTimegrid = dRequestedTimegrid(:);
dxStateTrajectory = zeros(numel(dOutputTimegrid), numel(dxState0));
dxStateTrajectory(1, :) = dxState0.';
dxCurrentState = dxState0;

for ui32IntervalIndex = uint32(1):uint32(numel(dOutputTimegrid) - 1)
    dIntervalTimeSpan = dOutputTimegrid( ...
        double(ui32IntervalIndex):double(ui32IntervalIndex) + 1).';
    dIntervalDuration = abs(dIntervalTimeSpan(2) - dIntervalTimeSpan(1));
    dIntervalMaximumStep = dMaximumStep;
    if dIntervalMaximumStep == 0.0
        dIntervalMaximumStep = max(dIntervalDuration, 1.0);
    end

    dxIntervalHistory = PropagateFixedStep(fcnStateDerivative, ...
        dIntervalTimeSpan, dxCurrentState, dIntervalMaximumStep, ...
        struct(), struct(), enumFixedStepScheme);
    dxCurrentState = dxIntervalHistory(end, :).';
    dxStateTrajectory(double(ui32IntervalIndex) + 1, :) = ...
        dxCurrentState.';
end
end
