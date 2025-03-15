classdef CGeneralPropagator < handle
    %% DESCRIPTION
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 15-03-2025    Pietro Califano     Implemented moving methods from CScenarioGeneration (now a subclass)
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

        function [] = propagateAttitudePoitingProfile(self)
            % Generate attitude pointing profile from CAttitudeGenerator class

        end

        function [] = propagateFreeAttitudeProfile(self)
            % TODO: integrates equations of motions of attitude kinematics
        end

        function [] = propagateNavPoseTrajectory(self)
            % TODO: integrates equations of motions of orbit dynamics + attitude kinematics
        end

        function [] = propagatePoseDynamicsTrajectory(self)
            % TODO: integrates equations of motions of orbit and attitude dynamics + kinematics
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
                settings.enumOdeFunctioName {mustBeMember(settings.enumOdeFunctioName, ["ode113", "ode45", "ode78"])} = "ode113"
            end


            % objDynamicFcnHandle = @(dTime, dxState) objDynamicFcnHandle(dTime, dxState);
            % cellOdeInput = {objDynamicFcnHandle, dTimegrid, dxState0, settings.objOdeOpts};

            assert(length(dTimegrid) >= 2, 'ERROR: invalid timegrid. It must contain at least two time instants.')

            % Determine timegrid if required
            if length(dTimegrid) == 2 && not(settings.dTimestep == 0)
                dTimegrid = dTimegrid(1):settings.dTimestep:dTimegrid(2);
            end

            switch settings.enumOdeFunctioName
                case "ode113"

                    [dTimegrid, dxStateTrajectory] = ode113(objDynamicFcnHandle, ...
                        dTimegrid, ...
                        dxState0, ...
                        settings.objOdeOpts);

                case "ode45"

                    [dTimegrid, dxStateTrajectory] = ode45(objDynamicFcnHandle, ...
                        dTimegrid, ...
                        dxState0, ...
                        settings.objOdeOpts);
                case "ode78"

                    [dTimegrid, dxStateTrajectory] = ode78(objDynamicFcnHandle, ...
                        dTimegrid, ...
                        dxState0, ...
                        settings.objOdeOpts);

                otherwise
                    error('Unsupported ode function')
            end
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

    end

    % methods (Access=private)
    % 
    % end



    % methods (Abstract, Access=public)
    % [x,y] = abstract_function_name(args) NOTE: number of args matter.
    % end
end

