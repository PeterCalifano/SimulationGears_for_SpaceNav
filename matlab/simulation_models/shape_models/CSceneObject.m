classdef (Abstract) CSceneObject < handle
%% DESCRIPTION
    % Base abstract class for scene objects providing standard interface methods common to all. Currently
    % implemented as pure virtual class (no methods/properties implemented here).
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 02-02-2025        Pietro Califano        First prototype implementation 
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
        function self = CSceneObject()
            arguments
            end


        end

        % GETTERS

        % SETTERS

        % METHODS
    end



    % methods (Access=private)
    % 
    % end

    % methods (Access=public, Static)
    % 
    % end

    methods (Abstract, Access = public)
       % NOTE: number of args matters, names do not.
       [dPosVector_W, dRot3_WfromObjF] = GetPose(self, enumParamType)

       [dRot3_WfromObjF] = rotation(self, enumParamType)

       [self] = SetPose(self, dPosVector_W, dRot3_WfromObjF)

    end
end
