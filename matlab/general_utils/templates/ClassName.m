classdef ClassName < handle
    %% DESCRIPTION
    % What the class represent
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % DD-MM-YYYY        Pietro Califano         Modifications
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

    %% Public properties
    properties
        Prop1    % e.g. numeric or string
        Prop2
    end

    %% Private or Hidden properties
    properties (Access = private, Hidden)
        PrivateProp  % used internally
    end

    %% Constructor and other public methods
    methods
        function self = ClassName(prop1, prop2, varargin)
            %CLASSNAME  Construct an instance of this class
            %
            %   obj = ClassName()              uses defaults
            %   obj = ClassName(p1, p2)        sets Prop1=p1, Prop2=p2
            %   obj = ClassName(...,'Opt',v)   parses name-value pairs

            %--- Set defaults
            default1 = 0;
            default2 = '';

            %--- Assign required inputs if provided
            if nargin >= 1 && ~isempty(prop1)
                self.Prop1 = prop1;
            else
                self.Prop1 = default1;
            end
            if nargin >= 2 && ~isempty(prop2)
                self.Prop2 = prop2;
            else
                self.Prop2 = default2;
            end

            %--- Parse optional name-value args
            p = inputParser;
            addParameter(p, 'OptA', 10);
            addParameter(p, 'OptB', true);
            parse(p, varargin{:});
            optA = p.Results.OptA;
            optB = p.Results.OptB;

            %--- Use the options (example)
            self.PrivateProp = optA;
            if optB
                fprintf('Initialized with OptB = true\n');
            end
        end

        function out = doSomething(obj, x)
            %dosomething  Example method that uses properties
            %
            %   y = obj.doSomething(x) returns Prop1 + x
            out = obj.Prop1 + x;
        end
    end

    %% Static factory methods
    methods (Static)
        function obj = fromStruct(s)
            %FROMSTRUCT  Create an instance from a struct
            %
            %   s must have fields .Prop1 and .Prop2
            obj = ClassName(s.Prop1, s.Prop2);
            % You can also pull in other struct fields as needed
        end
    end
end
