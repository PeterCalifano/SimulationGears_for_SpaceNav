classdef SimulinkTestClass
    %#codegen
    % A simple value-class that multiplies its input by a constant scale

    properties
        scale (1,1) double = 1;
    end

    methods
        function obj = SimulinkTestClass(s)
            % Constructor: set the scale factor
            obj.scale = s;
        end

        function y = multiply(obj, u)
            % Multiply input u by the stored scale factor
            y = obj.scale .* u;
        end
    end
end
