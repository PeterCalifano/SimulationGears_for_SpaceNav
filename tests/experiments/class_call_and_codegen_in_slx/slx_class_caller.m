function y = slx_class_caller(u)
%#codegen
% Example of calling a MATLAB class inside a MATLAB Function block

    persistent self
    if isempty(self)
        % Initialize the object once at start
        self = SimulinkTestClass(2.5);  % e.g. scale = 2.5
    end

    % Call the class method
    y = self.multiply(u);
end
