classdef CNonBaseSerializationProbe
    %% CNonBaseSerializationProbe
    % Helper class for CBaseDatastruct export tests. It intentionally does not inherit from CBaseDatastruct.

    properties (SetAccess = public, GetAccess = public)
        dScalar double = 42.0
        tablePayload table = table([1; 2], [3; 4], 'VariableNames', {'A', 'B'})
    end

    methods
        function strPayload = toStruct(self)
            arguments
                self (1,1) CNonBaseSerializationProbe
            end

            strPayload = struct();
            strPayload.dScalar = self.dScalar;
            strPayload.tablePayload = self.tablePayload;
        end
    end
end
