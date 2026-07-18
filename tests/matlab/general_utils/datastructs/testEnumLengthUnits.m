classdef testEnumLengthUnits < matlab.unittest.TestCase
    methods (Test)
        function testStringValuesMatchRepositoryUnitTokens(testCase)
            testCase.verifyEqual(string(EnumLengthUnits.m), "m");
            testCase.verifyEqual(string(EnumLengthUnits.km), "km");
        end

        function testFromAnyAcceptsEnumStringAndChar(testCase)
            testCase.verifyEqual(EnumLengthUnits.fromAny(EnumLengthUnits.m), EnumLengthUnits.m);
            testCase.verifyEqual(EnumLengthUnits.fromAny("km"), EnumLengthUnits.km);
            testCase.verifyEqual(EnumLengthUnits.fromAny('m'), EnumLengthUnits.m);
        end

        function testToStringCanonicalizesInput(testCase)
            testCase.verifyEqual(EnumLengthUnits.toString("km"), "km");
            testCase.verifyEqual(EnumLengthUnits.toString(EnumLengthUnits.m), "m");
        end

        function testInvalidUnitFailsFast(testCase)
            testCase.verifyError(@() EnumLengthUnits.fromAny("meter"), ...
                "EnumLengthUnits:UnsupportedUnit");
        end
    end
end
