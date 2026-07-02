classdef testIsInCylindricalTargetShadow < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for the cylindrical target-shadow predicate used by SRP dynamics.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testSpacecraftBehindTargetInsideCylinderIsShadowed(testCase)
            bIsInShadow = IsInCylindricalTargetShadow([-2.0; 0.5; 0.0], ...
                                                      [10.0; 0.0; 0.0], ...
                                                      1.0);

            testCase.verifyTrue(bIsInShadow);
        end

        function testSpacecraftOutsideCylinderIsSunlit(testCase)
            bIsInShadow = IsInCylindricalTargetShadow([-2.0; 2.0; 0.0], ...
                                                      [10.0; 0.0; 0.0], ...
                                                      1.0);

            testCase.verifyFalse(bIsInShadow);
        end

        function testSpacecraftSunwardOfTargetIsSunlit(testCase)
            bIsInShadow = IsInCylindricalTargetShadow([2.0; 0.0; 0.0], ...
                                                      [10.0; 0.0; 0.0], ...
                                                      1.0);

            testCase.verifyFalse(bIsInShadow);
        end

        function testInvalidGeometryReturnsSunlit(testCase)
            testCase.verifyFalse(IsInCylindricalTargetShadow([-2.0; 0.0; 0.0], ...
                                                            [10.0; 0.0; 0.0], ...
                                                            0.0));
            testCase.verifyFalse(IsInCylindricalTargetShadow([-2.0; 0.0; 0.0], ...
                                                            zeros(3, 1), ...
                                                            1.0));
        end
    end
end
