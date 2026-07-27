classdef EnumFixedStepScheme < uint8
    %% DESCRIPTION
    % Typed selection of the fixed-step numerical integration scheme used by
    % PropagateFixedStep.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 27-07-2026  Pietro Califano, Codex     First fixed-step scheme contract.
    % -------------------------------------------------------------------------------------------------------------
    %% ENUMERATIONS
    % RK2Heun, RK4, RK8.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        RK2Heun (2)
        RK4 (4)
        RK8 (8)
    end
end
