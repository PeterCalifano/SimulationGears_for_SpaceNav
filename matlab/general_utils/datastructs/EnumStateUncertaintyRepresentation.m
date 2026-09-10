classdef EnumStateUncertaintyRepresentation < uint8
    %% DESCRIPTION
    % Explicit storage representation for a navigation-state uncertainty.
    % Matrix contents are never inspected to infer this metadata.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-08-2026  Pietro Califano, Codex     First uncertainty representation contract.
    % -------------------------------------------------------------------------------------------------------------
    %% ENUMERATIONS
    % COVARIANCE, SQRT_COVARIANCE_UPPER, SQRT_COVARIANCE_LOWER,
    % INFORMATION, SQRT_INFORMATION_UPPER, SQRT_INFORMATION_LOWER.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        COVARIANCE (1)
        SQRT_COVARIANCE_UPPER (2)
        SQRT_COVARIANCE_LOWER (3)
        INFORMATION (4)
        SQRT_INFORMATION_UPPER (5)
        SQRT_INFORMATION_LOWER (6)
    end
end
