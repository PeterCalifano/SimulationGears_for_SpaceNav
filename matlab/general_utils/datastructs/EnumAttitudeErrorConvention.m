classdef EnumAttitudeErrorConvention < uint8
    %% DESCRIPTION
    % Perturbation convention for a navigation-state attitude-error block.
    % NONE is reserved for layouts that do not contain attitude uncertainty.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-08-2026  Pietro Califano, Codex     First attitude-error convention contract.
    % -------------------------------------------------------------------------------------------------------------
    %% ENUMERATIONS
    % NONE: no attitude-error block.
    % LEFT_FRAME: attitude error expressed in and rotated with the frame.
    % RIGHT_POSE: right-side pose perturbation unchanged by frame rotation.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        NONE (0)
        LEFT_FRAME (1)
        RIGHT_POSE (2)
    end
end
