classdef EnumNavStateUncertaintyLayout < uint8
    %% DESCRIPTION
    % Error-state layout associated with a navigation-state uncertainty.
    % Layout values define both component ordering and exact matrix size.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-08-2026  Pietro Califano, Codex     First navigation uncertainty layout contract.
    % -------------------------------------------------------------------------------------------------------------
    %% ENUMERATIONS
    % POSITION_VELOCITY: [position; velocity], dimension 6.
    % POSITION_VELOCITY_ATTITUDE_ERROR: [position; velocity; attitude error],
    % dimension 9.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        POSITION_VELOCITY (1)
        POSITION_VELOCITY_ATTITUDE_ERROR (2)
    end
end
