classdef EnumTargetAngularVelocityRegime < uint8
    %% SIGNATURE
    % enumRegime = EnumTargetAngularVelocityRegime.<Value>
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Classify the angular-velocity information carried by one canonical target rotational-state history. This
    % enum is the sole capability discriminator; consumers must not infer the regime from optional fields or
    % individual samples.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % enumRegime    (1,1) EnumTargetAngularVelocityRegime
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 23-08-2026  Pietro Califano, Codex     First implementation.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
    % 13-11-2026  Pietro Califano, Claude Opus 5.5   Name the Classify input dTargetAngVel_IN.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        UNAVAILABLE       (0)
        CONSTANT_INERTIAL (1)
        TIME_VARYING      (2)
    end

    methods (Static)
        function enumRegime = Classify(dTargetAngVel_IN)
            %% SIGNATURE
            % enumRegime = EnumTargetAngularVelocityRegime.Classify(dTargetAngVel_IN)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Classify a validated inertial angular-velocity history using every sample. A history is constant when
            % every component stays within a scale-aware absolute tolerance from its first sample.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % dTargetAngVel_IN    (3,N) double finite inertial angular-velocity history [rad/s]
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % enumRegime    (1,1) EnumTargetAngularVelocityRegime classified rate-bearing regime
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 23-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                dTargetAngVel_IN (3,:) double {mustBeReal, mustBeFinite}
            end
            arguments (Output)
                enumRegime (1,1) EnumTargetAngularVelocityRegime
            end

            if isempty(dTargetAngVel_IN)
                error('EnumTargetAngularVelocityRegime:EmptyHistory', ...
                    'A rate-bearing angular-velocity history must contain at least one sample.');
            end

            % Compare every sample with the first one, within a tolerance scaled by the largest first component
            dReferenceAngVel_IN = dTargetAngVel_IN(:, 1);
            dRegimeTolerance = 1.0e-12 * max(1.0, max(abs(dReferenceAngVel_IN)));

            if max(abs(dTargetAngVel_IN - dReferenceAngVel_IN), [], 'all') <= dRegimeTolerance
                enumRegime = EnumTargetAngularVelocityRegime.CONSTANT_INERTIAL;
            else
                enumRegime = EnumTargetAngularVelocityRegime.TIME_VARYING;
            end
        end
    end
end
