classdef EnumCovarianceSamplingDistribution < uint8
    %% SIGNATURE
    % enumDistribution = EnumCovarianceSamplingDistribution.<Value>
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Select a zero-mean probability distribution whose samples reproduce a configured covariance. Gaussian draws are
    % unbounded. Uniform-solid-ellipsoid draws are uniform in the covariance-matched solid ellipsoid, not in a box or
    % only on the ellipsoid surface.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % enumDistribution    (1,1) EnumCovarianceSamplingDistribution
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 23-08-2026  Pietro Califano, Codex     First implementation.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        GAUSSIAN                (0)
        UNIFORM_SOLID_ELLIPSOID (1)
    end

    methods (Static)
        function enumDistribution = FromConfigValue(varDistribution)
            %% SIGNATURE
            % enumDistribution = EnumCovarianceSamplingDistribution.FromConfigValue(varDistribution)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Normalize a configuration token or preserve an existing typed distribution. The legacy concise token
            % `uniform_ellipsoid` remains canonical and explicitly denotes the solid distribution.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % varDistribution    EnumCovarianceSamplingDistribution, char, or string scalar
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % enumDistribution   (1,1) EnumCovarianceSamplingDistribution
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 23-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                varDistribution {mustBeA(varDistribution, ...
                    ["EnumCovarianceSamplingDistribution", "char", "string"])}
            end
            arguments (Output)
                enumDistribution (1,1) EnumCovarianceSamplingDistribution
            end

            if isa(varDistribution, 'EnumCovarianceSamplingDistribution')
                if ~isscalar(varDistribution)
                    error('EnumCovarianceSamplingDistribution:NonScalarDistribution', ...
                        'Covariance sampling distribution must be scalar.');
                end
                enumDistribution = varDistribution;
                return
            end

            mustBeTextScalar(varDistribution);
            charDistribution = upper(char(strip(string(varDistribution))));
            switch charDistribution
                case 'GAUSSIAN'
                    enumDistribution = EnumCovarianceSamplingDistribution.GAUSSIAN;
                case {'UNIFORM_ELLIPSOID', 'UNIFORM_SOLID_ELLIPSOID'}
                    enumDistribution = EnumCovarianceSamplingDistribution.UNIFORM_SOLID_ELLIPSOID;
                otherwise
                    error('EnumCovarianceSamplingDistribution:UnsupportedDistribution', ...
                        'Unsupported covariance sampling distribution "%s".', ...
                        charDistribution);
            end
        end
    end

    methods
        function charConfigValue = ToConfigValue(self)
            %% SIGNATURE
            % charConfigValue = self.ToConfigValue()
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Convert the typed distribution to its canonical lowercase configuration token.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    (1,1) EnumCovarianceSamplingDistribution
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % charConfigValue    (1,:) char canonical configuration token
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 23-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                self (1,1) EnumCovarianceSamplingDistribution
            end
            arguments (Output)
                charConfigValue (1,:) char
            end

            switch self
                case EnumCovarianceSamplingDistribution.GAUSSIAN
                    charConfigValue = 'gaussian';
                case EnumCovarianceSamplingDistribution.UNIFORM_SOLID_ELLIPSOID
                    charConfigValue = 'uniform_ellipsoid';
                otherwise
                    error('EnumCovarianceSamplingDistribution:UnsupportedEnumValue', ...
                        'Unsupported covariance sampling distribution.');
            end
        end
    end
end
