classdef CNativeDetectorRendererFixture < CCameraRendererFixture
    %% SIGNATURE
    % objRenderer = CNativeDetectorRendererFixture()
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Advertise native detector entry points without permitting state probes.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % objRenderer  Deterministic preview fixture with native API sentinels.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 18-10-2026  Pietro Califano, Codex gpt-6.1  Exercise hosted-noise boundary.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CCameraRendererFixture
    % -------------------------------------------------------------------------------------------------------------

    methods
        function setDetectorNoise(self, dDarkCurrent, dReadNoise, bSampleNoise, ui64Seed)
            %% SIGNATURE
            % setDetectorNoise(self, dDarkCurrent, dReadNoise, bSampleNoise, ui64Seed)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Fail if the host attempts to change native detector configuration.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self          Fixture renderer.
            % dDarkCurrent  Dark charge rate in e-/s.
            % dReadNoise    Read-noise standard deviation in electrons.
            % bSampleNoise  Native sampling flag.
            % ui64Seed      Native seed; never consumed.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; always throws to detect an unwanted mutation.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 18-10-2026  Pietro Califano, Codex gpt-6.1  Add mutation sentinel.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % ---------------------------------------------------------------------------------------------------------
            % Keep the native signature; the sentinel throws without reading its inputs.
            arguments (Input)
                self (1,1) CNativeDetectorRendererFixture %#ok<INUSA>
                dDarkCurrent (1,1) double %#ok<INUSA>
                dReadNoise (1,1) double %#ok<INUSA>
                bSampleNoise (1,1) logical %#ok<INUSA>
                ui64Seed (1,1) uint64 %#ok<INUSA>
            end

            error('CNativeDetectorRendererFixture:UnexpectedMutation', ...
                'The host guard must not mutate detector settings.');
        end

        function captureSensorFrame(self) %#ok<MANU>
            %% SIGNATURE
            % captureSensorFrame(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Fail if the host samples or captures a native frame to probe noise.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self  Fixture renderer.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None; always throws to detect an unwanted acquisition.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 18-10-2026  Pietro Califano, Codex gpt-6.1  Add sampling sentinel.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % ---------------------------------------------------------------------------------------------------------
            arguments (Input)
                self (1,1) CNativeDetectorRendererFixture
            end

            error('CNativeDetectorRendererFixture:UnexpectedCapture', ...
                'The host guard must not capture a sensor frame.');
        end
    end
end
