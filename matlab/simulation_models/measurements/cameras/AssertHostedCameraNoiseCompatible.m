function AssertHostedCameraNoiseCompatible(objSettings, objRenderer)
%% SIGNATURE
% AssertHostedCameraNoiseCompatible(objSettings, objRenderer)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Require a preview renderer for hosted stochastic image errors. The selected
% Spectra preview wrapper has no native detector-noise interface. A physical
% detector provider exposing setDetectorNoise or captureSensorFrame cannot
% establish its noise state through this preview protocol, so reject combining
% it with hosted noise. Do not capture frames, mutate detector settings or draw
% randomness to probe that state. Physical-detector integration is separate.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objSettings  Hosted image-error settings; deterministic effects remain valid.
% objRenderer  Injected renderer inspected only for native detector entry points.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None. Throw AssertHostedCameraNoiseCompatible:NativeDetectorProvider when
% hosted stochastic effects and a native detector interface are both present.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2026  Pietro Califano, Codex gpt-6.1  Guard preview noise ownership.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SImageErrorsPipelineSettings
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    objSettings (1,1) SImageErrorsPipelineSettings
    objRenderer (1,1)
end

% Match enabled stochastic stages, including retained white-noise aliases.
bHostedStochasticNoise = objSettings.bAddShotNoise || ...
    objSettings.dSigmaDigitNumRelGenericWN > 0 || objSettings.dSigmaGenericWNRel > 0 || ...
    objSettings.dSigmaGenericWN > 0 || objSettings.dDeadPixelPerc > 0 || ...
    objSettings.dDeadBucketFraction > 0 || ...
    (objSettings.dBloomingProbability > 0 && objSettings.dBloomPoissonMean > 0) || ...
    (objSettings.dBrightSpotsProbability > 0 && objSettings.dBrightSpotsPoissonRate > 0);

% These physical-detector APIs do not expose a read-only noise-status query.
if bHostedStochasticNoise && (ismethod(objRenderer, 'setDetectorNoise') || ...
        ismethod(objRenderer, 'captureSensorFrame'))
    error('AssertHostedCameraNoiseCompatible:NativeDetectorProvider', ...
        ['Hosted stochastic image errors require a noise-free preview provider. ', ...
         'Disable hosted noise or supply that provider; native detector providers ', ...
         'need a separate integration that establishes sampling ownership.']);
end
end
