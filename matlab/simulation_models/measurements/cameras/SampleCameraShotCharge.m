function [dCounts, ui64ExposureSeed] = SampleCameraShotCharge(dExpectedElectrons, charBackend)
%% SIGNATURE
% [dCounts, ui64ExposureSeed] = SampleCameraShotCharge(dExpectedElectrons, charBackend)
% -------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Sample hosted shot charge with an explicitly selected MATLAB, CPU MEX or CUDA backend.
% MATLAB preserves the original poissrnd stream. CPU MEX uses its compiled implementation.
% CUDA consumes two uint32 seed words from the caller's current MATLAB stream, then samples
% each pixel from an independent keyed Philox subsequence. No backend seeds the caller RNG.
% Resetting that RNG repeats the exposure and subsequent RNG state on the same backend/device.
% Different backends have different realizations and random-stream consumption. Fix the
% backend during a trial; select cpu_mex at preflight when CUDA is unavailable.
% Example: rng(71); dCounts = SampleCameraShotCharge(10*ones(8), "matlab");
% Output: an 8-by-8 double array of integer charge counts, repeatable after rng(71).
% -------------------------------------------------------------------------------------------------
%% INPUT
% dExpectedElectrons   Real double expected charge per pixel, in electrons.
% charBackend          Explicit sampler: matlab, cpu_mex or cuda.
% -------------------------------------------------------------------------------------------------
%% OUTPUT
% dCounts              Sampled charge counts with the input shape.
% ui64ExposureSeed     Recorded CUDA exposure key; zero for CPU samplers.
% -------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-10-2026  Pietro Califano     Select reproducible standalone accelerated charge sampling.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% poissrnd, DrawPoissonCountsCpu_mex or standalone Spectra DrawPoissonCountsCuda_mex.
% -------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    dExpectedElectrons (:,:) double {mustBeReal}
    charBackend (1,1) string {mustBeMember(charBackend, ["matlab", "cpu_mex", "cuda"])} = "matlab"
end
arguments (Output)
    dCounts (:,:) double
    ui64ExposureSeed (1,1) uint64
end

% Sample with the selected backend; only CUDA records an exposure key
ui64ExposureSeed = uint64(0);
switch charBackend
    case "matlab"
        dCounts = poissrnd(dExpectedElectrons);
    case "cpu_mex"
        if exist('DrawPoissonCountsCpu_mex', 'file') ~= 3
            error('SampleCameraShotCharge:MissingCpuMex', ...
                'Build the CPU shot sampler with BuildMexTargets_CameraPoisson and add its output directory.');
        end
        dCounts = DrawPoissonCountsCpu_mex(dExpectedElectrons);
    case "cuda"
        if exist('DrawPoissonCountsCuda_mex', 'file') ~= 3
            error('SampleCameraShotCharge:MissingCudaMex', ...
                'Build the standalone Spectra image_noise component and add its MATLAB output directory.');
        end
        % Derive a fresh full-width key from caller-owned state, never time, PID or worker order.
        ui32SeedWords = randi([0, double(intmax('uint32'))], 1, 2, 'uint32');
        ui64ExposureSeed = bitor(bitshift(uint64(ui32SeedWords(1)), 32), uint64(ui32SeedWords(2)));
        dCounts = DrawPoissonCountsCuda_mex(dExpectedElectrons, ui64ExposureSeed, uint32(0));
end
end
