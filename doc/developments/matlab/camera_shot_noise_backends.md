# Hosted camera shot sampling

SimulationGears owns the camera signal model and its selectable shot sampler.
Spectra-RT owns the standalone CUDA library and MATLAB adapter under
`src/utils/image_noise`; neither is part of a renderer scene or provider.

`SImageErrorsPipelineSettings.charShotNoiseBackend` selects `matlab`, `cpu_mex`
or `cuda`. The portable default remains `matlab`. COSMICA profiles must name
the backend explicitly. Select one backend before a trial and retain it in
the saved configuration; do not silently change generators after a runtime fault.

The camera still applies spatial blur, charge sampling, independent Gaussian
read noise, and clipping in that order. Gain maps blurred DN to expected
electrons. The signed single-precision output retains detector excursions before
clipping. Acceleration changes the generator and its numerical realization,
not that signal-model ordering.

## Build and use

Build the CPU fallback after normal library setup:

```matlab
charMexPath = BuildMexTargets_CameraPoisson();
addpath(fileparts(charMexPath));
objSettings = SImageErrorsPipelineSettings('bAddShotNoise', true, ...
    'charShotNoiseBackend', "cpu_mex");
```

The builder writes generated files under `matlab/mex/`, supports variable
two-dimensional camera sizes, and does not generate artifacts at the repository root.

Build and install the standalone CUDA component using its Spectra-RT README.
The MEX and `libspectra_rt_camera_noise.so` must stay together. Set
`SIM_GEARS_CAMERA_NOISE_MEX_DIR` to the installed `matlab/` directory before
library setup, or explicitly add that directory in the caller. The worker's
existing `CUDA_VISIBLE_DEVICES` allocation selects the physical GPU. The helper
does not select or reset a device, alter a renderer, or create a parallel worker.

```matlab
rng(71, 'twister');
objSettings = SImageErrorsPipelineSettings('bAddShotNoise', true, ...
    'charShotNoiseBackend', "cuda", ...
    'dSigmaDigitNumRelGenericWN', 3.5/255);
[ui8Preview, fSignedSignal] = ApplyCameraImageErrors(uint8(128*ones(32)), objSettings);
```

The outputs are a 32-by-32 noisy preview and the corresponding unclipped
normalized detector signal. Resetting the caller RNG repeats both outputs and
the subsequent RNG state on the same backend, binary, toolkit and device.

## Randomness and numerical limits

The MATLAB backend preserves the previous `poissrnd` sequence. CPU MEX uses
its compiled sampler. CUDA consumes two uint32 words from the caller's MATLAB
stream to derive a full-width exposure key, then uses independent Philox
subsequences per pixel. `SampleCameraShotCharge` optionally returns that key;
the native keyed API itself does not consume or reseed MATLAB randomness.
Backend changes do not preserve pixel equality or subsequent CPU RNG state.
Disabled image effects consume no random draws.

The CUDA sampler sums independent cuRAND draws with individual expected charge
at most 4000 electrons. This avoids the installed cuRAND normal approximation
above that value; it does not rescale a low-rate sample. The numerical library
still has a small measurable bias. At 6300 electrons, 2,097,152 draws measured
a mean excess of 0.412 electrons and shot variance 0.70% below ideal. At gain
6300/255 electrons/DN and read sigma 3.5 DN, these are 0.0167 DN mean bias and
0.32% lower total variance. This deviation was explicitly accepted for the
preview model; it does not establish physical detector fidelity.

## Verification and cost

Native tests cover invalid inputs, zero charge, independent keys, exact replay,
chunking and moments in detector units. MATLAB tests cover backend selection,
caller-owned RNG, signed readout, clipping and effect ordering. All 27 focused
camera checks passed with both MEX modules present; no accelerated case was skipped.
A fresh production process exactly reproduced a retained 1,048,576-pixel draw.

On three retained 2048-by-1536 body images, the warm complete blur/shot/read
stage took 0.076-0.085 s on physical GPU1, including allocation and transfers,
versus 1.00-1.23 s with CPU Coder MEX sampling. The measured stage reduction
was 91.8-93.8%. These component timings do not establish a full simulation speedup.
