classdef SCameraSensorMetadata
    %% SIGNATURE
    % objMetadata = SCameraSensorMetadata(Name=Value)
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Optional host-side detector and radiometric characteristics in native units.
    % Empty numeric values mean unknown, including unknown ADC bit depth. The
    % record neither selects a noise model nor asserts radiometric calibration.
    % The renderer applies optics and QE separately; exposure belongs to each
    % acquisition. No processing, resampling or inference happens in this record.
    % Example: objMetadata = SCameraSensorMetadata('dGainElectronsPerDN', 312.5);
    % Output: dGainElectronsPerDN is 312.5; the remaining numeric fields are empty.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % Named detector scalars, pixel pitch [x,y] in micrometers, f-number,
    % transmission fractions, QE [e-/photon] and provenance. Spectral grids
    % are positive increasing row vectors [nm]. Band QE has wavelength rows
    % and physical-band columns; names, when supplied, match those columns.
    % Scalar QE is constant over the renderer wavelength domain; sampled
    % responses interpolate linearly and vanish outside their support. QE
    % includes fill factor but excludes optics. Sampled transmission replaces
    % scalar transmission, which must be empty or neutral (one) in that case.
    % Preserve native DN units; do not transfer gain to tone-mapped previews.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % objMetadata  Immutable-value fields; default construction has no measurements.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 16-10-2026  Pietro Califano, Codex gpt-6.1  Add optional detector metadata.
    % 16-10-2026  Pietro Califano, Codex gpt-6.1  Retain optional native radiometric inputs.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------
    properties (SetAccess = immutable)
        % Detector values are independent: unknown is distinct from explicit zero.
        dGainElectronsPerDN (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBePositive} = []
        dReadNoiseSigmaElectrons (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeNonnegative} = []
        dDarkCurrentElectronsPerSec (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeNonnegative} = []
        dFullWellElectrons (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBePositive} = []
        dDetectorBitDepth (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = []
        dOffsetElectrons (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite} = []

        % Physical pitch is authored, never inferred from projection geometry.
        dPixelPitchMicrometers (1,:) double {mustBeReal, mustBeFinite, mustBePositive} = []
        dFNumber (1,:) double {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBePositive} = []
        dOpticsTransmittance (1,:) double ...
            {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeInRange(dOpticsTransmittance, 0, 1)} = []
        dOpticsWavelengthNm (1,:) double {mustBeReal, mustBeFinite, mustBePositive} = []
        dOpticsTransmittanceSamples (1,:) double ...
            {mustBeReal, mustBeFinite, mustBeInRange(dOpticsTransmittanceSamples, 0, 1)} = []

        % Band names describe responses; they do not select a rendering mode.
        dQuantumEfficiencyWavelengthNm (1,:) double ...
            {mustBeReal, mustBeFinite, mustBePositive} = []
        dQuantumEfficiency (1,:) double ...
            {mustBeReal, mustBeFinite, mustBeInRange(dQuantumEfficiency, 0, 1)} = []
        charWavebandNames (1,:) string = strings(1, 0)
        dWavebandQuantumEfficiency (:,:) double ...
            {mustBeReal, mustBeFinite, mustBeInRange(dWavebandQuantumEfficiency, 0, 1)} = []
        charSourceReference (1,1) string = ""
    end

    methods
        function self = SCameraSensorMetadata(options)
            %% SIGNATURE
            % self = SCameraSensorMetadata(Name=Value)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Retain validated supplied fields without inferring missing values.
            % Reject inconsistent dimensions/grids before constructing a record.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % options  Optional physical inputs in the units documented above.
            % Dark current is e-/s/photosite; read noise is e- RMS; offset is
            % signed e- added before gain division. Full well is e-, gain is
            % e-/DN, and ADC depth is an integer number of bits (the current
            % native ADC supports only 1-16). Arrays are stored as supplied.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self  Value record; source text is provenance, not a mode selector.
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 16-10-2026  Pietro Califano, Codex gpt-6.1  Add radiometric metadata validation.
            % 11-11-2026  Pietro Califano, Claude Opus 5.5  Add section separators.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None. Host-side construction; no renderer/provider is required.
            % ---------------------------------------------------------------------------------------------------------
            arguments (Input)
                options.dGainElectronsPerDN (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBePositive} = []
                options.dReadNoiseSigmaElectrons (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeNonnegative} = []
                options.dDarkCurrentElectronsPerSec (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeNonnegative} = []
                options.dFullWellElectrons (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBePositive} = []
                options.dDetectorBitDepth (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = []
                options.dOffsetElectrons (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite} = []
                options.dPixelPitchMicrometers (1,:) double ...
                    {mustBeReal, mustBeFinite, mustBePositive} = []
                options.dFNumber (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBePositive} = []
                options.dOpticsTransmittance (1,:) double ...
                    {mustBeScalarOrEmpty, mustBeReal, mustBeFinite, ...
                    mustBeInRange(options.dOpticsTransmittance, 0, 1)} = []
                options.dOpticsWavelengthNm (1,:) double ...
                    {mustBeReal, mustBeFinite, mustBePositive} = []
                options.dOpticsTransmittanceSamples (1,:) double ...
                    {mustBeReal, mustBeFinite, ...
                    mustBeInRange(options.dOpticsTransmittanceSamples, 0, 1)} = []
                options.dQuantumEfficiencyWavelengthNm (1,:) double ...
                    {mustBeReal, mustBeFinite, mustBePositive} = []
                options.dQuantumEfficiency (1,:) double ...
                    {mustBeReal, mustBeFinite, mustBeInRange(options.dQuantumEfficiency, 0, 1)} = []
                options.charWavebandNames (1,:) string = strings(1, 0)
                options.dWavebandQuantumEfficiency (:,:) double ...
                    {mustBeReal, mustBeFinite, ...
                    mustBeInRange(options.dWavebandQuantumEfficiency, 0, 1)} = []
                options.charSourceReference (1,1) string = ""
            end
            arguments (Output)
                self (1,1) SCameraSensorMetadata
            end

            % Preserve physical axes and strictly ordered wavelength nodes.
            if ~isempty(options.dPixelPitchMicrometers) && numel(options.dPixelPitchMicrometers) ~= 2
                error('SCameraSensorMetadata:InvalidPixelPitch', ...
                    'Pixel pitch must be empty or contain [x,y] in micrometers.');
            end
            if any(diff(options.dOpticsWavelengthNm) <= 0) || ...
                    any(diff(options.dQuantumEfficiencyWavelengthNm) <= 0)
                error('SCameraSensorMetadata:InvalidWavelengthGrid', ...
                    'Optics and QE wavelength grids must be strictly increasing in nm.');
            end

            % A sampled optical response replaces the scalar, never multiplies it.
            if ~isempty(options.dOpticsWavelengthNm) || ~isempty(options.dOpticsTransmittanceSamples)
                if numel(options.dOpticsWavelengthNm) < 2 || ...
                        numel(options.dOpticsWavelengthNm) ~= numel(options.dOpticsTransmittanceSamples)
                    error('SCameraSensorMetadata:InvalidOpticalResponse', ...
                        'Sampled transmission requires at least two matching wavelength/value nodes.');
                end
                if ~isempty(options.dOpticsTransmittance) && options.dOpticsTransmittance ~= 1
                    error('SCameraSensorMetadata:DuplicateOpticalResponse', ...
                        'Leave scalar transmission empty or one when sampled transmission is supplied.');
                end
            end

            % Constant QE needs no grid; sampled curves share wavelength rows.
            dNumWavelengths = numel(options.dQuantumEfficiencyWavelengthNm);
            dNumPanSamples = numel(options.dQuantumEfficiency);
            [dNumBandSamples, dNumBands] = size(options.dWavebandQuantumEfficiency);
            if (dNumPanSamples > 1 && dNumPanSamples ~= dNumWavelengths) || ...
                    (dNumBandSamples > 1 && dNumBandSamples ~= dNumWavelengths) || ...
                    xor(dNumBandSamples == 0, dNumBands == 0)
                error('SCameraSensorMetadata:InvalidQuantumEfficiency', ...
                    'QE must be constant, match the shared grid, or be empty (band matrix 0-by-0).');
            end
            if ~isempty(options.charWavebandNames) && ...
                    (numel(options.charWavebandNames) ~= dNumBands || ...
                    any(ismissing(options.charWavebandNames) | strlength(options.charWavebandNames) == 0) || ...
                    numel(unique(options.charWavebandNames)) ~= dNumBands)
                error('SCameraSensorMetadata:InvalidWavebandNames', ...
                    'Optional waveband names must be nonempty, unique and match QE columns.');
            end

            % Retain independently known scalars; no gain/full-well/offset inference.
            self.dGainElectronsPerDN = options.dGainElectronsPerDN;
            self.dReadNoiseSigmaElectrons = options.dReadNoiseSigmaElectrons;
            self.dDarkCurrentElectronsPerSec = options.dDarkCurrentElectronsPerSec;
            self.dFullWellElectrons = options.dFullWellElectrons;
            self.dDetectorBitDepth = options.dDetectorBitDepth;
            self.dOffsetElectrons = options.dOffsetElectrons;

            % Copy response data without resampling, combining optics/QE or applying noise.
            self.dPixelPitchMicrometers = options.dPixelPitchMicrometers;
            self.dFNumber = options.dFNumber;
            self.dOpticsTransmittance = options.dOpticsTransmittance;
            self.dOpticsWavelengthNm = options.dOpticsWavelengthNm;
            self.dOpticsTransmittanceSamples = options.dOpticsTransmittanceSamples;
            self.dQuantumEfficiencyWavelengthNm = options.dQuantumEfficiencyWavelengthNm;
            self.dQuantumEfficiency = options.dQuantumEfficiency;
            self.charWavebandNames = options.charWavebandNames;
            self.dWavebandQuantumEfficiency = options.dWavebandQuantumEfficiency;
            self.charSourceReference = options.charSourceReference;
        end
    end
end
