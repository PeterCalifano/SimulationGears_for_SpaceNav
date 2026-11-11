classdef SImageErrorsPipelineSettings < CBaseSettings
    %% SIGNATURE
    % self = SImageErrorsPipelineSettings(Name=Value)
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % SimulationGears-owned settings for the hosted camera image-error pipeline.
    % Field names remain stable for existing YAML/runtime sensor consumers.
    % Gain maps preview DN to expected electrons; the default 6300/255 e-/DN
    % is an approximation, independent of optional physical detector metadata.
    % All image effects remain disabled by default for clean-image fixtures.
    % Select shot sampling explicitly: matlab, cpu_mex, or the independent cuda helper.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % Named output, blur, gain and image-error parameters documented below.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % self  Value settings; construction neither seeds nor consumes the RNG.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 08-03-2025        Pietro Califano         First implementation.
    % 12-10-2026  Pietro Califano, Codex gpt-6.1  Move to the sensor-model owner.
    % 18-10-2026  Pietro Califano, Codex gpt-6.1  Add explicit preview charge gain.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % See public methods below or call function: methods <class_name>.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % See list below or call function: properties <class_name>.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = public, GetAccess = public)
        charOutputDataType          (1,:) string {mustBeMember(charOutputDataType, ["input", "double", "single", "uint16", "uint8", "uint32"])} = "input"
        dNUM_LEVELS                 (1,1) double {mustBeNumeric} = 255;
        dMeanGenericWN              (1,1) double {mustBeNumeric} = 0
        dSigmaDigitNumRelGenericWN  (1,1) double {mustBeNumeric} = 0
        dSigmaGenericWNRel          (1,1) double {mustBeNumeric} = 0 % Deprecated legacy alias
        dSigmaGenericWN             (1,1) double {mustBeNumeric} = 0 % Deprecated legacy alias
        dSigmaGenericBlur           (1,1) double {mustBeNumeric} = 0
        dGenericBlurFilterSize      (1,1) int32  {mustBeNumeric} = 3
        bAddShotNoise               (1,1) logical  = false;
        charShotNoiseBackend        (1,1) string ...
            {mustBeMember(charShotNoiseBackend, ["matlab", "cpu_mex", "cuda"])} = "matlab"
        dGainElectronsPerDN         (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 6300/255
        dDeadPixelPerc              (1,1) double {mustBeNumeric} = 0
        dDeadBucketFraction         (1,1) double {mustBeNumeric} = 0
        dGammaValue                 (1,1) double {mustBeNumeric} = 0
        dBloomingProbability        (1,1) double {mustBeNumeric} = 0
        dBloomPoissonMean           (1,1) double {mustBeNumeric} = 0
        dPartialBloomFraction       (1,1) double {mustBeNumeric} = 0
        dBrightSpotsProbability     (1,1) double {mustBeNumeric} = 0
        dBrightSpotsPoissonRate     (1,1) double {mustBeNumeric} = 0
        charBloomType               (1,:) string {mustBeMember(charBloomType, ["blob", "row", "col"])} = "blob"
    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = SImageErrorsPipelineSettings(options, strErrorParams)
            %% SIGNATURE
            % self = SImageErrorsPipelineSettings(Name=Value)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Validate and retain hosted image-error settings and legacy aliases.
            % Gain applies only to the Poisson charge model, never to metadata.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % options        Output datatype and full-scale digital number.
            % strErrorParams Named effect parameters; gain is positive e-/DN.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self  Settings preserving clean defaults and caller RNG ownership.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 18-10-2026  Pietro Califano, Codex gpt-6.1  Document charge conversion.
            % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CBaseSettings
            % -------------------------------------------------------------------------------------------------
            arguments (Input)
                options.charOutputDataType                  (1,:) string {mustBeMember(options.charOutputDataType, ...
                                                            ["input", "double", "single", "uint16", "uint8", "uint32"])} = "input"
                options.dNUM_LEVELS                         (1,1) double = 255
                strErrorParams.dMeanGenericWN               (1,1) double {mustBeNumeric} = 0
                strErrorParams.dSigmaDigitNumRelGenericWN   (1,1) double {mustBeNumeric} = 0
                strErrorParams.dSigmaGenericWNRel           (1,1) double {mustBeNumeric} = 0
                strErrorParams.dSigmaGenericWN              (1,1) double {mustBeNumeric} = 0
                strErrorParams.dSigmaGenericBlur            (1,1) double {mustBeNumeric} = 0
                strErrorParams.dGenericBlurFilterSize       (1,1) int32 {mustBeNumeric}  = 3
                strErrorParams.bAddShotNoise                (1,1) logical  = false;
                strErrorParams.charShotNoiseBackend         (1,1) string ...
                    {mustBeMember(strErrorParams.charShotNoiseBackend, ["matlab", "cpu_mex", "cuda"])} = "matlab"
                strErrorParams.dGainElectronsPerDN          (1,1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive} = 6300/255
                strErrorParams.dDeadPixelPerc               (1,1) double {mustBeNumeric} = 0
                strErrorParams.dDeadBucketFraction          (1,1) double {mustBeNumeric} = 0
                strErrorParams.dGammaValue                  (1,1) double {mustBeNumeric} = 0
                strErrorParams.dBloomingProbability         (1,1) double {mustBeNumeric} = 0
                strErrorParams.dBloomPoissonMean            (1,1) double {mustBeNumeric} = 0
                strErrorParams.dPartialBloomFraction        (1,1) double {mustBeNumeric} = 0
                strErrorParams.dBrightSpotsProbability      (1,1) double {mustBeNumeric} = 0
                strErrorParams.dBrightSpotsPoissonRate      (1,1) double {mustBeNumeric} = 0
                strErrorParams.charBloomType                (1,:) string {mustBeMember(strErrorParams.charBloomType, ...
                                                            ["blob", "row", "col"])} = "blob"
            end
            arguments (Output)
                self (1,1) SImageErrorsPipelineSettings
            end

            % TODO: add functionality to CBaseSettings
            % checkDefaultConstruction(strKwargs, self);

            % Mark the instance as non-default when any effect parameter differs from its default
            cellFields = fieldnames(strErrorParams);

            for ui32FieldIdx = uint32(1):uint32(numel(cellFields))
                charField = cellFields{ui32FieldIdx};

                if ~isequal(strErrorParams.(charField), self.(charField))
                    self.bDefaultConstructed = false;
                    break;
                end

            end

            % Assign the white-noise parameters
            self.dMeanGenericWN             = strErrorParams.dMeanGenericWN;
            self.dSigmaGenericWN            = strErrorParams.dSigmaGenericWN;
            self.dSigmaGenericWNRel         = strErrorParams.dSigmaGenericWNRel;
            self.dSigmaDigitNumRelGenericWN = strErrorParams.dSigmaDigitNumRelGenericWN;

            % Resolve the legacy white-noise aliases into the DN-relative sigma
            if self.dSigmaDigitNumRelGenericWN == 0 && self.dSigmaGenericWNRel > 0
                self.dSigmaDigitNumRelGenericWN = self.dSigmaGenericWNRel;
            end
            if self.dSigmaDigitNumRelGenericWN == 0 && self.dSigmaGenericWN > 0
                self.dSigmaDigitNumRelGenericWN = sqrt(self.dSigmaGenericWN ./ options.dNUM_LEVELS);
            end
            self.dSigmaGenericWNRel = self.dSigmaDigitNumRelGenericWN;

            % Assign the remaining error model parameters
            self.dSigmaGenericBlur          = strErrorParams.dSigmaGenericBlur;
            self.dGenericBlurFilterSize     = strErrorParams.dGenericBlurFilterSize;
            self.bAddShotNoise              = strErrorParams.bAddShotNoise;
            self.charShotNoiseBackend       = strErrorParams.charShotNoiseBackend;
            self.dGainElectronsPerDN        = strErrorParams.dGainElectronsPerDN;
            self.dDeadPixelPerc             = strErrorParams.dDeadPixelPerc;
            self.dDeadBucketFraction        = strErrorParams.dDeadBucketFraction;
            self.dGammaValue                = strErrorParams.dGammaValue;
            self.dBloomingProbability       = strErrorParams.dBloomingProbability;
            self.dBloomPoissonMean          = strErrorParams.dBloomPoissonMean;
            self.dPartialBloomFraction      = strErrorParams.dPartialBloomFraction;
            self.dBrightSpotsProbability    = strErrorParams.dBrightSpotsProbability;
            self.dBrightSpotsPoissonRate    = strErrorParams.dBrightSpotsPoissonRate;
            self.charBloomType              = strErrorParams.charBloomType;

            % Assign settings
            self.charOutputDataType         = options.charOutputDataType;
            self.dNUM_LEVELS                = options.dNUM_LEVELS;

        end

        % GETTERS

        % SETTERS

        % METHODS
    end


    methods (Access = protected)


    end

    % methods (Access=private)
    %
    % end

    % methods (Access=public, Static)
    %
    % end

    % methods (Abstract, Access=public)
    % [x,y] = abstract_function_name(args) NOTE: number of args matter.
    % end
end
