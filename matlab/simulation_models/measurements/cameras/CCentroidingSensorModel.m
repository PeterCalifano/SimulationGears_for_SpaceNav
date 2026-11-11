classdef CCentroidingSensorModel
    %% SIGNATURE
    % self = CCentroidingSensorModel(Name=Value)
    % [self, dMeasurement, bValid, strDiagnostic] = self.step(dTimestamp, ...
    %     dTargetPosition_CAM, dCalibration, dWhiteDraw, dBiasDraw)
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Camera centroiding sensor model with caller-supplied standard-normal draws. It projects the
    % target position in the camera frame through a pinhole calibration, then adds white, constant
    % and correlated pixel errors. The model holds no RNG and no persistent state: step returns the
    % updated object by value. It is a simulation model, not an image-processing algorithm.
    %
    % The correlated bias follows db = -b/tau dt + sigma dW, discretized exactly over the elapsed
    % acquisition time, and starts at zero on the first step. Unlike the former per-update
    % recurrence, it depends on elapsed time, not on the number of steps; runs without correlated
    % bias match the former model. See RunCentroidingSensorExample.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % Constructor, name-value, one value per image axis:
    %   dWhiteNoiseSigma     (2,1) white noise standard deviation [px]
    %   dConstantBias        (2,1) constant bias [px]
    %   dBiasDrivingSigma    (2,1) diffusion amplitude of the correlated bias [px/sqrt(s)]
    %   dBiasTimeConstant    (2,1) correlation time [s], positive on every axis with a driving sigma
    % step:
    %   dTimestamp           (1,1) acquisition time [s], strictly increasing
    %   dTargetPosition_CAM  (3,1) target position in the camera frame, any length unit
    %   dCalibration         (3,3) pinhole matrix with bottom row [0, 0, 1]
    %   dWhiteDraw           (2,1) standard-normal draw for the white noise
    %   dBiasDraw            (2,1) independent standard-normal draw for the correlated bias
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % self           updated sensor; a failed input or timestamp check leaves the caller's object as it was
    % dMeasurement   (2,1) pixel coordinates; NaN when the target is not in front of the camera
    % bValid         true when the target is in front of the camera and the measurement is finite
    % strDiagnostic  truth projection, correlated bias, bias decay and bias innovation variance
    % The correlated bias also evolves on steps with an invalid projection.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 12-10-2026  Pietro Califano, Codex gpt-6.1  Own explicit-draw centroid sensor state.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; rename from CCentroidSensorModel.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % MATLAB only.
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = private)
        dWhiteNoiseSigma (2,1) double = zeros(2, 1)
        dConstantBias (2,1) double = zeros(2, 1)
        dBiasDrivingSigma (2,1) double = zeros(2, 1)
        dBiasTimeConstant (2,1) double = zeros(2, 1)
        dCorrelatedBias (2,1) double = zeros(2, 1)
        dLastTimestamp (1,1) double = -Inf
        bInitialized (1,1) logical = false
    end

    methods
        function self = CCentroidingSensorModel(options)
            % Require a positive time constant on every axis with a driving sigma.
            arguments (Input)
                options.dWhiteNoiseSigma (2,1) double {mustBeReal, mustBeFinite, mustBeNonnegative} = zeros(2, 1)
                options.dConstantBias (2,1) double {mustBeReal, mustBeFinite} = zeros(2, 1)
                options.dBiasDrivingSigma (2,1) double {mustBeReal, mustBeFinite, mustBeNonnegative} = zeros(2, 1)
                options.dBiasTimeConstant (2,1) double {mustBeReal, mustBeFinite, mustBeNonnegative} = zeros(2, 1)
            end

            if any(options.dBiasDrivingSigma > 0 & options.dBiasTimeConstant <= 0)
                error('CCentroidingSensorModel:InvalidBiasTimeConstant', ...
                    'Each enabled centroid bias axis requires a positive time constant in seconds.');
            end

            self.dWhiteNoiseSigma = options.dWhiteNoiseSigma;
            self.dConstantBias = options.dConstantBias;
            self.dBiasDrivingSigma = options.dBiasDrivingSigma;
            self.dBiasTimeConstant = options.dBiasTimeConstant;
        end

        function [self, dMeasurement, bValid, strDiagnostic] = step(self, ...
                dTimestamp, dTargetPosition_CAM, dCalibration, dWhiteDraw, dBiasDraw)
            % Advance one acquisition using supplied draws; never sample an RNG.
            arguments (Input)
                self (1,1) CCentroidingSensorModel
                dTimestamp (1,1) double {mustBeReal, mustBeFinite}
                dTargetPosition_CAM (3,1) double {mustBeReal, mustBeFinite}
                dCalibration (3,3) double {mustBeReal, mustBeFinite}
                dWhiteDraw (2,1) double {mustBeReal, mustBeFinite}
                dBiasDraw (2,1) double {mustBeReal, mustBeFinite}
            end
            arguments (Output)
                self (1,1) CCentroidingSensorModel
                dMeasurement (2,1) double
                bValid (1,1) logical
                strDiagnostic (1,1) struct
            end

            % Check the timestamp order and the pinhole calibration before changing state
            if self.bInitialized && dTimestamp <= self.dLastTimestamp
                error('CCentroidingSensorModel:NonMonotonicTimestamp', ...
                    'Centroiding sensor acquisitions must have strictly increasing timestamps.');
            end

            if dCalibration(1, 1) <= 0 || dCalibration(2, 2) <= 0 || ...
                    any(dCalibration(3, :) ~= [0, 0, 1])
                error('CCentroidingSensorModel:InvalidCalibration', ...
                    'Pinhole calibration requires positive focal lengths and bottom row [0,0,1].');
            end

            % Propagate each enabled axis with the exact OU transition; expm1 keeps the variance
            % accurate when the elapsed time is small against the time constant.
            dDecay = ones(2, 1);
            dBiasVariance = zeros(2, 1);
            if self.bInitialized
                dElapsed = dTimestamp - self.dLastTimestamp;
                bEnabled = self.dBiasDrivingSigma > 0;
                dTau = self.dBiasTimeConstant(bEnabled);

                dDecay(bEnabled) = exp(-dElapsed./dTau);
                dBiasVariance(bEnabled) = self.dBiasDrivingSigma(bEnabled).^2 ...
                    .*dTau./2.*(-expm1(-2*dElapsed./dTau));
                self.dCorrelatedBias = dDecay.*self.dCorrelatedBias ...
                    + sqrt(dBiasVariance).*dBiasDraw;
            end
            self.dLastTimestamp = dTimestamp;
            self.bInitialized = true;

            % Deliver a measurement only for a target in front of the camera; the bias above
            % evolves either way.
            dProjection = NaN(2, 1);
            dMeasurement = NaN(2, 1);
            bValid = dTargetPosition_CAM(3) > 0;
            if bValid
                dHomogeneous = dCalibration*dTargetPosition_CAM;
                dProjection = dHomogeneous(1:2)./dHomogeneous(3);
                dMeasurement = dProjection + self.dConstantBias ...
                    + self.dWhiteNoiseSigma.*dWhiteDraw + self.dCorrelatedBias;
                bValid = all(isfinite(dMeasurement));
            end

            % Report the truth projection and the bias transition of this step
            strDiagnostic = struct('dProjection', dProjection, ...
                'dCorrelatedBias', self.dCorrelatedBias, ...
                'dBiasDecay', dDecay, ...
                'dBiasVariance', dBiasVariance);
        end

        function self = reset(self)
            % Start a new bias history and keep the noise settings.
            arguments (Input)
                self (1,1) CCentroidingSensorModel
            end
            arguments (Output)
                self (1,1) CCentroidingSensorModel
            end

            self.dCorrelatedBias = zeros(2, 1);
            self.dLastTimestamp = -Inf;
            self.bInitialized = false;
        end
    end
end
