classdef CTrajectoryArcAnalyzer < handle
    %% SIGNATURE
    % CTrajectoryArcAnalyzer Build segmented trajectory analyses from kernels or state histories.
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % This class extracts manoeuvre-separated phases from either:
    %   1) a sampled state history [r(t), v(t)], or
    %   2) loaded SPICE kernels sampled through cspice_spkezr.
    %
    % Automatic phase detection is based on velocity discontinuities.
    % Manual phase tables are also supported and validated explicitly.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 16-04-2026    Pietro Califano, Codex     First generalized implementation for kernel arc analysis.
    % -------------------------------------------------------------------------------------------------------------

    properties (Access = public)
        dGroupingGap_s (1,1) double {mustBePositive} = 6 * 3600
        dMinVelocityJumpThreshold (1,1) double {mustBePositive} = 1.0e-6
        dMedianThresholdScale (1,1) double {mustBePositive} = 100.0
        dQuantileThresholdScale (1,1) double {mustBePositive} = 10.0
        dQuantilePercentile (1,1) double {mustBeGreaterThan(dQuantilePercentile, 0), mustBeLessThan(dQuantilePercentile, 100)} = 99.9
        dDefaultSamplingStep_s (1,1) double {mustBePositive} = 300.0
        dDefaultDetectionStep_s (1,1) double {mustBePositive} = 60.0
    end

    methods
        function self = CTrajectoryArcAnalyzer(options)
            arguments
                options.dGroupingGap_s (1,1) double {mustBePositive} = 6 * 3600
                options.dMinVelocityJumpThreshold (1,1) double {mustBePositive} = 1.0e-6
                options.dMedianThresholdScale (1,1) double {mustBePositive} = 100.0
                options.dQuantileThresholdScale (1,1) double {mustBePositive} = 10.0
                options.dQuantilePercentile (1,1) double {mustBeGreaterThan(options.dQuantilePercentile, 0), mustBeLessThan(options.dQuantilePercentile, 100)} = 99.9
                options.dDefaultSamplingStep_s (1,1) double {mustBePositive} = 300.0
                options.dDefaultDetectionStep_s (1,1) double {mustBePositive} = 60.0
            end

            self.dGroupingGap_s = options.dGroupingGap_s;
            self.dMinVelocityJumpThreshold = options.dMinVelocityJumpThreshold;
            self.dMedianThresholdScale = options.dMedianThresholdScale;
            self.dQuantileThresholdScale = options.dQuantileThresholdScale;
            self.dQuantilePercentile = options.dQuantilePercentile;
            self.dDefaultSamplingStep_s = options.dDefaultSamplingStep_s;
            self.dDefaultDetectionStep_s = options.dDefaultDetectionStep_s;
        end

        function strAnalysis = analyzeStateHistory(self, dTimestamps, dPos_W, dVel_W, options)
            arguments
                self
                dTimestamps (1,:) double {mustBeNumeric}
                dPos_W {mustBeNumeric}
                dVel_W {mustBeNumeric}
                options.dSunPosition_W double = zeros(0, 0)
                options.strPhaseTable struct = struct('charName', {}, 'dStartTime', {}, 'dEndTime', {})
                options.bUseManualPhases (1,1) logical = false
                options.dVelocityJumpThreshold (1,1) double = NaN
            end

            [dPos_W, dVel_W, dSunPosition_W] = self.normalizeStateInputs_(dTimestamps, dPos_W, dVel_W, options.dSunPosition_W);
            self.validateTimegrid_(dTimestamps);

            if options.bUseManualPhases || ~isempty(options.strPhaseTable)
                strPhases = self.validatePhaseTable_(options.strPhaseTable, dTimestamps);
                dManeuverTimes = [strPhases(1:end-1).dEndTime]';
                dManeuverDeltaV_W = self.computeBoundaryDeltaV_(dTimestamps, dVel_W, strPhases);
            else
                [dManeuverTimes, dManeuverDeltaV_W] = self.detectManoeuvreTimes_(dTimestamps, dVel_W, options.dVelocityJumpThreshold);
                strPhases = self.buildPhasesFromManoeuvres_(dTimestamps, dManeuverTimes);
            end

            strAnalysis = self.buildAnalysisStruct_( ...
                dTimestamps, dPos_W, dVel_W, dSunPosition_W, ...
                dManeuverTimes, dManeuverDeltaV_W, strPhases);
        end

        function strAnalysis = analyzeKernelTrajectory(self, varTargetID, options)
            arguments
                self
                varTargetID {mustBeA(varTargetID, ["string", "char", "double", "int32", "uint32", "single"])}
                options.dTimegrid (1,:) double = zeros(1, 0)
                options.charFrame (1,1) string {mustBeText} = "J2000"
                options.varObserver {mustBeA(options.varObserver, ["string", "char", "double", "int32", "uint32", "single"])} = "EARTH"
                options.charCorrection (1,1) string {mustBeText} = "NONE"
                options.varPhaseAngleObserver = []
                options.dCoverageIntervals (:,2) double = zeros(0, 2)
                options.charCoverageKernelPath (1,:) char = ''
                options.ui32MaxNumIntervals (1,1) uint32 = uint32(1000)
                options.dSamplingStep_s (1,1) double {mustBePositive} = NaN
                options.dDetectionStep_s (1,1) double {mustBePositive} = NaN
                options.strPhaseTable struct = struct('charName', {}, 'dStartTime', {}, 'dEndTime', {})
                options.bUseManualPhases (1,1) logical = false
                options.dVelocityJumpThreshold (1,1) double = NaN
            end

            dCoverageIntervals = options.dCoverageIntervals;
            if isempty(dCoverageIntervals)
                dCoverageIntervals = self.resolveCoverageIntervals_(varTargetID, options.charCoverageKernelPath, options.ui32MaxNumIntervals, options.dTimegrid);
            end

            dSamplingStep_s = options.dSamplingStep_s;
            if isnan(dSamplingStep_s)
                dSamplingStep_s = self.dDefaultSamplingStep_s;
            end

            dTimegrid = options.dTimegrid;
            if isempty(dTimegrid)
                dTimegrid = self.buildTimegridFromCoverage_(dCoverageIntervals, dSamplingStep_s);
            end
            self.validateTimegrid_(dTimegrid);

            dState_W = cspice_spkezr( ...
                self.formatSpiceId_(varTargetID), ...
                dTimegrid, ...
                char(options.charFrame), ...
                char(options.charCorrection), ...
                self.formatSpiceId_(options.varObserver));

            dSunPosition_W = zeros(0, 0);
            varPhaseAngleObserver = options.varPhaseAngleObserver;
            if isempty(varPhaseAngleObserver)
                varPhaseAngleObserver = options.varObserver;
            end

            if ~isempty(varPhaseAngleObserver)
                dSunPosition_W = cspice_spkpos( ...
                    'SUN', ...
                    dTimegrid, ...
                    char(options.charFrame), ...
                    char(options.charCorrection), ...
                    self.formatSpiceId_(varPhaseAngleObserver));
            end

            if options.bUseManualPhases || ~isempty(options.strPhaseTable)
                strPhases = self.validatePhaseTable_(options.strPhaseTable, dTimegrid);
                dManeuverTimes = [strPhases(1:end-1).dEndTime]';
                dManeuverDeltaV_W = self.computeBoundaryDeltaV_(dTimegrid, dState_W(4:6, :), strPhases);
            else
                dDetectionStep_s = options.dDetectionStep_s;
                if isnan(dDetectionStep_s)
                    dDetectionStep_s = self.dDefaultDetectionStep_s;
                end

                dDetectionTimegrid = self.buildTimegridFromCoverage_(dCoverageIntervals, dDetectionStep_s);
                dDetectionState_W = cspice_spkezr( ...
                    self.formatSpiceId_(varTargetID), ...
                    dDetectionTimegrid, ...
                    char(options.charFrame), ...
                    char(options.charCorrection), ...
                    self.formatSpiceId_(options.varObserver));

                [dManeuverTimes, dManeuverDeltaV_W] = self.detectManoeuvreTimes_( ...
                    dDetectionTimegrid, ...
                    dDetectionState_W(4:6, :), ...
                    options.dVelocityJumpThreshold);
                strPhases = self.buildPhasesFromManoeuvres_(dTimegrid, dManeuverTimes);
            end

            strAnalysis = self.buildAnalysisStruct_( ...
                dTimegrid, ...
                dState_W(1:3, :), ...
                dState_W(4:6, :), ...
                dSunPosition_W, ...
                dManeuverTimes, ...
                dManeuverDeltaV_W, ...
                strPhases);
            strAnalysis.dCoverageIntervals = dCoverageIntervals;
        end
    end

    methods (Access = private)
        function strAnalysis = buildAnalysisStruct_(self, dTimestamps, dPos_W, dVel_W, dSunPosition_W, dManeuverTimes, dManeuverDeltaV_W, strPhases)
            dNumPhases = numel(strPhases);
            dPhaseColors = turbo(max(dNumPhases, 2));
            dPhaseColors = dPhaseColors(1:dNumPhases, :);
            dRange = vecnorm(dPos_W, 2, 1);

            dPhaseAngleDeg = zeros(0, numel(dTimestamps));
            if ~isempty(dSunPosition_W)
                dPhaseAngleDeg = self.computePhaseAngleDeg_(dPos_W, dSunPosition_W);
            end

            dPhaseIntervals = zeros(dNumPhases, 2);
            dBoundaryIdx = zeros(1, max(dNumPhases - 1, 0));
            cellPhaseLabels = cell(1, dNumPhases);

            for idPhase = 1:dNumPhases
                dPhaseIntervals(idPhase, :) = [strPhases(idPhase).dStartTime, strPhases(idPhase).dEndTime];
                strPhases(idPhase).dStartIdx = self.closestIdx_(dTimestamps, strPhases(idPhase).dStartTime);
                strPhases(idPhase).dEndIdx = self.closestIdx_(dTimestamps, strPhases(idPhase).dEndTime);
                cellPhaseLabels{idPhase} = char(strPhases(idPhase).charName);

                if idPhase > 1
                    dBoundaryIdx(idPhase - 1) = strPhases(idPhase).dStartIdx;
                end
            end

            strAnalysis = struct();
            strAnalysis.dTimestamps = dTimestamps(:)';
            strAnalysis.dRelativeTime_s = dTimestamps(:)' - dTimestamps(1);
            strAnalysis.dPos_W = dPos_W;
            strAnalysis.dVel_W = dVel_W;
            strAnalysis.dSunPosition_W = dSunPosition_W;
            strAnalysis.dRange = dRange;
            strAnalysis.dPhaseAngleDeg = dPhaseAngleDeg;
            strAnalysis.dManeuverTimes = dManeuverTimes(:);
            strAnalysis.dManeuverDeltaV_W = dManeuverDeltaV_W;
            strAnalysis.strPhases = strPhases;
            strAnalysis.dPhaseIntervals = dPhaseIntervals;
            strAnalysis.dPhaseColors = dPhaseColors;
            strAnalysis.dBoundaryIdx = dBoundaryIdx;
            strAnalysis.cellPhaseLabels = cellPhaseLabels;
        end

        function [dPos_W, dVel_W, dSunPosition_W] = normalizeStateInputs_(self, dTimestamps, dPos_W, dVel_W, dSunPosition_W)
            dPos_W = self.normalizeVectorSamples_(dPos_W, 'dPos_W');
            dVel_W = self.normalizeVectorSamples_(dVel_W, 'dVel_W');
            assert(size(dPos_W, 2) == numel(dTimestamps), 'dPos_W length must match dTimestamps.');
            assert(size(dVel_W, 2) == numel(dTimestamps), 'dVel_W length must match dTimestamps.');

            if ~isempty(dSunPosition_W)
                dSunPosition_W = self.normalizeVectorSamples_(dSunPosition_W, 'dSunPosition_W');
                assert(size(dSunPosition_W, 2) == numel(dTimestamps), 'dSunPosition_W length must match dTimestamps.');
            end
        end

        function dSamples = normalizeVectorSamples_(self, dSamples, charName) %#ok<INUSD>
            if size(dSamples, 1) ~= 3 && size(dSamples, 2) == 3
                dSamples = dSamples';
            end

            assert(size(dSamples, 1) == 3, '%s must have size 3xN or Nx3.', charName);
        end

        function validateTimegrid_(self, dTimestamps) %#ok<INUSD>
            assert(isvector(dTimestamps) && numel(dTimestamps) >= 2, 'dTimestamps must contain at least two samples.');
            assert(all(diff(dTimestamps) > 0), 'dTimestamps must be strictly increasing.');
        end

        function [dManeuverTimes, dManeuverDeltaV_W] = detectManoeuvreTimes_(self, dTimestamps, dVel_W, dVelocityJumpThreshold)
            dVelocityJump = vecnorm(diff(dVel_W, 1, 2), 2, 1);

            if isempty(dVelocityJump)
                dManeuverTimes = zeros(0, 1);
                dManeuverDeltaV_W = zeros(3, 0);
                return
            end

            if isnan(dVelocityJumpThreshold)
                dVelocityJumpThreshold = max([ ...
                    self.dMinVelocityJumpThreshold, ...
                    self.dMedianThresholdScale * median(dVelocityJump), ...
                    self.dQuantileThresholdScale * prctile(dVelocityJump, self.dQuantilePercentile) ...
                    ]);
            end

            dCandidateIdx = find(dVelocityJump > dVelocityJumpThreshold);
            if isempty(dCandidateIdx)
                dManeuverTimes = zeros(0, 1);
                dManeuverDeltaV_W = zeros(3, 0);
                return
            end

            dCandidateTimes = 0.5 * (dTimestamps(dCandidateIdx) + dTimestamps(dCandidateIdx + 1));
            dGroupStartIdx = [1, find(diff(dCandidateTimes) > self.dGroupingGap_s) + 1];
            dManeuverTimes = zeros(numel(dGroupStartIdx), 1);
            dManeuverDeltaV_W = zeros(3, numel(dGroupStartIdx));

            for idGroup = 1:numel(dGroupStartIdx)
                dIdxStart = dGroupStartIdx(idGroup);
                if idGroup < numel(dGroupStartIdx)
                    dIdxEnd = dGroupStartIdx(idGroup + 1) - 1;
                else
                    dIdxEnd = numel(dCandidateIdx);
                end

                [~, dLocalPeakIdx] = max(dVelocityJump(dCandidateIdx(dIdxStart:dIdxEnd)));
                dPeakIdx = dCandidateIdx(dIdxStart + dLocalPeakIdx - 1);
                dManeuverTimes(idGroup) = 0.5 * (dTimestamps(dPeakIdx) + dTimestamps(dPeakIdx + 1));
                dManeuverDeltaV_W(:, idGroup) = dVel_W(:, dPeakIdx + 1) - dVel_W(:, dPeakIdx);
            end
        end

        function strPhases = buildPhasesFromManoeuvres_(self, dTimestamps, dManeuverTimes) %#ok<INUSD>
            dBoundaries = [dTimestamps(1); dManeuverTimes(:); dTimestamps(end)];
            dBoundaries = unique(dBoundaries, 'stable');

            dNumPhases = numel(dBoundaries) - 1;
            strPhases = repmat(struct('charName', "", 'dStartTime', 0, 'dEndTime', 0), dNumPhases, 1);
            for idPhase = 1:dNumPhases
                strPhases(idPhase).charName = sprintf('Phase %d', idPhase - 1);
                strPhases(idPhase).dStartTime = dBoundaries(idPhase);
                strPhases(idPhase).dEndTime = dBoundaries(idPhase + 1);
            end
        end

        function strPhases = validatePhaseTable_(self, strPhaseTable, dTimestamps) %#ok<INUSD>
            assert(~isempty(strPhaseTable), 'A non-empty strPhaseTable must be provided when manual phases are enabled.');
            assert(all(isfield(strPhaseTable, {'dStartTime', 'dEndTime'})), 'strPhaseTable must contain dStartTime and dEndTime fields.');

            dStartTimes = [strPhaseTable.dStartTime]';
            dEndTimes = [strPhaseTable.dEndTime]';

            assert(all(dEndTimes > dStartTimes), 'Each manual phase must satisfy dEndTime > dStartTime.');
            assert(all(diff(dStartTimes) >= 0), 'Manual phases must be sorted by start time.');
            assert(all(dStartTimes(2:end) >= dEndTimes(1:end-1)), 'Manual phases must not overlap.');
            assert(dStartTimes(1) >= dTimestamps(1) && dEndTimes(end) <= dTimestamps(end), ...
                'Manual phases must remain inside the analyzed time span.');

            strPhases = strPhaseTable;
            for idPhase = 1:numel(strPhases)
                if ~isfield(strPhases, 'charName') || strlength(string(strPhases(idPhase).charName)) == 0
                    strPhases(idPhase).charName = sprintf('Phase %d', idPhase - 1);
                else
                    strPhases(idPhase).charName = string(strPhases(idPhase).charName);
                end
            end
        end

        function dManeuverDeltaV_W = computeBoundaryDeltaV_(self, dTimestamps, dVel_W, strPhases) %#ok<INUSD>
            dNumMans = max(numel(strPhases) - 1, 0);
            dManeuverDeltaV_W = zeros(3, dNumMans);

            for idMan = 1:dNumMans
                dBoundaryIdx = self.closestIdx_(dTimestamps, strPhases(idMan + 1).dStartTime);
                dPreIdx = max(dBoundaryIdx - 1, 1);
                dManeuverDeltaV_W(:, idMan) = dVel_W(:, dBoundaryIdx) - dVel_W(:, dPreIdx);
            end
        end

        function dCoverageIntervals = resolveCoverageIntervals_(self, varTargetID, charCoverageKernelPath, ui32MaxNumIntervals, dTimegrid)
            if ~isempty(charCoverageKernelPath)
                dCover = cspice_spkcov(charCoverageKernelPath, self.normalizeTargetId_(varTargetID), double(ui32MaxNumIntervals));
                assert(~isempty(dCover), 'No SPK coverage was found in the specified kernel.');
                dCoverageIntervals = reshape(dCover(:), 2, [])';
                return
            end

            if ~isempty(dTimegrid)
                dCoverageIntervals = [dTimegrid(1), dTimegrid(end)];
                return
            end

            error('Coverage intervals are required. Provide dCoverageIntervals, charCoverageKernelPath, or dTimegrid.');
        end

        function dTimegrid = buildTimegridFromCoverage_(self, dCoverageIntervals, dDeltaTime_s) %#ok<INUSD>
            dTimegrid = zeros(1, 0);
            for idInterval = 1:size(dCoverageIntervals, 1)
                dCurrent = dCoverageIntervals(idInterval, 1):dDeltaTime_s:dCoverageIntervals(idInterval, 2);
                if isempty(dCurrent) || dCurrent(end) < dCoverageIntervals(idInterval, 2)
                    dCurrent = [dCurrent, dCoverageIntervals(idInterval, 2)]; %#ok<AGROW>
                end

                if isempty(dTimegrid)
                    dTimegrid = dCurrent;
                else
                    dTimegrid = [dTimegrid, dCurrent(dCurrent > dTimegrid(end))]; %#ok<AGROW>
                end
            end
        end

        function dPhaseAngleDeg = computePhaseAngleDeg_(self, dPos_W, dSunPosition_W) %#ok<INUSD>
            dCosAngle = dot(dPos_W, dSunPosition_W, 1) ./ ...
                max(vecnorm(dPos_W, 2, 1) .* vecnorm(dSunPosition_W, 2, 1), eps);
            dCosAngle = min(max(dCosAngle, -1.0), 1.0);
            dPhaseAngleDeg = acosd(dCosAngle);
        end

        function dIdx = closestIdx_(self, dTimestamps, dTime) %#ok<INUSD>
            [~, dIdx] = min(abs(dTimestamps - dTime));
        end

        function charTargetID = formatSpiceId_(self, varTargetID) %#ok<INUSD>
            if isstring(varTargetID) || ischar(varTargetID)
                charTargetID = char(string(varTargetID));
            else
                charTargetID = num2str(double(varTargetID));
            end
        end

        function i32TargetID = normalizeTargetId_(self, varTargetID) %#ok<INUSD>
            if isnumeric(varTargetID)
                i32TargetID = int32(varTargetID);
            else
                i32TargetID = int32(str2double(string(varTargetID)));
            end
        end
    end
end
