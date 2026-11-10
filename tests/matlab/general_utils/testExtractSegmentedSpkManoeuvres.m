classdef testExtractSegmentedSpkManoeuvres < matlab.unittest.TestCase
    %% DESCRIPTION
    % Verify strict impulse extraction from a segmented SPK. Synthetic type-9
    % kernels exercise coverage and boundary failures without external assets;
    % the optional RCS-1 fixture checks the delivered Phase-D trajectory.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 27-09-2026  Pietro Califano, Codex gpt-6  Add segmented-SPK extractor contract tests.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % ExtractSegmentedSpkManoeuvres, MICE, TemporaryFolderFixture.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testFourArcsProduceThreeImpulses(self)
            %% SIGNATURE
            % testFourArcsProduceThreeImpulses(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Check exact arc and impulse shapes, metadata, and endpoint states.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % WriteFourArcSpk_, ExtractSegmentedSpkManoeuvres.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_spkw09'), 'MICE is required.');
            self.addTeardown(@cspice_kclear);
            cspice_kclear();
            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            charSpkFile = fullfile(objFixture.Folder, 'four_arcs.bsp');
            dExpectedBurns = [1e-4, 0, 0; 0, -2e-4, 0; 0, 0, 3e-4];
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charSpkFile, [1e-4, 1e-4, 1e-4], dExpectedBurns, 0);
            cspice_furnsh(charSpkFile);

            strArcs = ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                ui32ExpectedArcCount=uint32(4), dExpectedCenterId=0, ...
                charExpectedSegmentFrame='J2000');
            self.verifySize(strArcs.dArcBounds, [2, 4]);
            self.verifySize(strArcs.dBurnDeltaV, [3, 3]);
            self.verifyEqual(strArcs.dBurnDeltaV, dExpectedBurns, ...
                'AbsTol', 1e-12);
            self.verifyEqual(strArcs.i32ArcCenterIds, ...
                zeros(1, 4, 'int32'));
            self.verifyEqual(strArcs.i32ArcFrameIds, ...
                ones(1, 4, 'int32'));
            self.verifyEqual(strArcs.i32ArcSegmentTypes, ...
                9 * ones(1, 4, 'int32'));
            self.verifyLessThan(max(vecnorm( ...
                strArcs.dBurnPostStates(1:3, :) - ...
                strArcs.dBurnPreStates(1:3, :), 2, 1)), 1e-6);
        end

        function testRejectGapsDiscontinuitiesAndMissingBurns(self)
            %% SIGNATURE
            % testRejectGapsDiscontinuitiesAndMissingBurns(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Reject boundaries that cannot safely represent impulses.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % WriteFourArcSpk_, ExtractSegmentedSpkManoeuvres.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_spkw09'), 'MICE is required.');
            self.addTeardown(@cspice_kclear);
            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            dExpectedBurns = [1e-4, 0, 0; 0, -2e-4, 0; 0, 0, 3e-4];

            charGapFile = fullfile(objFixture.Folder, 'long_gap.bsp');
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charGapFile, [1e-4, 0.5, 1e-4], dExpectedBurns, 0);
            cspice_kclear();
            cspice_furnsh(charGapFile);
            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charGapFile, int32(-100001), int32(0), 'J2000'), ...
                'ExtractSegmentedSpkManoeuvres:InvalidBoundaryGap');

            charJumpFile = fullfile(objFixture.Folder, 'position_jump.bsp');
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charJumpFile, [1e-4, 1e-4, 1e-4], dExpectedBurns, 1e-3);
            cspice_kclear();
            cspice_furnsh(charJumpFile);
            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charJumpFile, int32(-100001), int32(0), 'J2000'), ...
                'ExtractSegmentedSpkManoeuvres:PositionDiscontinuity');

            charNoBurnFile = fullfile(objFixture.Folder, 'no_burn.bsp');
            dMissingBurns = dExpectedBurns;
            dMissingBurns(:, 2) = 0;
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charNoBurnFile, [1e-4, 1e-4, 1e-4], dMissingBurns, 0);
            cspice_kclear();
            cspice_furnsh(charNoBurnFile);
            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charNoBurnFile, int32(-100001), int32(0), 'J2000'), ...
                'ExtractSegmentedSpkManoeuvres:MissingBurn');
        end

        function testRejectWrongMetadataAndAmbiguousSource(self)
            %% SIGNATURE
            % testRejectWrongMetadataAndAmbiguousSource(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Keep declared segment identity and active SPICE-pool priority explicit.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % WriteFourArcSpk_, ExtractSegmentedSpkManoeuvres.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_spkw09'), 'MICE is required.');
            self.addTeardown(@cspice_kclear);
            cspice_kclear();
            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            dExpectedBurns = [1e-4, 0, 0; 0, -2e-4, 0; 0, 0, 3e-4];
            charSpkFile = fullfile(objFixture.Folder, 'first.bsp');
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charSpkFile, [1e-4, 1e-4, 1e-4], dExpectedBurns, 0);

            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-100001), int32(0), 'J2000'), ...
                'ExtractSegmentedSpkManoeuvres:SourceNotLoaded');
            cspice_furnsh(charSpkFile);
            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                ui32ExpectedArcCount=uint32(3)), ...
                'ExtractSegmentedSpkManoeuvres:UnexpectedArcCount');
            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                dExpectedCenterId=int32(1)), ...
                'ExtractSegmentedSpkManoeuvres:SegmentMetadataMismatch');

            charPriorityFile = fullfile(objFixture.Folder, 'higher_priority.bsp');
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charPriorityFile, [1e-4, 1e-4, 1e-4], dExpectedBurns, 0);
            cspice_furnsh(charPriorityFile);
            self.verifyError(@() ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-100001), int32(0), 'J2000'), ...
                'ExtractSegmentedSpkManoeuvres:AmbiguousSource');
        end

        function testReferenceEvaluationSelectsBoundarySide(self)
            %% SIGNATURE
            % testReferenceEvaluationSelectsBoundarySide(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Keep filter timestamps intact while selecting the action-ordered
            % pre- or post-burn SPK endpoint for one nominal state.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % WriteFourArcSpk_, ExtractSegmentedSpkManoeuvres,
            % EvaluateSegmentedSpkState.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_spkw09'), 'MICE is required.');
            self.addTeardown(@cspice_kclear);
            cspice_kclear();
            objFixture = self.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            charSpkFile = fullfile(objFixture.Folder, 'reference_arcs.bsp');
            dExpectedBurns = [1e-4, 0, 0; 0, -2e-4, 0; 0, 0, 3e-4];
            testExtractSegmentedSpkManoeuvres.WriteFourArcSpk_( ...
                charSpkFile, [1e-4, 1e-4, 1e-4], dExpectedBurns, 0);
            cspice_furnsh(charSpkFile);
            strArcs = ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-100001), int32(0), 'J2000');

            [dOffGridState, strOffGridEvaluation] = EvaluateSegmentedSpkState( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                42.5, strArcs.dArcBounds);
            self.verifyEqual(dOffGridState(1), 1.0425, 'AbsTol', 1e-12);
            self.verifyEqual(strOffGridEvaluation.ui32ArcIndex, uint32(1));
            self.verifyFalse(strOffGridEvaluation.bBoundary);

            dBurnEpoch = strArcs.dBurnTimestamps(1);
            [dPreState, strPreEvaluation] = EvaluateSegmentedSpkState( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                dBurnEpoch, strArcs.dArcBounds, charBoundarySide='pre');
            [dPostState, strPostEvaluation] = EvaluateSegmentedSpkState( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                dBurnEpoch, strArcs.dArcBounds, charBoundarySide='post');
            self.verifyEqual(dPostState(4:6) - dPreState(4:6), ...
                dExpectedBurns(:, 1), 'AbsTol', 1e-12);
            self.verifyEqual(strPreEvaluation.dRequestedEpoch, dBurnEpoch);
            self.verifyEqual(strPreEvaluation.dEvaluatedEpoch, ...
                strArcs.dArcBounds(2, 1));
            self.verifyEqual(strPostEvaluation.dEvaluatedEpoch, dBurnEpoch);
            self.verifyTrue(strPreEvaluation.bBoundary);
            self.verifyEqual(strPostEvaluation.ui32ArcIndex, uint32(2));

            dGapEpoch = mean([strArcs.dArcBounds(2, 1), dBurnEpoch]);
            self.verifyError(@() EvaluateSegmentedSpkState( ...
                charSpkFile, int32(-100001), int32(0), 'J2000', ...
                dGapEpoch, strArcs.dArcBounds), ...
                'EvaluateSegmentedSpkState:OutsideCoverage');
        end

        function testPhaseDAssetWhenConfigured(self)
            %% SIGNATURE
            % testPhaseDAssetWhenConfigured(self)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare extracted burns with the delivered four-arc Phase-D SPK.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB unit-test instance.
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % ExtractSegmentedSpkManoeuvres, RCS1_PHASE_D_KERNEL_ROOT.
            % ---------------------------------------------------------------------------------------------------------

            self.assumeNotEmpty(which('cspice_spkw09'), 'MICE is required.');
            charKernelRoot = getenv('RCS1_PHASE_D_KERNEL_ROOT');
            self.assumeTrue(isfolder(charKernelRoot), ...
                'Set RCS1_PHASE_D_KERNEL_ROOT to the external phase-D kernels directory.');
            self.addTeardown(@cspice_kclear);
            cspice_kclear();

            % Load relative PATH_VALUES from the metakernel folder and restore
            % the caller's folder after the installed-asset check.
            charOriginalFolder = pwd;
            objFolderCleanup = onCleanup(@() cd(charOriginalFolder)); %#ok<NASGU>
            cd(fullfile(charKernelRoot, 'mk'));
            cspice_furnsh('kernels.mk');
            charSpkFile = fullfile(charKernelRoot, 'trajectories', ...
                'RCS1_dep2PrEPSSTO_v7.bsp');
            cspice_furnsh(charSpkFile);
            strArcs = ExtractSegmentedSpkManoeuvres( ...
                charSpkFile, int32(-19920605), int32(20099942), 'J2000', ...
                ui32ExpectedArcCount=uint32(4), ...
                dExpectedCenterId=20099942, ...
                charExpectedSegmentFrame='J2000');

            self.verifyEqual(strArcs.dArcBounds(1, 1), ...
                920815267.930794, 'AbsTol', 1e-6);
            self.verifyEqual(strArcs.dBurnTimestamps, ...
                [921067267.930794, 921412867.930794, 921672067.930794], ...
                'AbsTol', 1e-6);
            self.verifyEqual(strArcs.dBurnMagnitudes * 1000, ...
                [0.160171818048245, 0.1445880463108, 0.0221679408341286], ...
                'AbsTol', 1e-8);
            self.verifyEqual(strArcs.i32ArcSegmentTypes, ...
                9 * ones(1, 4, 'int32'));
        end
    end

    methods (Static, Access = private)
        function WriteFourArcSpk_(charSpkFile, dGaps, dBurnDeltaV, dPositionJump)
            %% SIGNATURE
            % WriteFourArcSpk_(charSpkFile, dGaps, dBurnDeltaV, dPositionJump)
            % ---------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Write four constant-velocity type-9 arcs with selected gaps and
            % burn jumps. Arc two may also carry a position discontinuity.
            % ---------------------------------------------------------------------------------------------------------
            %% INPUT
            % charSpkFile     Output SPK path.
            % dGaps           Three inter-arc gaps [s].
            % dBurnDeltaV     Three 3-component velocity jumps [km/s].
            % dPositionJump   Position offset at arc two [km].
            % ---------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % ---------------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
            % ---------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % MICE cspice_spkopn, cspice_spkw09, cspice_spkcls.
            % ---------------------------------------------------------------------------------------------------------

            arguments (Input)
                charSpkFile (1, :) char
                dGaps (1, 3) double
                dBurnDeltaV (3, 3) double
                dPositionJump (1, 1) double
            end

            i32Handle = cspice_spkopn(charSpkFile, 'segmented test', int32(0));
            dStart = 0;
            dStartPosition = [1; 0; 0];
            dVelocity = [1e-3; 0; 0];
            for ui32ArcIdx = uint32(1):uint32(4)
                if ui32ArcIdx > 1
                    dStart = dEnd + dGaps(ui32ArcIdx - 1);
                    dStartPosition = dEndPosition + dVelocity * ...
                        dGaps(ui32ArcIdx - 1);
                    if ui32ArcIdx == 2
                        dStartPosition(1) = dStartPosition(1) + dPositionJump;
                    end
                    dVelocity = dVelocity + dBurnDeltaV(:, ui32ArcIdx - 1);
                end
                dEnd = dStart + 100;
                dEndPosition = dStartPosition + dVelocity * 100;
                dStates = [dStartPosition, dEndPosition; dVelocity, dVelocity];
                cspice_spkw09(i32Handle, int32(-100001), int32(0), ...
                    'J2000', dStart, dEnd, sprintf('arc_%u', ui32ArcIdx), ...
                    int32(1), dStates, [dStart; dEnd]);
            end
            cspice_spkcls(i32Handle);
        end
    end
end
