function strVerification = testSrpLutBiasCodegen(charOutputRoot)
%% SIGNATURE
% strVerification = testSrpLutBiasCodegen(charOutputRoot)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build scalar/transverse SRP-bias MEX specializations with one, two and four
% outputs. Check source parity for signed bias, inactive/zero response, pole
% queries and both length units. Build transverse C++ force/Jacobian libraries
% and inspect their output signatures and derivative pruning. Preserve artifacts
% for review; this check does not execute a native consumer or a trajectory.
% Example: strVerification = testSrpLutBiasCodegen(tempname);
% Output: Six passing MEX builds, two native builds and saved JSON evidence.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot    New directory for generated code, binaries and test evidence.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification  Build/case counts and maximum source/MEX discrepancies.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 04-10-2026  Pietro Califano     Verify compiled model-aligned SRP bias.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB Coder, BuildSrpLutTestFixture, EvalRHS_SRPLutWithBias.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charOutputRoot (1, :) char
end

arguments (Output)
    strVerification (1, 1) struct
end

assert(~isfolder(charOutputRoot) && ~isfile(charOutputRoot), ...
    'testSrpLutBiasCodegen:ExistingOutput', 'Supply a new verification directory.');
mkdir(charOutputRoot);
charOriginalPath = path;
objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>

% Include regular illuminated queries, a dark face, poles and an inactive range.
dQueries = [13000, 15200, -13000, 0, 0, 0; ...
             3700, -5300, -3700, 0, 0, 0; ...
             2800,  1100, -2800, 1e4, -1e4, 0];
strPointing = struct('dDCM_INfromSCB', eye(3), ...
    'dJacDCMWrtPos_INfromSCB', zeros(3, 3, 3));
strSrpData = struct('dReferencePressure', 4e-6 * (1e4 / 1.495978707e11)^2, ...
    'dMass', 12, 'dBiasAcceleration', 0, 'bUseKilometersScale', false, ...
    'strPointing', strPointing);
objSrpType = coder.typeof(strSrpData);
objSrpType.Fields.strPointing = coder.cstructname(objSrpType.Fields.strPointing, 'SSrpPointing');
objSrpType = coder.cstructname(objSrpType, 'SSrpData');

ui32Cases = uint32(0);
ui32MexBuilds = uint32(0);
dMaxAccelerationError = 0;
dMaxPositionError = 0;
dMaxPositionRelativeError = 0;
dMaxBiasError = 0;

for bTransverse = [false, true]
    strLut = BuildSrpLutTestFixture(false, bTransverse);

    for ui8OutputCount = uint8([1, 2, 4])
        % Freeze the table/selection and retain only the requested output prefix.
        charKernelName = sprintf('SrpBias_%u_%u', bTransverse, ui8OutputCount);
        charBuildRoot = fullfile(charOutputRoot, charKernelName);
        mkdir(charBuildRoot);
        objConfig = coder.config('mex');
        objConfig.TargetLang = 'C++';
        objConfig.GenerateReport = false;
        objConfig.EnableDynamicMemoryAllocation = false;
        objConfig.EnableVariableSizing = false;
        objConfig.ConstantInputs = 'Remove';
        codegen('-config', objConfig, 'EvalRHS_SRPLutWithBias', ...
            '-args', {zeros(3, 1), objSrpType, coder.Constant(strLut), coder.Constant(bTransverse)}, ...
            '-nargout', num2str(ui8OutputCount), '-d', fullfile(charBuildRoot, 'Build'), ...
            '-o', fullfile(charBuildRoot, charKernelName));
        addpath(charBuildRoot, '-begin');
        fnCompiled = str2func(charKernelName);
        ui32MexBuilds = ui32MexBuilds + 1;

        for bKilometers = [false, true]
            dLengthScale = 1 + 999 * double(bKilometers);
            strCase = strSrpData;
            strCase.bUseKilometersScale = bKilometers;

            for dPressureScale = [0, 1]
                strCase.dReferencePressure = ...
                    strSrpData.dReferencePressure * dLengthScale * dPressureScale;

                for dBiasSign = [-1, 0, 1]
                    strCase.dBiasAcceleration = dBiasSign * 2e-8 / dLengthScale;

                    for ui32Query = uint32(1):uint32(size(dQueries, 2))
                        dQuery = dQueries(:, ui32Query) / dLengthScale;
                        [dExpected, dExpectedJac, dExpectedBias, bExpectedRegular] = ...
                            EvalRHS_SRPLutWithBias(dQuery, strCase, strLut, bTransverse);

                        if ui8OutputCount == 1
                            dGenerated = fnCompiled(dQuery, strCase);
                        elseif ui8OutputCount == 2
                            [dGenerated, dGeneratedJac] = fnCompiled(dQuery, strCase);
                        else
                            [dGenerated, dGeneratedJac, dGeneratedBias, bGeneratedRegular] = ...
                                fnCompiled(dQuery, strCase);
                            dMaxBiasError = max(dMaxBiasError, norm(dExpectedBias - dGeneratedBias));
                            assert(bExpectedRegular == bGeneratedRegular);
                        end

                        dMaxAccelerationError = max(dMaxAccelerationError, ...
                            norm(dExpected - dGenerated) * dLengthScale);
                        if ui8OutputCount > 1
                            dPositionError = norm(dExpectedJac - dGeneratedJac, 'fro');
                            dPositionScale = norm(dExpectedJac, 'fro');
                            dMaxPositionError = max(dMaxPositionError, dPositionError);
                            dMaxPositionRelativeError = max(dMaxPositionRelativeError, ...
                                dPositionError / max(dPositionScale, realmin));

                            % Account for large direction partials near zero panel response.
                            % Retain a strict absolute floor for zero/small derivatives.
                            assert(dPositionError < 1e-22 + 1e-12 * dPositionScale);
                        end
                        ui32Cases = ui32Cases + 1;
                    end
                end
            end
        end
    end
end

assert(dMaxAccelerationError < 1e-20 && dMaxBiasError < 1e-12);

% Inspect native force-only pruning separately from MATLAB gateway allocation.
for ui8OutputCount = uint8([1, 4])
    charBuildRoot = fullfile(charOutputRoot, sprintf('Native_%u', ui8OutputCount));
    mkdir(charBuildRoot);
    objConfig = coder.config('lib');
    objConfig.TargetLang = 'C++';
    objConfig.GenerateReport = false;
    objConfig.EnableDynamicMemoryAllocation = false;
    objConfig.EnableVariableSizing = false;
    codegen('-config', objConfig, 'EvalRHS_SRPLutWithBias', ...
        '-args', {zeros(3, 1), objSrpType, coder.Constant(strLut), coder.Constant(true)}, ...
        '-nargout', num2str(ui8OutputCount), '-d', charBuildRoot, ...
        '-o', fullfile(charBuildRoot, 'EvalRHS_SRPLutWithBias'));

    charSource = fileread(fullfile(charBuildRoot, 'EvalRHS_SRPLutWithBias.cpp'));
    charSource = regexprep(charSource, '//[^\n]*', '');
    assert(contains(charSource, 'dJacAccSRP_IN') == (ui8OutputCount > 1));
    assert(isempty(regexp(charSource, '\<(malloc|calloc|realloc)\s*\(', 'once')));
end

strVerification = struct('bPassed', true, 'ui32MexBuilds', ui32MexBuilds, ...
    'ui32NativeBuilds', uint32(2), 'ui32Cases', ui32Cases, ...
    'dMaxAccelerationError', dMaxAccelerationError, ...
    'dMaxPositionError', dMaxPositionError, 'dMaxBiasError', dMaxBiasError, ...
    'dMaxPositionRelativeError', dMaxPositionRelativeError, ...
    'bNativeConsumerExecuted', false);
i32File = fopen(fullfile(charOutputRoot, 'Bias_codegen.json'), 'w');
assert(i32File >= 0);
objFileCleanup = onCleanup(@() fclose(i32File)); %#ok<NASGU>
fprintf(i32File, '%s\n', jsonencode(strVerification, PrettyPrint=true));
disp(strVerification);
end
