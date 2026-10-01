function strVerification = testSrpLutOutputCodegen(charOutputRoot, strResponseLut)
%% SIGNATURE
% strVerification = testSrpLutOutputCodegen(charOutputRoot, strResponseLut)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify generated output prefixes, MEX buffer counts and physical-partial pruning.
% Compare reduced and diagnostic Jacobian calls on the same directions and modes.
% Benchmark identical one-output call patterns after warming each MEX. Preserve
% generated code and metrics outside source trees; run no trajectory simulation.
% Example: strVerification = testSrpLutOutputCodegen(charNewRoot, strPayload);
% Output: Passing parity/signature checks, generated artifacts and host timing.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot   New verification directory outside source trees.
% strResponseLut   Prepared transverse numeric LUT with fixed capacity.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification  Output counts, buffer counts, parity errors and timing [s/call].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Verify output-specialized SRP code generation.
% 01-10-2026  Pietro Califano, Codex gpt-6  Update acceleration names and remove eclipse inputs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CodegenSrpResponseLut, EvalJac_SrpResponseLut, ComputeSrpLutAcceleration; MATLAB Coder.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charOutputRoot (1, :) char
    strResponseLut (1, 1) struct
end

arguments (Output)
    strVerification (1, 1) struct
end

% Restore the caller's path after loading generated MEX artifacts.
charOriginalPath = path;
objCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>

% Generate only the requested leading outputs; retain a complete diagnostic control.
% Generate a separate fixed interface for each transverse inclusion mode.
charBaseRoot = charOutputRoot;
cellModeResults = cell(1, 2);
for bTransverse = [false, true]
    clear EvalSrpJac_outputs1 EvalSrpJac_outputs3 EvalSrpJac_outputs7 EvalSrpForce_outputs1
    charOutputRoot = fullfile(charBaseRoot, sprintf('Transverse_%u', bTransverse));
    strModeLut = strResponseLut;
    if ~bTransverse
        strModeLut = rmfield(strModeLut, 'dTransverseForcePerPressure');
    end
    ui8Counts = uint8([1, 3, 7]);
    cellBuilds = cell(1, 3);
    dBufferCounts = zeros(1, 3);
    for ui32Build = uint32(1):uint32(3)
        ui8Requested = ui8Counts(ui32Build);
        charKernel = sprintf('EvalSrpJac_outputs%u', ui8Requested);
        ui8Option = ui8Requested;
        if ui32Build == 1
            ui8Option = uint8(0);  % Exercise the default Jacobian-only MEX contract.
        end
        cellBuilds{ui32Build} = CodegenSrpResponseLut(fullfile(charOutputRoot, charKernel), ...
            strModeLut, bIncludeTransverse=bTransverse, charEntryPoint='EvalJac_SrpResponseLut', ...
            charKernelName=charKernel, ui8OutputCount=ui8Option);
        assert(cellBuilds{ui32Build}.ui8OutputCount == ui8Requested);
        addpath(cellBuilds{ui32Build}.charOutputRoot);
        charApi = fileread(fullfile(cellBuilds{ui32Build}.charOutputRoot, 'Build', ...
            'interface', '_coder_EvalJac_SrpResponseLut_api.cpp'));
        charGateway = fileread(fullfile(cellBuilds{ui32Build}.charOutputRoot, 'Build', ...
            'interface', '_coder_EvalJac_SrpResponseLut_mex.cpp'));
        assert(contains(charGateway, sprintf('if (nlhs > %u)', ui8Requested)));
        dBufferCounts(ui32Build) = numel(regexp(charApi, 'mxMalloc\(', 'match'));
    end
    assert(dBufferCounts(1) == 1 && dBufferCounts(1) < dBufferCounts(3));

    % Check both modes, pole adjustments and all generated output prefixes.
    dDirections = [eye(3), -eye(3), [1e-10;0;1], [0;1e-10;-1], ...
        GenerateSrpLutDirections(uint32(256))];
    dMaxDifference = 0;
    for ui32Query = uint32(1):uint32(size(dDirections, 2))
        dQuery = 2.7*dDirections(:, ui32Query);
        cellExpected = cell(1, 7);
        [cellExpected{:}] = EvalJac_SrpResponseLut(dQuery, strModeLut, bTransverse);
        for ui32Build = uint32(1):uint32(3)
            cellActual = cell(1, ui8Counts(ui32Build));
            [cellActual{:}] = feval(cellBuilds{ui32Build}.charKernelName, dQuery);
            for ui32Output = uint32(1):uint32(numel(cellActual))
                if islogical(cellExpected{ui32Output})
                    assert(isequal(cellActual{ui32Output}, cellExpected{ui32Output}));
                else
                    dDifference = max(abs(cellActual{ui32Output}-cellExpected{ui32Output}), [], 'all');
                    dMaxDifference = max(dMaxDifference, dDifference);
                end
            end
        end
    end
    assert(dMaxDifference < 1e-12);

    % Preserve force-only generation and reject unsupported counts before building.
    strForce = CodegenSrpResponseLut(fullfile(charOutputRoot, 'Force_only'), strModeLut, ...
        bIncludeTransverse=bTransverse, charKernelName='EvalSrpForce_outputs1');
    assert(strForce.ui8OutputCount == 1);
    addpath(strForce.charOutputRoot);
    dExpected = EvaluateSrpResponseLut(dDirections(:, 9), strModeLut, bTransverse);
    assert(norm(EvalSrpForce_outputs1(dDirections(:, 9))-dExpected) < 1e-12);

    bRejected = false;
    try
        CodegenSrpResponseLut(fullfile(charOutputRoot, 'Invalid_outputs'), strModeLut, ...
            bIncludeTransverse=bTransverse, charEntryPoint='EvalJac_SrpResponseLut', ui8OutputCount=uint8(8));
    catch objError
        bRejected = strcmp(objError.identifier, 'CodegenSrpResponseLut:InvalidOutputs');
    end
    assert(bRejected && ~isfolder(fullfile(charOutputRoot, 'Invalid_outputs')));

    % Inspect C++ interfaces across force-only, position-only and complete physical outputs.
    objConfig = coder.config('lib');
    objConfig.TargetLang = 'C++';
    objConfig.GenerateReport = false;
    objConfig.EnableDynamicMemoryAllocation = false;
    objConfig.EnableVariableSizing = false;
    cellArguments = {[1300;370;280], eye(3), 12, 4e-6, coder.Constant(strModeLut), ...
        coder.Constant(bTransverse), true, zeros(3, 3, 3)};
    ui8PhysicalCounts = uint8([1, 2, 5]);
    for ui8Count = ui8PhysicalCounts
        charPhysicalRoot = fullfile(charOutputRoot, sprintf('Physical_outputs%u', ui8Count));
        mkdir(charPhysicalRoot);
        codegen('-config', objConfig, 'ComputeSrpLutAcceleration', '-args', cellArguments, ...
            '-nargout', num2str(ui8Count), '-d', fullfile(charPhysicalRoot, 'Build'), ...
            '-o', fullfile(charPhysicalRoot, 'ComputeSrpLutAcceleration'));
        charHeader = fileread(fullfile(charPhysicalRoot, 'Build', 'ComputeSrpLutAcceleration.h'));
        assert(contains(charHeader, 'dJacAccSRP_IN') == (ui8Count >= 2));
        assert(contains(charHeader, 'dJacAccSRPatt_IN') == (ui8Count >= 3));
        assert(contains(charHeader, 'dJacAccSRPmass_IN') == (ui8Count >= 4));
        if ui8Count == 1
            assert(~isfile(fullfile(charPhysicalRoot, 'Build', 'EvalJac_SrpResponseLut.cpp')));
        end
    end

    % Measure each gateway with one returned matrix and the same MATLAB-loop overhead.
    dBatchSeconds = zeros(15, 3);
    dChecksum = zeros(1, 3);
    for ui32Build = uint32(1):uint32(3)
        fcnKernel = str2func(cellBuilds{ui32Build}.charKernelName);
        for ui32Warm = uint32(1):uint32(64)
            fcnKernel(dDirections(:, ui32Warm));
        end
        for ui32Repeat = uint32(1):uint32(15)
            objTimer = tic;
            for ui32Query = uint32(1):uint32(size(dDirections, 2))
                dJacobian = fcnKernel(dDirections(:, ui32Query));
                dChecksum(ui32Build) = dChecksum(ui32Build)+sum(dJacobian(:));
            end
            dBatchSeconds(ui32Repeat, ui32Build) = toc(objTimer)/size(dDirections, 2);
        end
    end
    strVerification = struct('bPassed', true, 'cellBuilds', {cellBuilds}, 'strForce', strForce, ...
        'ui8OutputCounts', ui8Counts, 'dGatewayBufferCounts', dBufferCounts, ...
        'dMaxOutputDifference', dMaxDifference, 'ui32QueryCount', uint32(size(dDirections, 2)), ...
        'ui8PhysicalCounts', ui8PhysicalCounts, 'dMedianSeconds', median(dBatchSeconds, 1), ...
        'dBatchSeconds', dBatchSeconds, 'dChecksum', dChecksum);
    save(fullfile(charOutputRoot, 'Output_codegen_verification.mat'), 'strVerification');
    writetable(table(double(ui8Counts.'), dBufferCounts.', 1e6*median(dBatchSeconds, 1).', ...
        'VariableNames', {'GeneratedOutputs', 'GatewayBuffers', 'MedianTime_us'}), ...
        fullfile(charOutputRoot, 'Output_timing.csv'));
    fprintf('SRP output specialization passed: buffers %s, median microseconds %s.\n', ...
        mat2str(dBufferCounts), mat2str(1e6*median(dBatchSeconds, 1), 5));
    cellModeResults{1 + double(bTransverse)} = strVerification;
end
strVerification = struct('bPassed', true, 'cellModeResults', {cellModeResults});

end
