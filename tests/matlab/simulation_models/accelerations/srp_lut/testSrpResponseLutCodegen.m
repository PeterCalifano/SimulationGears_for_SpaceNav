function strVerification = testSrpResponseLutCodegen(charOutputRoot, strResponseLut)
%% SIGNATURE
% strVerification = testSrpResponseLutCodegen(charOutputRoot, strResponseLut)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build runtime-table/frozen-table MEX and C++ through the public generator.
% Compare all source/MEX outputs over independent directions with transverse
% compensation disabled and enabled. Save numeric verification and generated
% source/build products; do not load kernels or run a spacecraft propagation.
% Check legacy cannonball and LUT orbital calls in one generated module.
% Example: strVerification = testSrpResponseLutCodegen(charVerificationRoot, ...
%     PackSrpResponseLut(strSaved.strLut, bIncludeTransverse=true));
% Output: Saved Codegen_verification.json and MEX/library artifacts.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot    New verification directory.
% strResponseLut    Validated payload with transverse samples for both specializations.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification  Code generation settings, tested modes and maximum parity errors.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026  Pietro Califano, Codex gpt-6  Verify bounded generated SRP evaluators.
% 01-10-2026  Pietro Califano, Codex GPT-6  Compare shared SRP diagnostics across generated orbital models.
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 01-10-2026  Pietro Califano, Codex gpt-6  Check shared type names and mixed orbital calls.
% 06-10-2026  Pietro Califano     Align runtime SRP response contracts.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CodegenSrpResponseLut, EvaluateSrpResponseLut, GenerateSrpLutDirections,
% CodegenOrbitalSrpTypesProbe; MATLAB Coder.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charOutputRoot (1, :) char
    strResponseLut (1, 1) struct
end

arguments (Output)
    strVerification (1, 1) struct
end
charOriginalPath = path;
objCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>

% Generate a separate fixed interface for each transverse inclusion mode.
charBaseRoot = charOutputRoot;
cellModeResults = cell(1, 2);
for bTransverse = [false, true]
    clear EvaluateSrpResponseLut_runtime EvaluateSrpResponseLut_frozen
    clear EvalJac_SrpResponseLut_frozen CombinedOrbitalSrpTypes
    charOutputRoot = fullfile(charBaseRoot, sprintf('Transverse_%u', bTransverse));
    strModeLut = strResponseLut;
    if ~bTransverse
        strModeLut = rmfield(strModeLut, 'dTransverseForcePerPressure');
    end
    % Build both runtime-table and frozen-table interfaces before comparing their outputs.
    strMex = CodegenSrpResponseLut(fullfile(charOutputRoot, 'Runtime_mex'), strModeLut, ...
        bIncludeTransverse=bTransverse, charKernelName='EvaluateSrpResponseLut_runtime', ...
        ui8OutputCount=uint8(3));
    assert(~strMex.bFreezeTable);
    addpath(strMex.charOutputRoot);
    strFrozenMex = CodegenSrpResponseLut(fullfile(charOutputRoot, 'Frozen_mex'), strModeLut, ...
        bIncludeTransverse=bTransverse, charKernelName='EvaluateSrpResponseLut_frozen', ...
        bFreezeTable=true, ui8OutputCount=uint8(3));
    addpath(strFrozenMex.charOutputRoot);
    dDirections = [eye(3), -eye(3), GenerateSrpLutDirections(uint32(240))];
    dMaxResponse = 0;
    dMaxCr = 0;
    dMaxTransverse = 0;
    for ui32Direction = uint32(1):uint32(size(dDirections, 2))
        dDirection = dDirections(:, ui32Direction);
        [dSource, dSourceCr, dSourceTransverse] = EvaluateSrpResponseLut(dDirection, strModeLut, bTransverse);
        [dGenerated, dGeneratedCr, dGeneratedTransverse] = EvaluateSrpResponseLut_runtime( ...
            dDirection, strModeLut);
        dMaxResponse = max(dMaxResponse, norm(dGenerated-dSource));
        dMaxCr = max(dMaxCr, abs(dGeneratedCr-dSourceCr));
        dMaxTransverse = max(dMaxTransverse, norm(dGeneratedTransverse-dSourceTransverse));

        % Verify the frozen-table signature omits the constant struct while
        % preserving all three outputs for the selected mode.
        [dGenerated, dGeneratedCr, dGeneratedTransverse] = EvaluateSrpResponseLut_frozen( ...
            dDirection);
        dMaxResponse = max(dMaxResponse, norm(dGenerated-dSource));
        dMaxCr = max(dMaxCr, abs(dGeneratedCr-dSourceCr));
        dMaxTransverse = max(dMaxTransverse, norm(dGeneratedTransverse-dSourceTransverse));
    end
    assert(dMaxResponse < 1e-12 && dMaxCr < 1e-12 && dMaxTransverse < 1e-12);

    % Change numerical samples without changing the compiled inclusion or layout.
    strChangedLut = strModeLut;
    strChangedLut.dEffectiveCr = 1.2 * strChangedLut.dEffectiveCr;
    if bTransverse
        strChangedLut.dTransverseForcePerPressure = 1.2 * strChangedLut.dTransverseForcePerPressure;
    end
    dQuery = [1;0.3;0.2];
    dChangedSource = EvaluateSrpResponseLut(dQuery, strChangedLut, bTransverse);
    dChangedGenerated = EvaluateSrpResponseLut_runtime(dQuery, strChangedLut);
    assert(norm(dChangedSource - dChangedGenerated) < 1e-12);
    assert(norm(dChangedGenerated - EvaluateSrpResponseLut_runtime(dQuery, strModeLut)) > 1e-10);

    % Compile the public C++ interface and preserve its explicit payload type.
    strLibrary = CodegenSrpResponseLut(fullfile(charOutputRoot, 'Cpp_library'), strModeLut, ...
        charTarget='lib', bIncludeTransverse=bTransverse, ...
        charKernelName='EvaluateSrpResponseLut', bFreezeTable=false);
    charTypesHeader = fileread(fullfile(strLibrary.charOutputRoot, 'Build', ...
        'EvaluateSrpResponseLut_types.h'));
    assert(contains(charTypesHeader, 'struct SSrpResponseLut {'));
    assert(contains(charTypesHeader, 'dTransverseForcePerPressure') == bTransverse);
    charLibraryHeader = fileread(fullfile(strLibrary.charOutputRoot, 'Build', 'EvaluateSrpResponseLut.h'));
    assert(~contains(charLibraryHeader, 'bIncludeTransverse'));
    if ~bTransverse
        % Inspect executable source rather than generated comments describing both modes.
        charLibrarySource = fileread(fullfile(strLibrary.charOutputRoot, 'Build', 'EvaluateSrpResponseLut.cpp'));
        charLibrarySource = regexprep(charLibrarySource, '//[^\n]*', '');
        assert(~contains(charLibrarySource, 'dInterpTransverse_SCB'));
        assert(~contains(charLibrarySource, 'dTransLowElevLowAz_SCB'));
    end

    % Compare analytical output prefixes against the source evaluator.
    strJacMex = CodegenSrpResponseLut(fullfile(charOutputRoot, 'Jacobian_mex'), strModeLut, ...
        bIncludeTransverse=bTransverse, charKernelName='EvalJac_SrpResponseLut_frozen', ...
        charEntryPoint='EvalJac_SrpResponseLut', ...
        ui8OutputCount=uint8(7));
    addpath(strJacMex.charOutputRoot);
    dMaxJacobian = 0;
    dMaxCrGradient = 0;
    dMaxTransverseJacobian = 0;
    for ui32Direction = uint32(1):uint32(size(dDirections, 2))
        dDirection = 2.7 * dDirections(:, ui32Direction);
        [dJac, dCrGradient, dJacTransverse, dForce, dCr, dTransverse, bRegular] = ...
            EvalJac_SrpResponseLut(dDirection, strModeLut, bTransverse);
        [dGeneratedJac, dGeneratedCrGradient, dGeneratedJacTransverse, dGeneratedForce, ...
            dGeneratedCr, dGeneratedTransverse, bGeneratedRegular] = ...
            EvalJac_SrpResponseLut_frozen(dDirection);
        dMaxJacobian = max(dMaxJacobian, norm(dJac - dGeneratedJac, 'fro'));
        dMaxCrGradient = max(dMaxCrGradient, norm(dCrGradient - dGeneratedCrGradient));
        dMaxTransverseJacobian = max(dMaxTransverseJacobian, norm(dJacTransverse - dGeneratedJacTransverse, 'fro'));
        assert(norm(dForce - dGeneratedForce) < 1e-12 && abs(dCr - dGeneratedCr) < 1e-12);
        assert(norm(dTransverse - dGeneratedTransverse) < 1e-12 && bRegular == bGeneratedRegular);
    end
    assert(dMaxJacobian < 1e-12 && dMaxCrGradient < 1e-12 && dMaxTransverseJacobian < 1e-12);
    strJacLibrary = CodegenSrpResponseLut(fullfile(charOutputRoot, 'Jacobian_cpp'), strModeLut, ...
        charTarget='lib', bIncludeTransverse=bTransverse, ...
        charKernelName='EvalJac_SrpResponseLut', bFreezeTable=false, ...
        charEntryPoint='EvalJac_SrpResponseLut');

    % Compile both orbital models together to catch empty/full LUT type collisions.
    dxState_IN = [1300; 370; 280; 0.01; 0.03; -0.01];
    dPosSun_IN = [1.5e11; 2e10; 3e10];
    strPointing = struct('dDCM_INfromSCB', eye(3), 'dJacDCMWrtPos_INfromSCB', zeros(3, 3, 3));
    strSrpData = struct('dReferencePressure', 4e-6, 'dMass', 12, 'dBiasAcceleration', 1e-8, ...
        'bUseKilometersScale', false, 'strPointing', strPointing);

    objSrpDataType = coder.typeof(strSrpData);
    objSrpDataType.Fields.strPointing = ...
        coder.cstructname(objSrpDataType.Fields.strPointing, 'SSrpPointing');
    objSrpDataType = coder.cstructname(objSrpDataType, 'SSrpData');
    objConfig = coder.config('mex');
    objConfig.TargetLang = 'C++';
    objConfig.GenerateReport = false;
    objConfig.EnableDynamicMemoryAllocation = false;
    objConfig.EnableVariableSizing = false;
    objConfig.ConstantInputs = 'Remove';
    charOrbitalRoot = fullfile(charOutputRoot, 'Combined_orbital_types');
    mkdir(charOrbitalRoot);
    codegen('-config', objConfig, 'CodegenOrbitalSrpTypesProbe', ...
        '-args', {dxState_IN, dPosSun_IN, false, objSrpDataType, ...
                  coder.Constant(strModeLut), coder.Constant(bTransverse)}, ...
        '-d', fullfile(charOrbitalRoot, 'Build'), '-o', fullfile(charOrbitalRoot, 'CombinedOrbitalSrpTypes'));
    addpath(charOrbitalRoot);

    % Compare both models with runtime eclipse in the selected transverse specialization.
    dMaxOrbitalDifference = 0;
    for bIsInEclipse = [false, true]
        [dExpectedCannonball, dExpectedLut, strExpectedCannonball, strExpectedLut] = ...
            CodegenOrbitalSrpTypesProbe(dxState_IN, dPosSun_IN, bIsInEclipse, ...
                                       strSrpData, strModeLut, bTransverse);
        [dGeneratedCannonball, dGeneratedLut, strGeneratedCannonball, strGeneratedLut] = ...
            CombinedOrbitalSrpTypes(dxState_IN, dPosSun_IN, bIsInEclipse, strSrpData);
        dMaxOrbitalDifference = max([dMaxOrbitalDifference, ...
            norm(dExpectedCannonball - dGeneratedCannonball), norm(dExpectedLut - dGeneratedLut)]);
        assert(isequal(fieldnames(strGeneratedCannonball), fieldnames(strGeneratedLut)));
        assert(norm(strExpectedCannonball.dAccSRP - strGeneratedCannonball.dAccSRP) < 1e-18);
        assert(norm(strExpectedLut.dAccSRP - strGeneratedLut.dAccSRP) < 1e-18);
    end
    assert(dMaxOrbitalDifference < 1e-18);

    % Save the generated interfaces and measured source/MEX discrepancies.
    strVerification = struct('bPassed', true, 'strMex', strMex, ...
        'strFrozenMex', strFrozenMex, 'strLibrary', strLibrary, ...
        'strJacMex', strJacMex, 'strJacLibrary', strJacLibrary, ...
        'charOrbitalTypesRoot', charOrbitalRoot, 'dMaxOrbitalDifference', dMaxOrbitalDifference, ...
        'dMaxJacobianDifference', dMaxJacobian, 'dMaxCrGradientDifference', dMaxCrGradient, ...
        'dMaxTransverseJacobianDifference', dMaxTransverseJacobian, ...
        'ui32DirectionCount', uint32(size(dDirections, 2)), 'bScalarAndTransversePassed', true, ...
        'dMaxResponseDifference_m2', dMaxResponse, 'dMaxCrDifference', dMaxCr, ...
        'dMaxTransverseDifference_m2', dMaxTransverse);

    i32File = fopen(fullfile(charOutputRoot, 'Codegen_verification.json'), 'w');
    assert(i32File >= 0);
    objFileCleanup = onCleanup(@() fclose(i32File)); %#ok<NASGU>
    fprintf(i32File, '%s\n', jsonencode(strVerification, PrettyPrint=true));
    fprintf('SRP lookup codegen passed: transverse %u, 246 directions, MEX/C++ parity.\n', ...
        bTransverse);
    cellModeResults{1 + double(bTransverse)} = strVerification;
end
strVerification = struct('bPassed', true, 'cellModeResults', {cellModeResults});

end
