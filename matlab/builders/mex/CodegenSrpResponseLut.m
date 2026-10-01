function strCodegen = CodegenSrpResponseLut(charOutputRoot, strResponseLut, kwargs)
%% SIGNATURE
% strCodegen = CodegenSrpResponseLut(charOutputRoot, strResponseLut, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate and build the fixed-schema SRP evaluator as MEX or a C++ library.
% Disable dynamic memory allocation and variable sizing. The default generated
% interface embeds an immutable table to remove repeated MATLAB transport;
% opt into a runtime const-pointer table for C++ or runtime-table validation.
% Freeze transverse inclusion independently of numerical table embedding.
% Remove constant inputs from the generated signatures and omit transverse
% storage and calculations in scalar builds.
% Specialize the generated signature to the requested output count. Default MEX
% to one output and C++ libraries to the complete entry-point signature.
% Example: strCodegen = CodegenSrpResponseLut(charOutputRoot, strResponseLut, ...
%     charTarget='lib', charKernelName='EvaluateSrpResponseLut');
% Output: A compiled library plus headers/code, or a MEX plus code, under charOutputRoot.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot          New output directory for generated source/build artifacts.
% strResponseLut          Validated fixed numeric payload from PackSrpResponseLut.
% kwargs.charTarget       'mex' or 'lib'; default 'mex'.
% kwargs.charKernelName   Output basename; empty derives the selected entry/target name.
% kwargs.bFreezeTable     Embed an immutable table; default true. Set false for a C++ const-pointer API.
% kwargs.charEntryPoint   Force evaluator or analytical Jacobian evaluator; default force.
% kwargs.bIncludeTransverse Compile transverse support; default false.
% kwargs.ui8OutputCount   Number of leading outputs to generate; zero selects the target default.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strCodegen              Target, paths, capacity and fixed-allocation settings.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 28-09-2026  Pietro Califano, Codex gpt-6  Add reproducible bounded lookup code generation.
% 28-09-2026  Pietro Califano, Codex gpt-6  Derive the fixed type from the validated input.
% 29-09-2026  Pietro Califano, Codex gpt-6  Embed immutable tables by default and generate derivatives.
% 29-09-2026  Pietro Califano, Codex gpt-6  Compile only the requested output prefix.
% 01-10-2026  Pietro Califano, Codex gpt-6  Document the shared generated types and build steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB Coder, EvaluateSrpResponseLut, EvalJac_SrpResponseLut, ValidateSrpResponseLut.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charOutputRoot (1, :) char
    strResponseLut (1, 1) struct
    kwargs.charTarget (1, :) char {mustBeMember(kwargs.charTarget, {'mex', 'lib'})} = 'mex'
    kwargs.charKernelName (1, :) char = ''
    kwargs.bIncludeTransverse (1, 1) logical = false
    kwargs.bFreezeTable (1, 1) logical = true
    kwargs.charEntryPoint (1, :) char {mustBeMember(kwargs.charEntryPoint, ...
        {'EvaluateSrpResponseLut', 'EvalJac_SrpResponseLut'})} = 'EvaluateSrpResponseLut'
    kwargs.ui8OutputCount (1, 1) uint8 = uint8(0)
end

arguments (Output)
    strCodegen (1, 1) struct
end

% Validate the payload and Coder availability before creating artifacts.
ValidateSrpResponseLut(strResponseLut);
if kwargs.bIncludeTransverse
    assert(isfield(strResponseLut, 'dTransverseForcePerPressure'), ...
        'CodegenSrpResponseLut:MissingTransverse', 'Supply transverse samples for this specialization.');
elseif isfield(strResponseLut, 'dTransverseForcePerPressure')
    % Exclude vector storage from scalar runtime-table interfaces as well as embedded builds.
    strResponseLut = rmfield(strResponseLut, 'dTransverseForcePerPressure');
end
assert(exist('codegen', 'file') ~= 0, 'CodegenSrpResponseLut:MissingCoder', ...
    'Install MATLAB Coder before generating the model.');

% Derive one valid artifact basename from the selected entry point and target.
charKernelName = kwargs.charKernelName;
if isempty(charKernelName)
    charKernelName = kwargs.charEntryPoint;
    if strcmp(kwargs.charTarget, 'mex')
        charKernelName = [charKernelName, '_mex'];
    end
end
assert(isvarname(charKernelName), 'CodegenSrpResponseLut:InvalidName', 'Supply a valid kernel basename.');

% Resolve the output prefix before creating any generated artifacts.
ui8AvailableOutputs = uint8(nargout(kwargs.charEntryPoint));
ui8OutputCount = kwargs.ui8OutputCount;
if ui8OutputCount == 0
    ui8OutputCount = ui8AvailableOutputs;
    if strcmp(kwargs.charTarget, 'mex')
        ui8OutputCount = uint8(1);
    end
end
assert(ui8OutputCount <= ui8AvailableOutputs, 'CodegenSrpResponseLut:InvalidOutputs', ...
    'Request at most %u outputs for %s.', ui8AvailableOutputs, kwargs.charEntryPoint);

% Preserve existing artifacts by requiring an absent or empty output directory.
if isfolder(charOutputRoot)
    strEntries = dir(charOutputRoot);
    assert(all(ismember({strEntries.name}, {'.', '..'})), ...
        'CodegenSrpResponseLut:ExistingOutput', 'Select an empty code-generation directory.');
end
mkdir(charOutputRoot);

% Embed the table by default; opt into runtime const-pointer storage explicitly.
objConfig = coder.config(kwargs.charTarget);
objConfig.TargetLang = 'C++';
objConfig.GenerateReport = false;
objConfig.EnableDynamicMemoryAllocation = false;
objConfig.EnableVariableSizing = false;
objTableType = coder.cstructname(coder.typeof(strResponseLut), 'SSrpResponseLut');
cellArguments = {[1;0;0], objTableType, coder.Constant(kwargs.bIncludeTransverse)};
if strcmp(kwargs.charTarget, 'mex')
    objConfig.ConstantInputs = 'Remove';
end
if kwargs.bFreezeTable
    cellArguments{2} = coder.Constant(strResponseLut);
end

codegen('-config', objConfig, kwargs.charEntryPoint, '-args', cellArguments, ...
    '-nargout', num2str(ui8OutputCount), ...
    '-d', fullfile(charOutputRoot, 'Build'), '-o', fullfile(charOutputRoot, charKernelName));

% Report the generated interface and fixed capacities for consumer checks.
strCodegen = struct('charTarget', kwargs.charTarget, 'charOutputRoot', charOutputRoot, ...
    'charKernelName', charKernelName, 'bFreezeTable', kwargs.bFreezeTable, ...
    'bIncludeTransverse', kwargs.bIncludeTransverse, ...
    'charEntryPoint', kwargs.charEntryPoint, 'ui8OutputCount', ui8OutputCount, ...
    'bDynamicMemoryAllocation', false, 'bVariableSizing', false, ...
    'ui32MaxAzimuthCount', uint32(size(strResponseLut.dAzimuth, 2)), ...
    'ui32MaxElevationCount', uint32(size(strResponseLut.dElevation, 2)));
end
