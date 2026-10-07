function strBuild = CodegenTriangleTracing(charOutputRoot, strRayData, kwargs)
%% SIGNATURE
% strBuild = CodegenTriangleTracing(charOutputRoot, strRayData, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compile generic tracing or the complete prepared LiDAR sensor model.
% Validate geometry on the host and freeze dimensions. MEX builds use direct
% numeric arguments to remove struct-field marshalling. Generated argument
% protection still copies shared arrays; include that cost in complete-call timing.
% Generate a host facade that retains the public tracing or sensor interface.
% Native libraries retain the prepared struct interface and fixed arrays.
% Keep query values, geometry and sensor settings mutable. Reject populated destinations.
% Example: strBuild = CodegenTriangleTracing(charRoot, strRayData, bLidar=true);
% Output: A five-output LaserRangefinderPrepared_MEX and its capacity manifest.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot       Empty artifact destination.
% strRayData           Validated numeric geometry from BuildTriangleRayData.
% kwargs.bLidar        Compile the complete sensor; false compiles TraceTriangleRay.
% kwargs.charTarget    mex or lib; default mex.
% kwargs.charKernelName Optional basename; empty selects the public default.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strBuild             Build paths, fixed capacities and numeric prototype.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Generate direct-array MEX kernels and public facades.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB Coder, ValidateTriangleRayData, TraceTriangleRay, TraceTriangleRayArrays,
% LaserRangefinderModel, LaserRangefinderModelArrays, ApplyLidarMeasurementError.
% ComputeFileSha256 (MathCore) for MEX source signatures.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charOutputRoot (1, :) char
    strRayData (1, 1) struct
    kwargs.bLidar (1, 1) logical = false
    kwargs.charTarget (1, :) char {mustBeMember(kwargs.charTarget, {'mex', 'lib'})} = 'mex'
    kwargs.charKernelName (1, :) char = ''
end

arguments (Output)
    strBuild (1, 1) struct
end

% Reject missing provenance support before compiling or creating build artifacts.
if strcmp(kwargs.charTarget, 'mex')
    assert(~isempty(which('ComputeFileSha256')), ...
        'CodegenTriangleTracing:MissingDependency', ...
        'Load the MathCore ComputeFileSha256 provider before generating a MEX signature.');
end

% Validate capacities before creating or populating the output directory.
ValidateTriangleRayData(strRayData);
charKernelName = kwargs.charKernelName;
if isempty(charKernelName)
    charKernelName = 'TraceTriangleRay_MEX';
    if kwargs.bLidar
        charKernelName = 'LaserRangefinderPrepared_MEX';
    end
end
assert(isvarname(charKernelName), 'CodegenTriangleTracing:Name', ...
    'Select a valid kernel basename.');
if isfolder(charOutputRoot)
    strEntries = dir(charOutputRoot);
    assert(all(ismember({strEntries.name}, {'.', '..'})), ...
        'CodegenTriangleTracing:ExistingOutput', 'Select an empty build destination.');
end
if ~isfolder(charOutputRoot)
    mkdir(charOutputRoot);
end

% Bound dimensions and primitive types without capturing constant geometry.
objConfig = coder.config(kwargs.charTarget);
objConfig.TargetLang = 'C++';
objConfig.EnableDynamicMemoryAllocation = false;
objConfig.EnableVariableSizing = false;
objConfig.GenerateReport = false;
if strcmp(kwargs.charTarget, 'mex')
    objConfig.IntegrityChecks = false;
    objConfig.ResponsivenessChecks = false;
end
objRayType = coder.cstructname(coder.typeof(strRayData), 'STriangleRayData');

if kwargs.bLidar
    charEntryPoint = 'LaserRangefinderModel';
    objModelType = coder.typeof(struct('strRayData', strRayData));
    objModelType.Fields.strRayData = objRayType;
    cellArguments = {objModelType, [0; 0; 1], zeros(3, 1), 0, 0, zeros(3, 1), ...
                     [0; 1e5], false, false, false};
    ui8OutputCount = uint8(5);
else
    charEntryPoint = 'TraceTriangleRay';
    strQuery = struct('dDirection', [0; 0; 1], 'bAnyHit', false, 'bTwoSided', true, ...
                     'dMinDistance', 0, 'dMaxDistance', Inf, ...
                     'ui32IgnoreTriangle', uint32(0));
    cellArguments = {objRayType, zeros(3, 1), strQuery};
    ui8OutputCount = uint8(4);
end

% Direct numeric inputs remove struct-field imports while keeping geometry mutable.
charCompiledEntry = charEntryPoint;
charCompiledKernel = charKernelName;
cellGeometryFields = {'ui32TriangleCount', 'dVertex0', 'dEdge1', 'dEdge2', ...
                      'dNodeMin', 'dNodeMax', 'ui32NodeLeft', 'ui32NodeRight', ...
                      'ui32LeafStart', 'ui32LeafCount', 'ui32TriangleOrder', 'bUseBvh'};

if strcmp(kwargs.charTarget, 'mex')
    cellGeometryArgs = cell(1, numel(cellGeometryFields));
    for ui32Field = uint32(1):uint32(numel(cellGeometryFields))
        cellGeometryArgs{ui32Field} = coder.typeof(strRayData.(cellGeometryFields{ui32Field}));
    end

    charCompiledKernel = [charKernelName, '_kernel'];
    assert(isvarname(charCompiledKernel), 'CodegenTriangleTracing:Name', ...
        'Leave room for the generated _kernel suffix in the MEX basename.');

    if kwargs.bLidar
        charCompiledEntry = 'LaserRangefinderModelArrays';
        cellArguments = [cellGeometryArgs, ...
                         {[0; 0; 1], zeros(3, 1), 0, 0, [0; 1e5], false, false}];
    else
        charCompiledEntry = 'TraceTriangleRayArrays';
        cellArguments = [cellGeometryArgs, {zeros(3, 1), strQuery}];
    end
end

codegen('-config', objConfig, charCompiledEntry, '-args', cellArguments, ...
    '-nargout', num2str(ui8OutputCount), '-d', fullfile(charOutputRoot, 'Build'), ...
    '-o', fullfile(charOutputRoot, charCompiledKernel));

if strcmp(kwargs.charTarget, 'mex')
    WriteMexFacade_(charOutputRoot, charKernelName, charCompiledKernel, ...
                    cellGeometryFields, kwargs.bLidar);
end

% Record fixed capacities and the entry points used by each generated interface.
strBuild = struct('charOutputRoot', charOutputRoot, 'charKernelName', charKernelName, ...
    'charEntryPoint', charEntryPoint, 'charTarget', kwargs.charTarget, ...
    'charCompiledEntryPoint', charCompiledEntry, ...
    'charCompiledKernelName', charCompiledKernel, ...
    'bDirectGeometryInputs', strcmp(kwargs.charTarget, 'mex'), ...
    'ui32TriangleCapacity', uint32(size(strRayData.dVertex0, 2)), ...
    'ui32NodeCapacity', uint32(size(strRayData.dNodeMin, 2)), ...
    'strRayData', strRayData);
if strcmp(kwargs.charTarget, 'mex')
    % Bind consumer selection to this builder and its numerical source dependencies.
    strSignature = rmfield(strBuild, 'strRayData');
    strSignature.ui32SchemaVersion = uint32(1);
    cellSourceNames = {charEntryPoint, charCompiledEntry, 'LaserRangefinderNoiseModel', ...
                      'ApplyLidarMeasurementError', 'RayTraceTriangMesh', ...
                      'TraceTriangleRay', 'TraceTriangleRayArrays'};
    if ~kwargs.bLidar
        cellSourceNames = {'TraceTriangleRay', 'TraceTriangleRayArrays'};
    end
    cellSourceNames = [cellSourceNames, ...
        {'private/IntersectTriangleEdges.m', 'private/IntersectRayBounds.m', 'CodegenTriangleTracing'}];
    strSignature.cellSources = cellSourceNames;
    strSignature.cellSourceHashes = cell(1, numel(cellSourceNames));
    charCommonRoot = fileparts(which('TraceTriangleRay'));
    for ui32Source = uint32(1):uint32(numel(cellSourceNames))
        charSource = which(cellSourceNames{ui32Source});
        if startsWith(cellSourceNames{ui32Source}, 'private/')
            charSource = fullfile(charCommonRoot, cellSourceNames{ui32Source});
        end
        strSignature.cellSourceHashes{ui32Source} = ComputeFileSha256(charSource);
    end
    save(fullfile(charOutputRoot, [charKernelName, '_signature.mat']), 'strSignature');
end
end

function WriteMexFacade_(charOutputRoot, charPublicName, charKernelName, cellFields, bLidar)
%% SIGNATURE
% WriteMexFacade_(charOutputRoot, charPublicName, charKernelName, cellFields, bLidar)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate the host adapter from the public prepared-struct interface to the
% compiled numeric-array entry point. Keep all tracing and sensor policy in the
% compiled shared implementation; this adapter only extracts input fields.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot  Build directory containing the compiled kernel.
% charPublicName  Public facade basename.
% charKernelName  Compiled numeric kernel basename.
% cellFields      Ordered geometry field names matching the numeric entry point.
% bLidar          Generate the ten-input sensor facade; false selects generic tracing.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; write the generated MATLAB facade beside its MEX and signature.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Preserve public interfaces with direct MEX arrays.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB file I/O.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charOutputRoot  (1, :) char
    charPublicName  (1, :) char
    charKernelName  (1, :) char
    cellFields     (1, :) cell
    bLidar         (1, 1) logical
end

if bLidar
    charOutputs = 'dRange, bHit, bValid, dPoint, dError';
    % Public order is noise, legacy pruning, validity; retain it in the facade.
    cellInputLines = {sprintf('    %s(strModel, dDirection, dOrigin, dSigma, dBias, ...', ...
                             charPublicName), ...
                      '        dTargetPosition, dInterval, bNoise, bPruning, bChecks)'};
    charSetup = 'strData = strModel.strRayData;';
    charTrailingArgs = 'dDirection, dOrigin, dSigma, dBias, dInterval, bNoise, bChecks';
else
    charOutputs = 'bHit, dRange, dPoint, ui32TriangleId';
    cellInputLines = {sprintf('    %s(strData, dOrigin, strQuery)', charPublicName)};
    charSetup = '';
    charTrailingArgs = 'dOrigin, strQuery';
end

% Preserve the public argument order and keep the generated call readable.
cellLines = [{sprintf('function [%s] = ...', charOutputs)}, cellInputLines, ...
             {'% Generated by CodegenTriangleTracing; edit the owning source and rebuild.', ''}];
if ~isempty(charSetup)
    cellLines{end + 1} = charSetup;
    cellLines{end + 1} = '';
end
cellLines{end + 1} = sprintf('[%s] = %s(strData.%s, ...', ...
                           charOutputs, charKernelName, cellFields{1});

for ui32Field = uint32(2):uint32(3):uint32(numel(cellFields))
    ui32Last = min(ui32Field + 2, uint32(numel(cellFields)));
    cellArguments = strcat('strData.', cellFields(ui32Field:ui32Last));
    cellLines{end + 1} = ['    ', strjoin(cellArguments, ', '), ', ...'];
end
cellLines{end + 1} = ['    ', charTrailingArgs, ');'];
cellLines{end + 1} = '';
cellLines{end + 1} = 'end';

% Close the generated file on success and on any write failure.
charFacadePath = fullfile(charOutputRoot, [charPublicName, '.m']);
i32File = fopen(charFacadePath, 'w');
assert(i32File >= 0, 'CodegenTriangleTracing:FacadeWrite', ...
    'Unable to create the generated MEX facade: %s', charFacadePath);
objFileCleanup = onCleanup(@() fclose(i32File)); %#ok<NASGU>
fprintf(i32File, '%s\n', cellLines{:});
end
