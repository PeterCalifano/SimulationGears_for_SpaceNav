function strBuild = CodegenPanelSrpResponse(charOutputRoot, strPanel, kwargs)
%% SIGNATURE
% strBuild = CodegenPanelSrpResponse(charOutputRoot, strPanel, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compile the complete shadow-aware panel response with fixed-size runtime data.
% Select a fixed batch capacity and leading output count. Keep host metadata out
% of generated arguments; do not require constant mesh capture. Preserve previous
% builds by rejecting populated destinations.
% Example: strBuild = CodegenPanelSrpResponse(charOutputRoot,strPanel,ui8OutputCount=uint8(2));
% Output: A MEX returning force/pressure and frozen-visibility direction partials.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot           New artifact directory.
% strPanel                 Prepared SI optical/geometry data with strShadowData.
% kwargs.charTarget        'mex' or 'lib'; default mex.
% kwargs.ui32DirectionCount Fixed batch capacity; default one.
% kwargs.ui8OutputCount    Leading outputs; zero selects one for MEX, three for lib.
% kwargs.charKernelName    Artifact basename; default ComputePanelSrpResponse_mex.
% kwargs.bRuntimeChecks   Include generated integrity/responsiveness checks;
%                         default false for validated prepared geometry.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strBuild                 Paths, selected signature and numeric prototype.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Compile selected panel outputs with fixed runtime geometry.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB Coder, ValidateTriangleRayData, ComputePanelSrpResponse.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charOutputRoot (1, :) char
    strPanel (1, 1) struct
    kwargs.charTarget (1, :) char {mustBeMember(kwargs.charTarget, {'mex', 'lib'})} = 'mex'
    kwargs.ui32DirectionCount (1, 1) uint32 {mustBePositive} = uint32(1)
    kwargs.ui8OutputCount (1, 1) uint8 = uint8(0)
    kwargs.charKernelName (1, :) char = 'ComputePanelSrpResponse_mex'
    kwargs.bRuntimeChecks (1, 1) logical = false
end

arguments (Output)
    strBuild (1, 1) struct
end

% Validate optics and prepared geometry before allocating any build directory.
dAreas = strPanel.dSCquadsArea(:);
dNormals = strPanel.dQuadsNormals_SCB;
dCoefficients = strPanel.dDiffSpecQuadsCoeffs;
dCentres = strPanel.dQuadsPressCentre_SCB;
dFaceCount = numel(dAreas);
assert(all(isfinite(dAreas)) && all(dAreas >= 0) && ...
    isequal(size(dNormals), [3, dFaceCount]) && all(isfinite(dNormals), 'all') && ...
    all(vecnorm(dNormals)>0) && isequal(size(dCoefficients), [dFaceCount, 2]) && ...
    all(isfinite(dCoefficients), 'all') && all(dCoefficients >= 0, 'all') && ...
    all(sum(dCoefficients, 2) <= 1) && isequal(size(dCentres), [3, dFaceCount]) && ...
    all(isfinite(dCentres), 'all'), ...
    'CodegenPanelSrpResponse:PanelData', 'Supply complete finite SI optics and geometry.');
strShadow = strPanel.strShadowData;
ValidateTriangleRayData(strShadow.strRayData);
assert(strShadow.strRayData.ui32TriangleCount == dFaceCount && ...
    size(strShadow.dSamplePoints_SCB, 1) == 3 && size(strShadow.dSamplePoints_SCB, 3) == dFaceCount && ...
    size(strShadow.dSamplePoints_SCB, 2)>0 && all(isfinite(strShadow.dSamplePoints_SCB), 'all') && ...
    isscalar(strShadow.dRayOffset) && isfinite(strShadow.dRayOffset) && strShadow.dRayOffset>0, ...
    'CodegenPanelSrpResponse:ShadowData', 'Supply matching finite samples and a positive ray offset.');
strNumericPanel = struct('dSCquadsArea', dAreas, 'dDiffSpecQuadsCoeffs', dCoefficients, ...
    'dQuadsNormals_SCB', dNormals, 'dQuadsPressCentre_SCB', dCentres, ...
    'strShadowData', struct('dSamplePoints_SCB', strShadow.dSamplePoints_SCB, ...
        'dRayOffset', strShadow.dRayOffset, 'strRayData', strShadow.strRayData));
ui8Outputs = kwargs.ui8OutputCount;
if ui8Outputs == 0
    ui8Outputs = uint8(1);
    if strcmp(kwargs.charTarget, 'lib')
        ui8Outputs = uint8(3);
    end
end
assert(ui8Outputs <= 3 && isvarname(kwargs.charKernelName), ...
    'CodegenPanelSrpResponse:Signature', 'Select one through three outputs and a valid basename.');
if isfolder(charOutputRoot)
    strEntries = dir(charOutputRoot);
    assert(all(ismember({strEntries.name}, {'.', '..'})), ...
        'CodegenPanelSrpResponse:ExistingOutput', 'Select an empty build destination.');
end
if ~isfolder(charOutputRoot)
    mkdir(charOutputRoot);
end

% Freeze capacities while retaining runtime geometry and shadow selection.
objConfig = coder.config(kwargs.charTarget);
objConfig.TargetLang = 'C++';
objConfig.EnableDynamicMemoryAllocation = false;
objConfig.EnableVariableSizing = false;
objConfig.GenerateReport = false;
if strcmp(kwargs.charTarget, 'mex')
    objConfig.IntegrityChecks = kwargs.bRuntimeChecks;
    objConfig.ResponsivenessChecks = kwargs.bRuntimeChecks;
end
objPanelType = coder.cstructname(coder.typeof(strNumericPanel), 'SPanelSrpModel');
codegen('-config', objConfig, 'ComputePanelSrpResponse', '-args', ...
    {repmat([1;0;0], 1, kwargs.ui32DirectionCount), objPanelType, true}, ...
    '-nargout', num2str(ui8Outputs), '-d', fullfile(charOutputRoot, 'Build'), ...
    '-o', fullfile(charOutputRoot, kwargs.charKernelName));
strBuild = struct('charOutputRoot', charOutputRoot, 'charKernelName', kwargs.charKernelName, ...
    'charTarget', kwargs.charTarget, 'ui8OutputCount', ui8Outputs, ...
    'ui32DirectionCount', kwargs.ui32DirectionCount, 'strNumericPanel', strNumericPanel);
end
