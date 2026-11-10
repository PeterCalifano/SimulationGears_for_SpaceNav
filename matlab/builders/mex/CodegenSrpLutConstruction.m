function strBuild = CodegenSrpLutConstruction(charOutputRoot, strPanel, kwargs)
%% SIGNATURE
% strBuild = CodegenSrpLutConstruction(charOutputRoot, strPanel, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compile the complete numeric LUT batch, including visibility and force, with
% fixed-size runtime geometry and optics. Keep changing sample coefficients out
% of specialization. Reject populated destinations and report compilation errors.
% Example: strBuild = CodegenSrpLutConstruction(tempname, strNumericPanel);
% Output: A fixed-size MEX provider and its build metadata.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot    Empty or new code-generation directory.
% strPanel         Validated numeric SI panel and shadow geometry.
% ui32BatchCount   Fixed direction capacity; default 64.
% charKernelName   Generated function basename.
% bIncludeTorque  Include torque/pressure in addition to force and visibility.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strBuild         Provider path, signature and fixed-allocation policy.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Add source-shared complete LUT construction codegen.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB Coder, ComputeSrpLutGrid.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charOutputRoot (1,:) char
    strPanel (1,1) struct
    kwargs.ui32BatchCount (1,1) uint32 {mustBePositive} = uint32(64)
    kwargs.charKernelName (1,:) char = 'ComputeSrpLutGrid_mex'
    kwargs.bIncludeTorque (1,1) logical = false
end
arguments (Output)
    strBuild (1,1) struct
end

% Validate compilation availability before allocating any artifacts.
assert(exist('codegen','file') ~= 0 && license('test','MATLAB_Coder'), ...
    'CodegenSrpLutConstruction:MissingCoder','MATLAB Coder is required for LUT MEX construction.');
assert(isvarname(kwargs.charKernelName),'CodegenSrpLutConstruction:InvalidName', ...
    'Supply a valid generated function basename.');
% Reject malformed external prototypes before entering unchecked fixed-size kernels.
ui32Faces = uint32(numel(strPanel.dSCquadsArea));
assert(isequal(size(strPanel.dQuadsNormals_SCB),[3,double(ui32Faces)]) && ...
    all(isfinite(strPanel.dQuadsNormals_SCB),'all') && all(vecnorm(strPanel.dQuadsNormals_SCB)>0) && ...
    all(isfinite(strPanel.dSCquadsArea)) && all(strPanel.dSCquadsArea>=0) && ...
    isequal(size(strPanel.dDiffSpecQuadsCoeffs),[double(ui32Faces),2]) && ...
    all(isfinite(strPanel.dDiffSpecQuadsCoeffs),'all') && ...
    all(strPanel.dDiffSpecQuadsCoeffs>=0,'all') && ...
    all(sum(strPanel.dDiffSpecQuadsCoeffs,2)<=1+8*eps), ...
    'CodegenSrpLutConstruction:PanelData','Supply finite physical SI panel inputs.');
strShadow = strPanel.strShadowData;
assert(size(strShadow.dSamplePoints_SCB,1)==3 && ...
    size(strShadow.dSamplePoints_SCB,3)==ui32Faces && ...
    size(strShadow.dSamplePoints_SCB,2)>0 && ...
    all(isfinite(strShadow.dSamplePoints_SCB),'all') && ...
    isfinite(strShadow.dRayOffset) && strShadow.dRayOffset>0, ...
    'CodegenSrpLutConstruction:ShadowData','Supply finite matching quadrature and a positive ray offset.');
if isfolder(charOutputRoot)
    strEntries = dir(charOutputRoot);
    assert(all(ismember({strEntries.name},{'.','..'})), ...
        'CodegenSrpLutConstruction:ExistingOutput','Select an empty output directory.');
end
mkdir(charOutputRoot);

% Compile both cold geometry preparation and warm optical weighting in one provider.
objConfig = coder.config('mex');
objConfig.TargetLang = 'C++';
objConfig.GenerateReport = false;
objConfig.EnableDynamicMemoryAllocation = false;
objConfig.EnableVariableSizing = false;
objConfig.IntegrityChecks = false;
objConfig.ResponsivenessChecks = false;
ui8OutputCount = uint8(2 + kwargs.bIncludeTorque);
cellArguments = {repmat([1;0;0],1,kwargs.ui32BatchCount),coder.typeof(strPanel), ...
    true,false,ones(numel(strPanel.dSCquadsArea),kwargs.ui32BatchCount)};
codegen('-config',objConfig,'ComputeSrpLutGrid','-args',cellArguments, ...
    '-nargout',num2str(ui8OutputCount),'-d',fullfile(charOutputRoot,'Build'), ...
    '-o',fullfile(charOutputRoot,kwargs.charKernelName));
strBuild = struct('charKernelName',kwargs.charKernelName,'charOutputRoot',charOutputRoot, ...
    'ui32BatchCount',kwargs.ui32BatchCount,'bIncludeTorque',kwargs.bIncludeTorque, ...
    'bDynamicMemoryAllocation',false,'bVariableSizing',false);
end
