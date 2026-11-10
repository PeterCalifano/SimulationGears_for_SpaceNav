function strChecks = testSrpLutConstruction(bCompile)
%% SIGNATURE
% strChecks = testSrpLutConstruction(bCompile)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify complete construction, independent occlusion/force/torque oracles,
% geometry versus optics reuse, disk-cache transitions, and 0.5-degree capacity.
% Use synthetic triangles and disposable artifacts; optionally compile the real grid kernel.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% bCompile  Run complete construction MEX parity; default false.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strChecks Independent oracle, reuse and optional compiled parity checks.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Cover optimized construction and fine truth payloads.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpResponseLut, PackSrpResponseLut, EvaluateSrpResponseLut.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    bCompile (1,1) logical = false
end
arguments (Output)
    strChecks (1,1) struct
end
charTemporary = tempname;
mkdir(charTemporary);
objCleanup = onCleanup(@() rmdir(charTemporary,'s')); %#ok<NASGU>
strPanel = struct('dVerticesPos',[0,0,0;1,0,0;0,1,0;0,0,1;1,0,1;0,1,1], ...
    'ui32FaceVertexIds',uint32([1,2,3;4,5,6]),'dSCquadsArea',[0.5;0.5], ...
    'dQuadsNormals_SCB',[0,0;0,0;1,1],'dQuadsPressCentre_SCB',[1/3,1/3;1/3,1/3;0,1], ...
    'dDiffSpecQuadsCoeffs',[0,0.2;0,0.2],'charSourceObjFilePath','');

% The upper plate alone is illuminated; its force and torque are analytic.
[strSource,strContext] = BuildSrpResponseLut(strPanel,0.5,90,bUseCodegen=false, ...
    bIncludeTransverse=true,bIncludeTorque=true,ui32ShadowLevel=uint32(0));
[dForce,~,~,dTorque] = EvaluateSrpResponseLut([0;0;1],strSource.strResponseLut,true);
assert(norm(dForce-[0;0;-0.6]) < 1e-8 && norm(dTorque-[-0.2;0.2;0]) < 1e-8);
[~,strContext] = BuildSrpResponseLut(strPanel,0.5,90,bUseCodegen=false, ...
    bIncludeTransverse=true,bIncludeTorque=true,ui32ShadowLevel=uint32(0),strPreparation=strContext);
assert(strContext.ui32LutBuildCount==1 && strContext.ui32VisibilityBuildCount==1);

% Optics and reference area change responses while preserving geometric visibility.
strChanged = strPanel;
strChanged.dDiffSpecQuadsCoeffs(:,2) = 0.1;
[strChangedLut,strContext] = BuildSrpResponseLut(strChanged,0.7,90,bUseCodegen=false, ...
    bIncludeTransverse=true,bIncludeTorque=true,ui32ShadowLevel=uint32(0),strPreparation=strContext);
assert(strContext.ui32LutBuildCount==2 && strContext.ui32VisibilityBuildCount==1);
assert(norm(EvaluateSrpResponseLut([0;0;1],strChangedLut.strResponseLut,true)) > 0);

% Torque depends on pressure centres even when visibility and force are identical.
strMoved = strChanged;
strMoved.dQuadsPressCentre_SCB(1,:) = strMoved.dQuadsPressCentre_SCB(1,:) + 1;
[strMovedLut,strContext] = BuildSrpResponseLut(strMoved,0.7,90,bUseCodegen=false, ...
    bIncludeTransverse=true,bIncludeTorque=true,ui32ShadowLevel=uint32(0),strPreparation=strContext);
[dMovedForce,~,~,dMovedTorque] = EvaluateSrpResponseLut([0;0;1],strMovedLut.strResponseLut,true);
[dUnmovedForce,~,~,dUnmovedTorque] = EvaluateSrpResponseLut([0;0;1],strChangedLut.strResponseLut,true);
assert(norm(dMovedForce-dUnmovedForce)<1e-12 && norm(dMovedTorque-dUnmovedTorque)>0.1);
assert(strContext.ui32VisibilityBuildCount==1);

% A table disk hit must not mark uncomputed visibility as ready for changed optics.
BuildSrpResponseLut(strPanel,0.5,90,bUseCodegen=false,bIncludeTransverse=true, ...
    ui32ShadowLevel=uint32(0),bUseDiskCache=true,charCacheDirectory=charTemporary);
[~,strDiskContext] = BuildSrpResponseLut(strPanel,0.5,90,bUseCodegen=false,bIncludeTransverse=true, ...
    ui32ShadowLevel=uint32(0),bUseDiskCache=true,charCacheDirectory=charTemporary);
[strAfterDisk,strDiskContext] = BuildSrpResponseLut(strChanged,0.5,90,bUseCodegen=false, ...
    bIncludeTransverse=true,ui32ShadowLevel=uint32(0),strPreparation=strDiskContext);
assert(norm(EvaluateSrpResponseLut([0;0;1],strAfterDisk.strResponseLut,true)) > 0);
assert(strDiskContext.ui32VisibilityBuildCount==1);

% Fine grids use explicit capacities; constant fields provide independent interpolation oracles.
strFine = struct('dAzimuth',-180:0.5:180,'dElevation',-90:0.5:90, ...
    'dEffectiveCr',ones(361,721),'dReferenceArea_m2',0.5, ...
    'dTransverseForcePerPressure',zeros(3,361,721), ...
    'dTorquePerPressure',repmat([1;2;3],1,361,721));
strFinePayload = PackSrpResponseLut(strFine,ui32Capacity=uint32([721,361]), ...
    bIncludeTransverse=true,bIncludeTorque=true);
[dFineForce,~,~,dFineTorque] = EvaluateSrpResponseLut([1;2;3],strFinePayload,true);
assert(norm(dFineForce+0.5*[1;2;3]/sqrt(14))<1e-12 && norm(dFineTorque-[1;2;3])<1e-12);
dMexResidual = 0;
if bCompile
    strCompiled = BuildSrpResponseLut(strPanel,0.5,90,bIncludeTransverse=true, ...
        bIncludeTorque=true,ui32ShadowLevel=uint32(0),charCacheDirectory=charTemporary);
    dMexResidual = max(abs(strSource.dForcePerPressure-strCompiled.dForcePerPressure),[],'all');
    assert(dMexResidual<1e-12 && ...
        max(abs(strSource.dTorquePerPressure-strCompiled.dTorquePerPressure),[],'all')<1e-12);
end
strChecks = struct('ui32IndependentChecks',uint32(7),'bCompiled',bCompile, ...
    'dMexForceResidual',dMexResidual);
disp(strChecks);
end
