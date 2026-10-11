function [strLut, strPreparation] = BuildSrpResponseLut(strPanel, dReferenceArea, dAngularGridStep, kwargs)
%% SIGNATURE
% strLut = BuildSrpResponseLut(strPanel, dReferenceArea, dAngularGridStep, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate a spacecraft-generic scalar and transverse SRP response over the
% complete sphere. Evaluate the owning panel law and optional self-shadowing;
% never fit a trajectory. Preserve exact seam/pole equality and bind prepared
% geometry, optical coefficients, normal convention and source identities to
% the host artifact. Keep the deployable payload numeric and fixed-size.
% Example: strLut = BuildSrpResponseLut(strPanel, 0.5329, 5, bSelfShadowing=false);
% Output: A 73-by-37 table with independent scalar and vector entries.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strPanel                  Prepared panel geometry in metres and areas in m^2.
% dReferenceArea            Positive scalar-model reference area [m^2].
% dAngularGridStep          Angular grid spacing [deg], at least half a degree.
% kwargs.ui32ShadowLevel    Equal-area subdivision level; default three.
% kwargs.dRayOffset         Ray-origin offset/tolerance [m]; default 1e-8.
% kwargs.bSelfShadowing     Enable opaque two-sided mesh occlusion; default true.
% kwargs.fcnVisibility      Optional diagnostic visibility override; empty uses the numeric grid kernel.
% kwargs.fcnPanelResponse   Optional complete source/MEX response override (three-input signature).
%                          Owns shadowing; incompatible with visibility overrides and disk caching.
% kwargs.ui32BatchCount     Fixed direction capacity; default 64, including padded tail batches.
% kwargs.bUseCodegen        Compile complete numeric batches by default; false evaluates the same MATLAB code.
% kwargs.strPreparation    Host preparation context reused across samples.
% kwargs.charCacheDirectory Cache root; empty uses COSMICA_SRP_LUT_CACHE_DIR or tempdir.
% kwargs.bIncludeTorque    Include body-origin torque samples for truth propagation.
% kwargs.bUseDiskCache     Reuse/publish immutable complete tables for truth workers; default false.
% kwargs.bIncludeTransverse Include transverse storage in the numeric payload; default false.
% kwargs.bCompactPayload    Match fixed capacity to grid dimensions; default true.
% kwargs.strSpacecraftSrp   Optional validated descriptor identities/assignments.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strLut                    Response arrays, numeric payload and host provenance.
% strPreparation            Geometry/visibility, provider and last response reuse context.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Move generic response construction to SimulationGears.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% 06-10-2026  Codex (GPT-6)  Reuse geometry and responses with default complete numeric MEX batches.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildPanelSrpShadowData, ComputeSrpLutGrid, CodegenSrpLutConstruction,
% PackSrpResponseLut, ComputeFileSha256; Java SHA-256 for host provenance.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strPanel (1, 1) struct
    dReferenceArea (1, 1) double {mustBeFinite, mustBePositive}
    dAngularGridStep (1, 1) double {mustBeFinite, mustBeGreaterThanOrEqual(dAngularGridStep, 0.5)}
    kwargs.ui32ShadowLevel (1, 1) uint32 = uint32(3)
    kwargs.dRayOffset (1, 1) double {mustBeFinite, mustBePositive} = 1e-8
    kwargs.bSelfShadowing (1, 1) logical = true
    kwargs.fcnVisibility = []
    kwargs.fcnPanelResponse = []
    kwargs.ui32BatchCount (1,1) uint32 {mustBePositive} = uint32(64)
    kwargs.bUseCodegen (1,1) logical = true
    kwargs.strPreparation (1,1) struct = struct()
    kwargs.charCacheDirectory (1,:) char = ''
    kwargs.bIncludeTorque (1,1) logical = false
    kwargs.bUseDiskCache (1,1) logical = false
    kwargs.bIncludeTransverse (1, 1) logical = false
    kwargs.bCompactPayload (1, 1) logical = true
    kwargs.strSpacecraftSrp (1, 1) struct = struct()
end

arguments (Output)
    strLut (1, 1) struct
    strPreparation (1,1) struct
end

% Validate host inputs once before evaluating any numerical batches.
assert(isempty(kwargs.fcnVisibility) || isa(kwargs.fcnVisibility,'function_handle'), ...
    'BuildSrpResponseLut:VisibilityProvider','Supply a function handle or empty default.');
assert(isempty(kwargs.fcnPanelResponse) || isa(kwargs.fcnPanelResponse,'function_handle'), ...
    'BuildSrpResponseLut:ResponseProvider','Supply a function handle or empty default.');
assert(isempty(kwargs.fcnPanelResponse) || ...
    (isempty(kwargs.fcnVisibility) && ~kwargs.bUseDiskCache), ...
    'BuildSrpResponseLut:ProviderPolicy', ...
    'A complete response override owns visibility and cannot use the built-in disk cache.');
assert(kwargs.ui32ShadowLevel <= 5,'BuildSrpResponseLut:ShadowLevel','Shadow depth must not exceed five.');
assert(abs(180/dAngularGridStep-round(180/dAngularGridStep)) < 1e-9, ...
    'BuildSrpResponseLut:GridStep','Grid spacing must divide 180 degrees.');
strPreparation = kwargs.strPreparation;
dNormals = strPanel.dQuadsNormals_SCB;
dAreas = strPanel.dSCquadsArea(:);
dCoefficients = strPanel.dDiffSpecQuadsCoeffs;
ui32FaceCount = uint32(numel(dAreas));
assert(isequal(size(dNormals),[3,double(ui32FaceCount)]) && all(isfinite(dNormals),'all') && ...
    all(vecnorm(dNormals)>0) && all(isfinite(dAreas)) && all(dAreas>=0) && ...
    isequal(size(dCoefficients),[double(ui32FaceCount),2]) && all(isfinite(dCoefficients),'all') && ...
    all(dCoefficients>=0,'all') && all(sum(dCoefficients,2)<=1+8*eps), ...
    'BuildSrpResponseLut:PanelData','Supply finite physical optics, areas and nonzero normals.');

% Prepare quadrature only when the caller does not already supply the exact layout.
dExpectedVertices = reshape(strPanel.dVerticesPos(strPanel.ui32FaceVertexIds.',:).',3,3,[]);
if isfield(strPanel,'strShadowData') && ...
        isfield(strPanel.strShadowData,'dFaceVertices_SCB') && ...
        size(strPanel.strShadowData.dSamplePoints_SCB,2) == 4^double(kwargs.ui32ShadowLevel) && ...
        isequal(strPanel.strShadowData.dFaceVertices_SCB,dExpectedVertices)
    dFaceVertices_SCB = strPanel.strShadowData.dFaceVertices_SCB;
    dSamplePoints_SCB = strPanel.strShadowData.dSamplePoints_SCB;
else
    [dFaceVertices_SCB,dSamplePoints_SCB] = BuildPanelSrpShadowData(strPanel,kwargs.ui32ShadowLevel);
end
strGeometry = struct('dNormals',dNormals,'dVertices',dFaceVertices_SCB,'dSamples',dSamplePoints_SCB, ...
    'dGridStep',dAngularGridStep,'bSelfShadowing',kwargs.bSelfShadowing, ...
    'dRayOffset',kwargs.dRayOffset,'fcnVisibility',kwargs.fcnVisibility, ...
    'fcnPanelResponse',kwargs.fcnPanelResponse);
if ~isfield(strPreparation,'cellSourceHashes')
    cellOwners = {'BuildSrpResponseLut','BuildPanelSrpShadowData','ComputeSrpLutGrid', ...
        'ComputePanelSrpResponse','ComputePreparedPanelVisibility','ComputePanelSunVisibility', ...
        'CodegenSrpLutConstruction','PackSrpResponseLut','ValidateSrpResponseLut', ...
        'BuildTriangleRayData'};
    strPreparation.cellSourceHashes = cellfun(@(charOwner) ComputeFileSha256(which(charOwner)), ...
        cellOwners,'UniformOutput',false);
end
bReuseGeometry = isfield(strPreparation,'strGeometry') && ...
    isequaln(strPreparation.strGeometry,strGeometry);
if ~isfield(strPreparation,'ui32LutBuildCount')
    strPreparation.ui32LutBuildCount = uint32(0);
    strPreparation.ui32VisibilityBuildCount = uint32(0);
    strPreparation.ui32MexBuildCount = uint32(0);
end
dAzimuth = -180:dAngularGridStep:180;
dElevation = -90:dAngularGridStep:90;
if ~bReuseGeometry
    % Include each pole once and omit the redundant azimuth seam.
    [dAzimuthNodes,dElevationNodes] = meshgrid(dAzimuth(1:end-1),dElevation(2:end-1));
    dDirections = [[0;0;-1], [cosd(dElevationNodes(:)).'.*cosd(dAzimuthNodes(:)).'; ...
        cosd(dElevationNodes(:)).'.*sind(dAzimuthNodes(:)).'; sind(dElevationNodes(:)).'], [0;0;1]];
    dVertex0 = reshape(dFaceVertices_SCB(:,1,:),3,[]);
    strEdges = struct('dVertex0',dVertex0, ...
        'dEdge1',reshape(dFaceVertices_SCB(:,2,:),3,[])-dVertex0, ...
        'dEdge2',reshape(dFaceVertices_SCB(:,3,:),3,[])-dVertex0);
    strShadow = struct('dSamplePoints_SCB',dSamplePoints_SCB,'dFaceVertices_SCB',dFaceVertices_SCB, ...
        'dRayOffset',kwargs.dRayOffset,'strTriangleEdges',strEdges);
    strPreparation.strGeometry = strGeometry;
    strPreparation.dDirections = dDirections;
    strPreparation.strNumericPanel = struct('dSCquadsArea',dAreas, ...
        'dDiffSpecQuadsCoeffs',dCoefficients,'dQuadsNormals_SCB',dNormals, ...
        'dQuadsPressCentre_SCB',strPanel.dQuadsPressCentre_SCB,'strShadowData',strShadow);
    if kwargs.bSelfShadowing
        strPreparation.dVisibility = zeros(double(ui32FaceCount),size(dDirections,2));
    else
        strPreparation.dVisibility = zeros(double(ui32FaceCount),0);
    end
    strPreparation.bVisibilityReady = ~kwargs.bSelfShadowing;
    strHashGeometry = rmfield(strGeometry,{'fcnVisibility','fcnPanelResponse'});
    strHashGeometry.charVisibilityProvider = ProviderName_(kwargs.fcnVisibility);
    strHashGeometry.charResponseProvider = ProviderName_(kwargs.fcnPanelResponse);
    strPreparation.charGeometryKey = HashInputs_(struct('strGeometry',strHashGeometry, ...
        'cellSources',{strPreparation.cellSourceHashes}));
end

% Response identity contains every optical/area input; unrelated dynamics never enter it.
strResponseIdentity = struct('charGeometryKey',strPreparation.charGeometryKey, ...
    'dAreas',dAreas,'dCoefficients',dCoefficients,'dReferenceArea',dReferenceArea, ...
    'bCompact',kwargs.bCompactPayload,'bTransverse',kwargs.bIncludeTransverse, ...
    'bTorque',kwargs.bIncludeTorque,'strSpacecraftSrp',kwargs.strSpacecraftSrp);
if kwargs.bIncludeTorque
    strResponseIdentity.dPressureCentres = strPanel.dQuadsPressCentre_SCB;
end
charResponseKey = HashInputs_(strResponseIdentity);
if isempty(kwargs.fcnPanelResponse) && bReuseGeometry && isfield(strPreparation,'charResponseKey') && ...
        strcmp(strPreparation.charResponseKey,charResponseKey)
    strLut = strPreparation.strLut;
    return
end
charTableFile = '';
if kwargs.bUseDiskCache
    charCacheRoot = kwargs.charCacheDirectory;
    if isempty(charCacheRoot)
        charCacheRoot = getenv('COSMICA_SRP_LUT_CACHE_DIR');
        if isempty(charCacheRoot)
            charCacheRoot = fullfile(tempdir,'cosmica-srp-lut-cache');
        end
    end
    charTableFile = fullfile(charCacheRoot,'tables',[charResponseKey,'.mat']);
    if isfile(charTableFile)
        strStored = load(charTableFile,'strLut','charResponseKey');
        assert(strcmp(strStored.charResponseKey,charResponseKey), ...
            'BuildSrpResponseLut:TableIdentity','Reject a mismatched cached response.');
        strLut = strStored.strLut;
        ValidateSrpResponseLut(strLut.strResponseLut);
        strPreparation.charResponseKey = charResponseKey;
        strPreparation.strLut = strLut;
        return
    end
end
strNumericPanel = strPreparation.strNumericPanel;
strNumericPanel.dSCquadsArea = dAreas;
strNumericPanel.dDiffSpecQuadsCoeffs = dCoefficients;
strNumericPanel.dQuadsPressCentre_SCB = strPanel.dQuadsPressCentre_SCB;
dDirections = strPreparation.dDirections;
ui32BatchCount = kwargs.ui32BatchCount;
fcnGrid = @ComputeSrpLutGrid;
if ~isempty(kwargs.fcnPanelResponse)
    % Keep the branch's complete-provider ABI independent of the cached grid ABI.
    strNumericPanel.strShadowData = struct('dSamplePoints_SCB',dSamplePoints_SCB, ...
        'dRayOffset',kwargs.dRayOffset,'strRayData',BuildTriangleRayData(dFaceVertices_SCB,false));
elseif kwargs.bUseCodegen
    [fcnGrid,bBuilt] = ResolveGridMex_(strNumericPanel,ui32BatchCount,kwargs.bIncludeTorque, ...
        strPreparation.cellSourceHashes,kwargs.charCacheDirectory);
    strPreparation.ui32MexBuildCount = strPreparation.ui32MexBuildCount + uint32(bBuilt);
end

% Diagnostic overrides share the same response stage but supply their own visibility.
bReuseVisibility = strPreparation.bVisibilityReady;
if ~bReuseVisibility && kwargs.bSelfShadowing
    strPreparation.ui32VisibilityBuildCount = strPreparation.ui32VisibilityBuildCount + uint32(1);
end
if ~bReuseVisibility && kwargs.bSelfShadowing && ~isempty(kwargs.fcnVisibility)
    for ui32Direction = uint32(1):uint32(size(dDirections,2))
        strPreparation.dVisibility(:,ui32Direction) = kwargs.fcnVisibility( ...
            dDirections(:,ui32Direction),dNormals,dSamplePoints_SCB,dFaceVertices_SCB,kwargs.dRayOffset);
    end
    bReuseVisibility = true;
end
ui32NodeCount = uint32(size(dDirections,2));
dResponses = zeros(3,double(ui32NodeCount));
dTorques = zeros(3,0);
if kwargs.bIncludeTorque
    dTorques = zeros(3,double(ui32NodeCount));
end
dBatch = zeros(3,double(ui32BatchCount));
dVisibleBatch = ones(double(ui32FaceCount),double(ui32BatchCount));
for ui32First = uint32(1):ui32BatchCount:ui32NodeCount
    ui32Last = min(ui32First+ui32BatchCount-1,ui32NodeCount);
    ui32Active = ui32Last-ui32First+1;
    dBatch(:,:) = repmat(dDirections(:,ui32First),1,double(ui32BatchCount));
    dBatch(:,1:ui32Active) = dDirections(:,ui32First:ui32Last);
    if bReuseVisibility && kwargs.bSelfShadowing
        dVisibleBatch(:,1:ui32Active) = strPreparation.dVisibility(:,ui32First:ui32Last);
        dVisibleBatch(:,ui32Active+1:end) = 1;
    end
    if ~isempty(kwargs.fcnPanelResponse)
        if kwargs.bIncludeTorque
            [dResponse,~,dTorqueBatch] = kwargs.fcnPanelResponse( ...
                dBatch,strNumericPanel,kwargs.bSelfShadowing);
            dTorques(:,ui32First:ui32Last) = dTorqueBatch(:,1:ui32Active);
        else
            dResponse = kwargs.fcnPanelResponse(dBatch,strNumericPanel,kwargs.bSelfShadowing);
        end
    elseif kwargs.bIncludeTorque
        [dResponse,dVisibleBatch,dTorqueBatch] = fcnGrid( ...
            dBatch,strNumericPanel,kwargs.bSelfShadowing,bReuseVisibility,dVisibleBatch);
        dTorques(:,ui32First:ui32Last) = dTorqueBatch(:,1:ui32Active);
    else
        [dResponse,dVisibleBatch] = fcnGrid( ...
            dBatch,strNumericPanel,kwargs.bSelfShadowing,bReuseVisibility,dVisibleBatch);
    end
    dResponses(:,ui32First:ui32Last) = dResponse(:,1:ui32Active);
    if kwargs.bSelfShadowing
        strPreparation.dVisibility(:,ui32First:ui32Last) = dVisibleBatch(:,1:ui32Active);
    end
end
strPreparation.bVisibilityReady = isempty(kwargs.fcnPanelResponse);
strPreparation.ui32LutBuildCount = strPreparation.ui32LutBuildCount + uint32(1);

% Restore the established complete-sphere layout with exact periodic boundaries.
dForcePerPressure = RestoreGrid_(dResponses,numel(dElevation),numel(dAzimuth));
[dAzimuthNodes,dElevationNodes] = meshgrid(dAzimuth,dElevation);
dFullDirections = [cosd(dElevationNodes(:)).'.*cosd(dAzimuthNodes(:)).'; ...
    cosd(dElevationNodes(:)).'.*sind(dAzimuthNodes(:)).'; sind(dElevationNodes(:)).'];
dFullResponses = reshape(dForcePerPressure,3,[]);
dEffectiveCr = reshape(sum(-dFullResponses.*dFullDirections,1)/dReferenceArea, ...
    numel(dElevation),numel(dAzimuth));
dTransverseForcePerPressure = reshape(dFullResponses - ...
    dFullDirections.*sum(dFullDirections.*dFullResponses,1),size(dForcePerPressure));
charGeometrySha256 = '';
if isfield(strPanel,'charSourceObjFilePath') && ~isempty(strPanel.charSourceObjFilePath)
    charGeometrySha256 = ComputeFileSha256(strPanel.charSourceObjFilePath);
end

% Make periodic seam and pole values exact rather than relying on roundoff.
dEffectiveCr(:, end) = dEffectiveCr(:, 1);
dEffectiveCr(1, :) = dEffectiveCr(1, 1);
dEffectiveCr(end, :) = dEffectiveCr(end, 1);
dForcePerPressure(:, :, end) = dForcePerPressure(:, :, 1);
dForcePerPressure(:, 1, :) = repmat(dForcePerPressure(:, 1, 1), 1, 1, numel(dAzimuth));
dForcePerPressure(:, end, :) = repmat(dForcePerPressure(:, end, 1), 1, 1, numel(dAzimuth));

dTransverseForcePerPressure(:, :, end) = dTransverseForcePerPressure(:, :, 1);
dTransverseForcePerPressure(:, 1, :) = repmat(dTransverseForcePerPressure(:, 1, 1), 1, 1, numel(dAzimuth));
dTransverseForcePerPressure(:, end, :) = repmat(dTransverseForcePerPressure(:, end, 1), 1, 1, numel(dAzimuth));

% Bind the host artifact to its geometry, optical law and construction policy.
strLut = struct('dAzimuth', dAzimuth, 'dElevation', dElevation, 'dEffectiveCr', dEffectiveCr, ...
    'dForcePerPressure', dForcePerPressure, ...
    'dTransverseForcePerPressure', dTransverseForcePerPressure, 'dReferenceArea_m2', dReferenceArea, ...
    'charSunDirection', 'Spacecraft-to-Sun in spacecraft mesh frame', ...
    'charAxisUnits', 'deg', 'charValueUnits', 'Dimensionless effective Cr', ...
    'charVectorValueUnits', 'm^2', ...
    'charInterpolation', 'Bilinear scalar and nodal transverse vector; periodic azimuth and constant poles', ...
    'bSelfShadowing', kwargs.bSelfShadowing, 'charGeometrySha256', charGeometrySha256);

strLut.dOpticalCoefficients = strPanel.dDiffSpecQuadsCoeffs;
strLut.ui32SamplesPerFace = uint32(size(dSamplePoints_SCB, 2));
strLut.dRayOffset = kwargs.dRayOffset;
strLut.charTransversePolicy = 'Project interpolated nodal transverse samples; retain the scalar parallel coefficient';
strLut.charBodyMounting = 'Identity mounting in the spacecraft mesh frame';
strLut.charNormalConvention = 'Caller-prepared optical normals; two-sided opaque occluders';
strLut.charGeneratorSha256 = ComputeFileSha256([mfilename('fullpath'), '.m']);
strLut.strResponseImplementation = struct( ...
    'charPanelSha256',ComputeFileSha256(which('ComputePanelSrpResponse')), ...
    'charVisibilitySha256',ComputeFileSha256(which('ComputePreparedPanelVisibility')), ...
    'charGeometryBuilderSha256',ComputeFileSha256(which('BuildTriangleRayData')), ...
    'charResponseProvider',ProviderName_(kwargs.fcnPanelResponse), ...
    'charTraversal','Conservative projected flat scan');
strLut.strSpacecraftModel = struct('dVertices', strPanel.dVerticesPos, ...
    'ui32Faces', strPanel.ui32FaceVertexIds, 'dAreas', strPanel.dSCquadsArea, ...
    'dNormals', strPanel.dQuadsNormals_SCB, 'dOptics', strPanel.dDiffSpecQuadsCoeffs, ...
    'charArticulation', 'Fixed', 'charBodyMounting', strLut.charBodyMounting);

if ~isempty(fieldnames(kwargs.strSpacecraftSrp))
    % Invalidate rich LUT identity when the descriptor or assignments change.
    strLut.strSpacecraftSrp = kwargs.strSpacecraftSrp;
    strLut.strSpacecraftModel.charDescriptorSha256 = kwargs.strSpacecraftSrp.charDescriptorSha256;
    strLut.strSpacecraftModel.ui32FamilyByTriangle = kwargs.strSpacecraftSrp.ui32FamilyByTriangle;
end

% Hash the prepared geometry and optical assignments independently of the source file.
objDigest = java.security.MessageDigest.getInstance('SHA-256');
objDigest.update(unicode2native(jsonencode(strLut.strSpacecraftModel), 'UTF-8'));
strLut.charSpacecraftModelSha256 = lower(reshape(dec2hex(typecast(objDigest.digest(), 'uint8'), 2).', 1, []));
strLut.charProvenance = 'DERIVED; direct panel-law and mesh-visibility evaluations';

% Pack only fixed-size numeric data for runtime and generated-code consumers.
assert(all(isfinite(dEffectiveCr), 'all') && all(dEffectiveCr >= 0, 'all'));
ui32GridCapacity = uint32([361, 181]);

if kwargs.bCompactPayload
    ui32GridCapacity = uint32([numel(dAzimuth), numel(dElevation)]);
else
    ui32GridCapacity = max(ui32GridCapacity,uint32([numel(dAzimuth),numel(dElevation)]));
end

if kwargs.bIncludeTorque
    strLut.dTorquePerPressure = RestoreGrid_(dTorques,numel(dElevation),numel(dAzimuth));
end
strLut.strResponseLut = PackSrpResponseLut(strLut,ui32Capacity=ui32GridCapacity, ...
    bIncludeTransverse=kwargs.bIncludeTransverse,bIncludeTorque=kwargs.bIncludeTorque);
strLut.strPreparationCounts = struct('ui32LutBuildCount',strPreparation.ui32LutBuildCount, ...
    'ui32VisibilityBuildCount',strPreparation.ui32VisibilityBuildCount, ...
    'ui32MexBuildCount',strPreparation.ui32MexBuildCount);
strPreparation.charResponseKey = charResponseKey;
strPreparation.strLut = strLut;
if kwargs.bUseDiskCache
    charTableRoot = fileparts(charTableFile);
    if ~isfolder(charTableRoot)
        mkdir(charTableRoot);
    end
    charTemporaryTable = [tempname(charTableRoot),'.mat'];
    save(charTemporaryTable,'strLut','charResponseKey','-v7');
    [bPublished,charMessage] = movefile(charTemporaryTable,charTableFile,'f');
    assert(bPublished,'BuildSrpResponseLut:TablePublicationFailed','%s',charMessage);
end
coder.cstructname(strLut, 'SPanelSrpResponseLut');

end

function charDigest = HashInputs_(strInputs)
%% SIGNATURE
% charDigest = HashInputs_(strInputs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Hash host preparation inputs using the existing deterministic JSON/SHA-256 convention.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strInputs  Complete identity inputs.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charDigest Lowercase SHA-256 digest.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Separate geometry, response and compiled-provider identities.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Java SHA-256, jsonencode.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strInputs (1,1) struct
end
arguments (Output)
    charDigest (1,:) char
end
objDigest = java.security.MessageDigest.getInstance('SHA-256');
objDigest.update(unicode2native(jsonencode(strInputs),'UTF-8'));
charDigest = lower(reshape(dec2hex(typecast(objDigest.digest(),'uint8'),2).',1,[]));
end

function charName = ProviderName_(fcnProvider)
%% SIGNATURE
% charName = ProviderName_(fcnProvider)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Name an explicit diagnostic override without assigning a callback to the default provider.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnProvider Optional function handle.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charName    Provider name or empty default marker.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Bind diagnostic callbacks to geometry reuse.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% func2str.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    fcnProvider
end
arguments (Output)
    charName (1,:) char
end
charName = '';
if ~isempty(fcnProvider)
    charName = func2str(fcnProvider);
end
end

function dGrid = RestoreGrid_(dUnique,dElevationCount,dAzimuthCount)
%% SIGNATURE
% dGrid = RestoreGrid_(dUnique,dElevationCount,dAzimuthCount)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Restore unique directions to the public complete sphere with exact seam and pole copies.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dUnique         South pole, interior directions, and north pole responses.
% dElevationCount Number of public elevation nodes.
% dAzimuthCount   Number of public azimuth nodes.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dGrid           Complete three-component response grid.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Eliminate duplicate sphere evaluations.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dUnique (3,:) double
    dElevationCount (1,1) double
    dAzimuthCount (1,1) double
end
arguments (Output)
    dGrid (3,:,:) double
end
dGrid = zeros(3,dElevationCount,dAzimuthCount);
dGrid(:,2:end-1,1:end-1) = reshape(dUnique(:,2:end-1),3,dElevationCount-2,dAzimuthCount-1);
dGrid(:,1,:) = repmat(dUnique(:,1),1,1,dAzimuthCount);
dGrid(:,end,:) = repmat(dUnique(:,end),1,1,dAzimuthCount);
dGrid(:,:,end) = dGrid(:,:,1);
end

function [fcnGrid,bBuilt] = ResolveGridMex_(strPanel,ui32BatchCount,bTorque,cellHashes,charCacheDirectory)
%% SIGNATURE
% [fcnGrid,bBuilt] = ResolveGridMex_(strPanel,ui32BatchCount,bTorque,cellHashes,charCacheDirectory)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve one immutable source/layout-bound provider. Compile under a private
% temporary directory and publish only complete artifacts. Numerical sample values
% remain runtime inputs and do not cause additional builds.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strPanel           Numeric panel prototype.
% ui32BatchCount     Fixed direction capacity.
% bTorque            Include torque in the compiled output prefix.
% cellHashes         Numerical source fingerprints.
% charCacheDirectory Explicit cache or environment/default selection.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% fcnGrid            Compiled complete numerical grid provider.
% bBuilt             True only for a newly compiled artifact.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Reuse complete fixed-size construction MEX providers.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CodegenSrpLutConstruction, HashInputs_.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strPanel (1,1) struct
    ui32BatchCount (1,1) uint32
    bTorque (1,1) logical
    cellHashes cell
    charCacheDirectory (1,:) char
end
arguments (Output)
    fcnGrid (1,1) function_handle
    bBuilt (1,1) logical
end
if isempty(charCacheDirectory)
    charCacheDirectory = getenv('COSMICA_SRP_LUT_CACHE_DIR');
    if isempty(charCacheDirectory)
        charCacheDirectory = fullfile(tempdir,'cosmica-srp-lut-cache');
    end
end
strSignature = struct('dFaceCount',numel(strPanel.dSCquadsArea), ...
    'dSampleCount',size(strPanel.strShadowData.dSamplePoints_SCB,2), ...
    'ui32BatchCount',ui32BatchCount,'bTorque',bTorque, ...
    'cellSources',{cellHashes},'charMatlabVersion',version,'charMexExtension',mexext);
charKey = HashInputs_(strSignature);
charKernelName = ['SrpLutGrid_',charKey(1:16)];
charParent = fullfile(charCacheDirectory,'mex');
charArtifact = fullfile(charParent,charKey);
charMexFile = fullfile(charArtifact,[charKernelName,'.',mexext]);
bBuilt = false;
if ~isfile(charMexFile)
    if ~isfolder(charParent)
        mkdir(charParent);
    end
    charTemporary = tempname(charParent);
    objCleanup = onCleanup(@() RemoveTemporary_(charTemporary)); %#ok<NASGU>
    strBuild = CodegenSrpLutConstruction(charTemporary,strPanel, ...
        ui32BatchCount=ui32BatchCount,charKernelName=charKernelName,bIncludeTorque=bTorque);
    strBuild.charOutputRoot = charArtifact;
    save(fullfile(charTemporary,'provider.mat'),'strSignature','strBuild','-v7');
    if isfile(charMexFile)
        % A concurrent publisher may have completed this identical provider.
        RemoveTemporary_(charTemporary);
    else
        [bSuccess,charMessage] = movefile(charTemporary,charArtifact);
        assert(bSuccess,'BuildSrpResponseLut:PublicationFailed','%s',charMessage);
        bBuilt = true;
    end
end
strStored = load(fullfile(charArtifact,'provider.mat'),'strSignature');
assert(isequaln(strStored.strSignature,strSignature), ...
    'BuildSrpResponseLut:ProviderIdentity','Reject a mismatched construction provider.');
addpath(charArtifact);
fcnGrid = str2func(charKernelName);
end

function RemoveTemporary_(charDirectory)
%% SIGNATURE
% RemoveTemporary_(charDirectory)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Remove only the private temporary build directory created by this invocation.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charDirectory Owned unpublished temporary directory.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Clean failed or concurrently superseded private builds.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% rmdir.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charDirectory (1,:) char
end
if isfolder(charDirectory)
    rmdir(charDirectory,'s');
end
end
