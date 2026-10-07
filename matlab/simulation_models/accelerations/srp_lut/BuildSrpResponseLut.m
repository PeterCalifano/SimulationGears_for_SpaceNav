function strLut = BuildSrpResponseLut(strPanel, dReferenceArea, dAngularGridStep, kwargs)
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
% dAngularGridStep          Angular grid spacing [deg], at least one degree.
% kwargs.ui32ShadowLevel    Equal-area subdivision level; default three.
% kwargs.dRayOffset         Ray-origin offset/tolerance [m]; default 1e-8.
% kwargs.bSelfShadowing     Enable opaque two-sided mesh occlusion; default true.
% kwargs.fcnPanelResponse   Selected complete owner source/MEX panel evaluator.
% kwargs.ui32BatchCount      Fixed evaluator direction capacity; default one.
% kwargs.bIncludeTransverse Include transverse storage in the numeric payload; default false.
% kwargs.bCompactPayload    Match fixed capacity to grid dimensions; default true.
% kwargs.strSpacecraftSrp   Optional validated descriptor identities/assignments.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strLut                    Response arrays, numeric payload and host provenance.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-09-2026  Pietro Califano, Codex gpt-6  Move generic response construction to SimulationGears.
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% 05-10-2026  Pietro Califano, Codex (GPT-6)        Batch the complete prepared panel evaluator.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildPanelSrpShadowData, BuildTriangleRayData, ComputePanelSrpResponse,
% PackSrpResponseLut, ComputeFileSha256; Java SHA-256 for host provenance.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strPanel (1, 1) struct
    dReferenceArea (1, 1) double {mustBeFinite, mustBePositive}
    dAngularGridStep (1, 1) double {mustBeFinite, mustBeGreaterThanOrEqual(dAngularGridStep, 1)}
    kwargs.ui32ShadowLevel (1, 1) uint32 = uint32(3)
    kwargs.dRayOffset (1, 1) double {mustBeFinite, mustBePositive} = 1e-8
    kwargs.bSelfShadowing (1, 1) logical = true
    kwargs.fcnPanelResponse (1, 1) function_handle = @ComputePanelSrpResponse
    kwargs.ui32BatchCount (1, 1) uint32 {mustBePositive} = uint32(1)
    kwargs.bIncludeTransverse (1, 1) logical = false
    kwargs.bCompactPayload (1, 1) logical = true
    kwargs.strSpacecraftSrp (1, 1) struct = struct()
end

arguments (Output)
    strLut (1, 1) struct
end

% Prepare one reusable geometry payload and retain an optional exact mesh identity.
[dFaceVertices_SCB, dSamplePoints_SCB] = BuildPanelSrpShadowData(strPanel, kwargs.ui32ShadowLevel);
charGeometrySha256 = '';
if isfield(strPanel, 'charSourceObjFilePath') && ~isempty(strPanel.charSourceObjFilePath)
    charGeometrySha256 = ComputeFileSha256(strPanel.charSourceObjFilePath);
end

% Require complete-sphere axes with an integer number of uniform cells.
assert(abs(180 / dAngularGridStep - round(180 / dAngularGridStep)) < 1e-9);
dAzimuth = -180:dAngularGridStep:180;
dElevation = -90:dAngularGridStep:90;

% Evaluate the complete law in fixed batches, including the partially filled tail.
% Keep parsing and rich metadata out of the generated response signature.
strNumericPanel = struct('dSCquadsArea', strPanel.dSCquadsArea(:), ...
    'dDiffSpecQuadsCoeffs', strPanel.dDiffSpecQuadsCoeffs, ...
    'dQuadsNormals_SCB', strPanel.dQuadsNormals_SCB, ...
    'dQuadsPressCentre_SCB', strPanel.dQuadsPressCentre_SCB, ...
    'strShadowData', struct('dSamplePoints_SCB', dSamplePoints_SCB, ...
        'dRayOffset', kwargs.dRayOffset, ...
        'strRayData', BuildTriangleRayData(dFaceVertices_SCB, false)));
[dAzimuthNodes, dElevationNodes] = meshgrid(dAzimuth, dElevation);
dDirections = [cosd(dElevationNodes(:)).'.*cosd(dAzimuthNodes(:)).'; ...
    cosd(dElevationNodes(:)).'.*sind(dAzimuthNodes(:)).'; sind(dElevationNodes(:)).'];
dResponses = zeros(size(dDirections));
ui32NodeCount = uint32(size(dDirections, 2));
for ui32First = uint32(1):kwargs.ui32BatchCount:ui32NodeCount
    ui32Last = min(ui32First + kwargs.ui32BatchCount - 1, ui32NodeCount);
    ui32ActiveCount = ui32Last - ui32First + 1;
    dBatch = repmat(dDirections(:, ui32First), 1, kwargs.ui32BatchCount);
    dBatch(:, 1:ui32ActiveCount) = dDirections(:, ui32First:ui32Last);
    dResponse = kwargs.fcnPanelResponse(dBatch, strNumericPanel, kwargs.bSelfShadowing);
    dResponses(:, ui32First:ui32Last) = dResponse(:, 1:ui32ActiveCount);
end
dForcePerPressure = reshape(dResponses, 3, numel(dElevation), numel(dAzimuth));
dEffectiveCr = reshape(sum(-dResponses.*dDirections, 1)/dReferenceArea, ...
    numel(dElevation), numel(dAzimuth));
dTransverseForcePerPressure = reshape( ...
    dResponses - dDirections.*sum(dDirections.*dResponses, 1), size(dForcePerPressure));

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
    'charPanelSha256', ComputeFileSha256(which('ComputePanelSrpResponse')), ...
    'charVisibilitySha256', ComputeFileSha256(which('ComputePreparedPanelVisibility')), ...
    'charTracingSha256', ComputeFileSha256(which('TraceTriangleRay')), ...
    'charTracingArraysSha256', ComputeFileSha256(which('TraceTriangleRayArrays')), ...
    'charGeometryBuilderSha256', ComputeFileSha256(which('BuildTriangleRayData')), ...
    'charTraversal', 'Conservative projected flat scan');
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
end

strLut.strResponseLut = PackSrpResponseLut(strLut, ui32Capacity = ui32GridCapacity, ...
                                           bIncludeTransverse = kwargs.bIncludeTransverse);
coder.cstructname(strLut, 'SPanelSrpResponseLut');

end
