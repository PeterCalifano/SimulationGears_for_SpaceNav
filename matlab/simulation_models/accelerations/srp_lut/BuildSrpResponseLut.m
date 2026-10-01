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
% kwargs.fcnVisibility      Selected owner source/MEX visibility function.
% kwargs.bIncludeTransverse Include transverse storage in the numeric payload; default false.
% kwargs.bCompactPayload    Match fixed capacity to grid dimensions; default true.
% kwargs.strSpacecraftSrp   Optional validated descriptor identities/assignments.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strLut                    Response arrays, numeric payload and host provenance.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Move generic response construction to SimulationGears.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildPanelSrpShadowData, ComputeQuadsModelSRP, ComputePanelSunVisibility,
% PackSrpResponseLut, ComputeFileSha256; Java SHA-256 for host provenance.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strPanel (1, 1) struct
    dReferenceArea (1, 1) double {mustBeFinite, mustBePositive}
    dAngularGridStep (1, 1) double {mustBeFinite, mustBeGreaterThanOrEqual(dAngularGridStep, 1)}
    kwargs.ui32ShadowLevel (1, 1) uint32 = uint32(3)
    kwargs.dRayOffset (1, 1) double {mustBeFinite, mustBePositive} = 1e-8
    kwargs.bSelfShadowing (1, 1) logical = true
    kwargs.fcnVisibility (1, 1) function_handle = @ComputePanelSunVisibility
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
dEffectiveCr = zeros(numel(dElevation), numel(dAzimuth));
dForcePerPressure = zeros(3, numel(dElevation), numel(dAzimuth));
dTransverseForcePerPressure = zeros(size(dForcePerPressure));

% Sample the panel law over the complete sphere before enforcing periodic boundaries.
for ui32Elevation = uint32(1):uint32(numel(dElevation))

    for ui32Azimuth = uint32(1):uint32(numel(dAzimuth))

        dElevationCosine = cosd(dElevation(ui32Elevation));
        dSunDir_SCB = [dElevationCosine * cosd(dAzimuth(ui32Azimuth)); ...
                      dElevationCosine * sind(dAzimuth(ui32Azimuth)); ...
                      sind(dElevation(ui32Elevation))];

        dVisiblePanelAreas = strPanel.dSCquadsArea;

        if kwargs.bSelfShadowing
            % Scale each panel area by its own equal-area visibility samples.
            dVisiblePanelAreas = dVisiblePanelAreas .* ...
                kwargs.fcnVisibility(dSunDir_SCB, strPanel.dQuadsNormals_SCB, ...
                                     dSamplePoints_SCB, dFaceVertices_SCB, kwargs.dRayOffset);
        end

        % Use unit mass and pressure so the panel acceleration equals force per pressure.
        dForcePerPressure_SCB = ComputeQuadsModelSRP(dSunDir_SCB, [1;0;0;0], 1, zeros(3, 1), 1, ...
                                                   dVisiblePanelAreas, strPanel.dDiffSpecQuadsCoeffs, ...
                                                   strPanel.dQuadsNormals_SCB, ...
                                                   strPanel.dQuadsPressCentre_SCB);

        dForcePerPressure(:, ui32Elevation, ui32Azimuth) = dForcePerPressure_SCB;
        dEffectiveCr(ui32Elevation, ui32Azimuth) = dot(dForcePerPressure_SCB, -dSunDir_SCB) / dReferenceArea;

        % Remove the nodal parallel component before interpolating across directions.
        dTransverseForcePerPressure(:, ui32Elevation, ui32Azimuth) = ...
            dForcePerPressure_SCB - dSunDir_SCB * dot(dSunDir_SCB, dForcePerPressure_SCB);
    end

    if mod(ui32Elevation, 10) == 0
        fprintf('LUT elevation row %u/%u complete.\n', ui32Elevation, numel(dElevation));
    end
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

strLut.strResponseLut = PackSrpResponseLut(strLut, ui32Capacity=ui32GridCapacity, ...
                                           bIncludeTransverse=kwargs.bIncludeTransverse);
coder.cstructname(strLut, 'SPanelSrpResponseLut');

end
