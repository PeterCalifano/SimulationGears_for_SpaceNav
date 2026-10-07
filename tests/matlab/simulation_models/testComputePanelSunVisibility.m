function testComputePanelSunVisibility()
%% SIGNATURE
% testComputePanelSunVisibility()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify direct Sun visibility against independent full, partial, absent and
% two-sided occlusion fixtures. Check common-frame/unit transformations, face
% ordering, the ray-offset boundary and identified invalid-input failures.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% Raise an assertion on a visibility-contract failure.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026  Pietro Califano, Codex gpt-6  First fixture harness.
% 30-09-2026  Pietro Califano, Codex gpt-6  Extend geometry invariants and boundary coverage.
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Cover batched response and LUT tail handling.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePanelSunVisibility, ComputePanelSrpResponse, BuildSrpResponseLut,
% BuildTriangleRayData, ComputeQuadsModelSRP.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

% Block the lower face while leaving the upper face exposed to a point-source Sun.
dLowerTriangle = [0, 1, 0; 0, 0, 1; 0, 0, 0];
dUpperTriangle = dLowerTriangle + [0; 0; 1];
dVertices = cat(3, dLowerTriangle, dUpperTriangle);
dNormals = repmat([0;0;1], 1, 2);
dSamples = reshape(mean(dVertices, 2), 3, 1, 2);
dVisible = ComputePanelSunVisibility([0;0;1], dNormals, dSamples, dVertices, 1e-8);
assert(isequal(dVisible, [0;1]), 'Full overlap must block only the lower face.');
assert(isequal(ComputePanelSunVisibility([0;0;17], dNormals, ...
    dSamples, dVertices, 1e-8), dVisible), 'Sun-vector magnitude must not change visibility.');

% Exclude each emitter from its own blocker list, including a single-face mesh.
assert(ComputePanelSunVisibility([0;0;1], dNormals(:, 1), ...
    dSamples(:, :, 1), dLowerTriangle, 1e-8) == 1);

% Preserve blocking when the opaque upper triangle has reversed winding.
dVertices(:, :, 2) = dVertices(:, [1, 3, 2], 2);
assert(isequal(ComputePanelSunVisibility([0;0;1], dNormals, ...
    dSamples, dVertices, 1e-8), [0;1]));

% Resolve a quarter-area blocker with four equal-area barycentric samples.
dVertices(:, :, 2) = 0.5*dLowerTriangle + [0; 0; 1];
dBarycentric = [2/3, 1/6, 1/6, 1/3; 1/6, 2/3, 1/6, 1/3; 1/6, 1/6, 2/3, 1/3];
dSamples = cat(3, dLowerTriangle*dBarycentric, dVertices(:, :, 2)*dBarycentric);
dVisible = ComputePanelSunVisibility([0;0;1], dNormals, dSamples, dVertices, 1e-8);
assert(isequal(dVisible, [0.75;1]), 'A quarter-area blocker must leave 75 percent visible.');

% Preserve face ordering and visibility under a common rigid transform or unit change.
assert(isequal(ComputePanelSunVisibility([0;0;1], dNormals(:, [2, 1]), ...
    dSamples(:, :, [2, 1]), dVertices(:, :, [2, 1]), 1e-8), dVisible([2, 1])));
dRotation = [2/3, -2/3, 1/3; 1/3, 2/3, 2/3; -2/3, -1/3, 2/3];
dTranslation = [3; -4; 5];
dRotatedVertices = pagemtimes(dRotation, dVertices) + dTranslation;
dRotatedSamples = pagemtimes(dRotation, dSamples) + dTranslation;
assert(isequal(ComputePanelSunVisibility(dRotation*[0;0;1], dRotation*dNormals, ...
    dRotatedSamples, dRotatedVertices, 1e-8), dVisible));
for dLengthScale = [1e-3, 1e3]
    assert(isequal(ComputePanelSunVisibility([0;0;1], dNormals, ...
        dLengthScale*dSamples, dLengthScale*dVertices, dLengthScale*1e-8), dVisible));
end

% Reject back-facing illumination and blockers behind the ray origin.
assert(all(ComputePanelSunVisibility([0;0;-1], dNormals, ...
    dSamples, dVertices, 1e-8) == 0));
assert(all(ComputePanelSunVisibility([1;0;0], dNormals, ...
    dSamples, dVertices, 1e-8) == 0), 'Grazing sunlight must leave these faces inactive.');
dVertices(:, :, 2) = dLowerTriangle + [0; 0; -1];
dSamples = cat(3, dLowerTriangle*dBarycentric, dVertices(:, :, 2)*dBarycentric);
assert(isequal(ComputePanelSunVisibility([0;0;1], dNormals, ...
    dSamples, dVertices, 1e-8), [1;0]));

% Leave disjoint forward triangles visible when their rays miss the other face.
dVertices(:, :, 2) = dLowerTriangle + [2; 0; 1];
dSamples = cat(3, dLowerTriangle*dBarycentric, dVertices(:, :, 2)*dBarycentric);
assert(isequal(ComputePanelSunVisibility([0;0;1], dNormals, ...
    dSamples, dVertices, 1e-8), [1;1]));

% Apply both the origin shift and the hit-distance tolerance to nearby occluders.
dRayOffset = 1e-4;
for dSeparation = [0, 1.5*dRayOffset, 3*dRayOffset]
    dVertices(:, :, 2) = dLowerTriangle + [0; 0; dSeparation];
    dSamples = cat(3, dLowerTriangle*dBarycentric, dVertices(:, :, 2)*dBarycentric);
    dExpected = [double(dSeparation <= 2*dRayOffset); 1];
    assert(isequal(ComputePanelSunVisibility([0;0;1], dNormals, ...
        dSamples, dVertices, dRayOffset), dExpected));
end

% Reject undefined directions and mismatched geometry before casting rays.
ExpectFailure_(@() ComputePanelSunVisibility(zeros(3, 1), dNormals, ...
    dSamples, dVertices, 1e-8), 'ComputePanelSunVisibility:ZeroSunDirection');
ExpectFailure_(@() ComputePanelSunVisibility([0;0;1], dNormals, ...
    dSamples(:, :, 1), dVertices, 1e-8), 'ComputePanelSunVisibility:GeometrySizeMismatch');
ExpectFailure_(@() ComputePanelSunVisibility([0;0;1], dNormals, ...
    dSamples, dVertices(:, :, 1), 1e-8), 'ComputePanelSunVisibility:GeometrySizeMismatch');
ExpectFailure_(@() ComputePanelSunVisibility([0;0;1], dNormals, ...
    zeros(3, 0, 2), dVertices, 1e-8), 'ComputePanelSunVisibility:GeometrySizeMismatch');
% Compare batched force and torque with the established per-direction panel law.
dVertices = cat(3, dLowerTriangle, dLowerTriangle + [2; 0; 1]);
dSamples  = cat(3, dLowerTriangle * dBarycentric, dVertices(:,:,2) * dBarycentric);
strPanel  = struct('dSCquadsArea', [0.5; 0.5], ...
                  'dDiffSpecQuadsCoeffs', [0.2, 0.3; 0.1, 0.6], ...
                  'dQuadsNormals_SCB', dNormals, ...
                  'dQuadsPressCentre_SCB', reshape(mean(dVertices,2),3,[]), ...
                  'dVerticesPos', reshape(dVertices,3,[]).', ...
                  'ui32FaceVertexIds', reshape(uint32(1:6),3,[]).', ...
                  'strShadowData', struct('dSamplePoints_SCB', dSamples, ...
                                         'dRayOffset', 1e-8, ...
                                         'strRayData', BuildTriangleRayData(dVertices,false)));
dDirections = [0, 0.25, -0.25; 0, 0.5, 0.5; 1, 1, 1];
[dForce, dJacobian, dTorque] = ComputePanelSrpResponse(dDirections, strPanel, true);
assert(isequal(dForce, ComputePanelSrpResponse(dDirections, strPanel, true)));

for ui32Direction = uint32(1):uint32(size(dDirections,2))
    dDirection = dDirections(:,ui32Direction) / norm(dDirections(:,ui32Direction));
    dVisible   = ComputePanelSunVisibility(dDirection, dNormals, dSamples, dVertices, 1e-8);
    [dOracleForce, dOracleTorque] = ComputeQuadsModelSRP( ...
        dDirection, [1;0;0;0], 1, zeros(3,1), 1, strPanel.dSCquadsArea .* dVisible, ...
        strPanel.dDiffSpecQuadsCoeffs, dNormals, strPanel.dQuadsPressCentre_SCB);
    assert(norm(dForce(:,ui32Direction) - dOracleForce) < 1e-14);
    assert(norm(dTorque(:,ui32Direction) - dOracleTorque) < 1e-14);
    assert(all(isfinite(dJacobian(:,:,ui32Direction)), 'all'));
end

% A 15-node grid exercises a partially filled four-direction evaluator batch.
strSingleLut = BuildSrpResponseLut(strPanel, 1, 90, ui32BatchCount=uint32(1));
strBatchLut  = BuildSrpResponseLut(strPanel, 1, 90, ui32BatchCount=uint32(4));
assert(isequal(strSingleLut.dForcePerPressure, strBatchLut.dForcePerPressure));
assert(isequal(strSingleLut.dEffectiveCr, strBatchLut.dEffectiveCr));
assert(isequal(strSingleLut.dTransverseForcePerPressure, strBatchLut.dTransverseForcePerPressure));

fprintf('Panel visibility, batched response and LUT tail fixtures passed.\n');
end

function ExpectFailure_(fcnCall, charErrorId)
%% SIGNATURE
% ExpectFailure_(fcnCall, charErrorId)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Require the identified boundary failure from the tested public function.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnCall      Public function invocation.
% charErrorId  Expected error identifier.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; assert if the call succeeds or raises another identifier.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Document boundary-failure checks.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    fcnCall (1, 1) function_handle
    charErrorId (1, :) char
end

try
    fcnCall();
catch objError
    assert(strcmp(objError.identifier, charErrorId), ...
        'Expected %s, received %s.', charErrorId, objError.identifier);
    return
end

error('testComputePanelSunVisibility:MissingError', 'Expected %s.', charErrorId);
end
