function [dPotentialPert, dAccPertTB] = EvalPolyhedronGravPerturbationSamples( ...
    dSamplePos_TB, ...
    ui32FaceVertexIds, ...
    dVerticesPos, ...
    dDensity, ...
    ui32EdgeVertexIds, ...
    dEdgeDyadics, ...
    dFaceDyadics, ...
    dGravConst, ...
    dGravParam)%#codegen
arguments
    dSamplePos_TB       (3,:) double {mustBeFinite, mustBeReal}
    ui32FaceVertexIds   (:,3) uint32
    dVerticesPos        (:,3) double {mustBeFinite, mustBeReal}
    dDensity            (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    ui32EdgeVertexIds   (:,2) uint32
    dEdgeDyadics        (3,3,:) double {mustBeFinite, mustBeReal}
    dFaceDyadics        (3,3,:) double {mustBeFinite, mustBeReal}
    dGravConst          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dPotentialPert, dAccPertTB] = EvalPolyhedronGravPerturbationSamples( ...
%     dSamplePos_TB, ui32FaceVertexIds, dVerticesPos, dDensity, ...
%     ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst, dGravParam)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluates the perturbative polyhedron gravity field at multiple sample
% positions. The returned field excludes the central point-mass term.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSamplePos_TB:       [3 x N]         Sample positions in target-body frame.
% ui32FaceVertexIds:   [nFaces x 3]    Triangle vertex indices.
% dVerticesPos:        [nVertices x 3] Vertex coordinates.
% dDensity:            [1]             Bulk density.
% ui32EdgeVertexIds:   [nEdges x 2]    Precomputed edge vertex pairs.
% dEdgeDyadics:        [3 x 3 x Ne]    Precomputed edge dyadics.
% dFaceDyadics:        [3 x 3 x Nf]    Precomputed face dyadics.
% dGravConst:          [1]             Gravitational constant.
% dGravParam:          [1]             Central gravitational parameter.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:      [N x 1]         Perturbative potential samples.
% dAccPertTB:          [3 x N]         Perturbative acceleration samples.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Promote helper from SH fitter to shared utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalPolyhedronGrav()
% -------------------------------------------------------------------------------------------------------------

ui32NumSamples = uint32(size(dSamplePos_TB, 2));
dPotentialPert = zeros(double(ui32NumSamples), 1);
dAccPertTB = zeros(3, double(ui32NumSamples));

for idSample = 1:double(ui32NumSamples)

    % Extract sample position
    dPos = dSamplePos_TB(:, idSample);

    % Evaluate total polyhedron gravity field at the sample position
    [dAccTotal, ~, dPotentialTotal] = EvalPolyhedronGrav(dPos, ui32FaceVertexIds, dVerticesPos, dDensity, ...
    ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst);

    % Subtract central point-mass term to get perturbation
    dRadius = norm(dPos);
    dPotentialPert(idSample) = dPotentialTotal - dGravParam / dRadius;
    dAccPertTB(:, idSample) = dAccTotal + dGravParam * dPos / dRadius^3;

end

end
