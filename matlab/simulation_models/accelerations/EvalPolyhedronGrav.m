function [dAccGrav, dJacAccGrav, dGravPotential, dLaplacian] = EvalPolyhedronGrav( ...
    dFieldPoint_BF, ...
    ui32FaceVertexIds, ...
    dVerticesPos, ...
    dDensity, ...
    ui32EdgeVertexIds, ...
    dEdgeDyadics, ...
    dFaceDyadics, ...
    dGravConst) %#codegen
arguments
    dFieldPoint_BF    (3,1) double
    ui32FaceVertexIds (:,3) uint32
    dVerticesPos      (:,3) double
    dDensity          (1,1) double
    ui32EdgeVertexIds (:,2) uint32
    dEdgeDyadics      (3,3,:) double
    dFaceDyadics      (3,3,:) double
    dGravConst        (1,1) double = 6.67430e-11
end
%% PROTOTYPE
% [dAccGrav, dJacAccGrav, dGravPotential, dLaplacian] = EvalPolyhedronGrav(
%     dFieldPoint_BF, ui32FaceVertexIds, dVerticesPos, dDensity,
%     ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics, dGravConst)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes the exact gravitational acceleration, its 3x3 Jacobian,
% gravitational potential, and Laplacian due to a constant-density
% polyhedron at a given field point in the body-fixed frame.
% The acceleration is the gradient of the potential (DU), the Jacobian is
% the Hessian of the potential (DDU). Both edge and face contributions are
% computed in a single pass over the mesh to avoid redundant work.
%
% The function returns the acceleration and Jacobian together because they
% share all intermediate quantities (edge wire potentials Le, face solid
% angles omegaf, relative position vectors). Splitting would require
% recomputing two full mesh loops.
%
% ACHTUNG: All inputs must be in consistent units (e.g., all in SI: meters,
% kg/m^3, m^3/(kg*s^2)). The output units follow from the input units.
%
% REFERENCE:
% 1) Werner, R.A. and Scheeres, D.J. (1997), Exterior Gravitation of a
%    Polyhedron Derived and Compared with Harmonic and Mascon Gravitation
%    Representations of Asteroid 4769 Castalia. Celestial Mechanics and
%    Dynamical Astronomy, 65, 313-344.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dFieldPoint_BF:    [3x1, LU]              Field point position in body-fixed frame
% ui32FaceVertexIds: [nFaces x 3, uint32]   Triangle vertex indices (1-based)
% dVerticesPos:      [nVertices x 3, LU]    Vertex coordinates in body-fixed frame
% dDensity:          [1x1, MU/LU^3]         Constant bulk density of the body
% ui32EdgeVertexIds: [nEdges x 2, uint32]   Edge vertex pairs (from ComputePolyhedronFaceEdgeData)
% dEdgeDyadics:      [3x3xnEdges, 1/LU^2]  Edge dyadic tensors Ee (from ComputePolyhedronFaceEdgeData)
% dFaceDyadics:      [3x3xnFaces, -]        Face dyadic tensors Ff (from ComputePolyhedronFaceEdgeData)
% dGravConst:        [1x1]                  Gravitational constant G [LU^3/(MU*TU^2)]
%                                           Default: 6.67430e-11 m^3/(kg*s^2)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccGrav:        [3x1, LU/TU^2]      Gravitational acceleration (gradient of potential)
% dJacAccGrav:     [3x3, 1/TU^2]       Jacobian of acceleration (Hessian of potential)
% dGravPotential:  [1x1, LU^2/TU^2]    Gravitational potential
% dLaplacian:      [1x1, 1/TU^2]       Laplacian of the potential
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-03-2026    Pietro Califano    Codegen-ready implementation for SimulationGears,
%                                  ported and optimized from simulationUtils
%                                  (original: Fabio Ferrari v1.0, Carmine Buonagura v2.0,
%                                  Lucia Civati v2.1).
% 29-03-2026    Pietro Califano    Vectorized edge and face loops using
%                                  pagemtimes and batch operations. Eliminates
%                                  per-element for-loops for acceleration,
%                                  Jacobian, potential and Laplacian.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePolyhedronFaceEdgeData (for preprocessing inputs)
% -------------------------------------------------------------------------------------------------------------

dGRho = dGravConst * dDensity;

%% Edge contributions (vectorized over all edges)
% Gather vertex positions for all edges: [3 x nEdges]
dEdgeVerticesI = dVerticesPos(ui32EdgeVertexIds(:, 1), :)'; % [3 x nEdges]
dEdgeVerticesJ = dVerticesPos(ui32EdgeVertexIds(:, 2), :)'; % [3 x nEdges]

% Relative position vectors from field point to edge vertices
dEdgeRelPosI = dEdgeVerticesI - dFieldPoint_BF; % [3 x nEdges], broadcast
dEdgeRelPosJ = dEdgeVerticesJ - dFieldPoint_BF;

% Distances to vertices and edge lengths
dEdgeDistI = sqrt(sum(dEdgeRelPosI.^2, 1)); % [1 x nEdges]
dEdgeDistJ = sqrt(sum(dEdgeRelPosJ.^2, 1));
dEdgeLengths = sqrt(sum((dEdgeVerticesJ - dEdgeVerticesI).^2, 1));

% Wire potential (logarithmic term) for all edges: [1 x nEdges]
dWirePotentialLe = log((dEdgeDistI + dEdgeDistJ + dEdgeLengths) ./ ...
    (dEdgeDistI + dEdgeDistJ - dEdgeLengths));

% Batched matrix-vector product: Ee * relPosI for each edge
% dEdgeDyadics is [3 x 3 x nEdges], dEdgeRelPosI reshaped to [3 x 1 x nEdges]
dEeTimesRelPos = pagemtimes(dEdgeDyadics, reshape(dEdgeRelPosI, 3, 1, [])); % [3 x 1 x nEdges]
dEeTimesRelPos = reshape(dEeTimesRelPos, 3, []); % [3 x nEdges]

% Potential: sum_e (relPosI' * Ee * relPosI) * Le
dEdgeQuadForm = sum(dEdgeRelPosI .* dEeTimesRelPos, 1); % [1 x nEdges]
dPotentialEdges = sum(dEdgeQuadForm .* dWirePotentialLe);

% Gradient: sum_e (Ee * relPosI) * Le
dGradientEdges = dEeTimesRelPos * dWirePotentialLe'; % [3 x nEdges] * [nEdges x 1] = [3 x 1]

% Hessian: sum_e Ee * Le
dHessianEdges = sum(dEdgeDyadics .* reshape(dWirePotentialLe, 1, 1, []), 3); % [3 x 3]

%% Face contributions (vectorized over all faces)
% Gather vertex positions for all faces: [3 x nFaces]
dFaceVerts1 = dVerticesPos(ui32FaceVertexIds(:, 1), :)';
dFaceVerts2 = dVerticesPos(ui32FaceVertexIds(:, 2), :)';
dFaceVerts3 = dVerticesPos(ui32FaceVertexIds(:, 3), :)';

% Relative position vectors from field point to face vertices
dFaceRelPos1 = dFaceVerts1 - dFieldPoint_BF; % [3 x nFaces]
dFaceRelPos2 = dFaceVerts2 - dFieldPoint_BF;
dFaceRelPos3 = dFaceVerts3 - dFieldPoint_BF;

% Distances to vertices
dFaceDist1 = sqrt(sum(dFaceRelPos1.^2, 1)); % [1 x nFaces]
dFaceDist2 = sqrt(sum(dFaceRelPos2.^2, 1));
dFaceDist3 = sqrt(sum(dFaceRelPos3.^2, 1));

% Solid angles via exact formula (vectorized)
dCrossR2R3 = cross(dFaceRelPos2, dFaceRelPos3, 1);    % [3 x nFaces]
dSolidAngleNum = sum(dFaceRelPos1 .* dCrossR2R3, 1);  % [1 x nFaces]

dDotR2R3 = sum(dFaceRelPos2 .* dFaceRelPos3, 1);
dDotR3R1 = sum(dFaceRelPos3 .* dFaceRelPos1, 1);
dDotR1R2 = sum(dFaceRelPos1 .* dFaceRelPos2, 1);

dSolidAngleDen = dFaceDist1 .* dFaceDist2 .* dFaceDist3 ...
    + dFaceDist1 .* dDotR2R3 ...
    + dFaceDist2 .* dDotR3R1 ...
    + dFaceDist3 .* dDotR1R2;

dSolidAngles = 2.0 * atan2(dSolidAngleNum, dSolidAngleDen); % [1 x nFaces]

% Batched matrix-vector product: Ff * relPos1 for each face
dFfTimesRelPos = pagemtimes(dFaceDyadics, reshape(dFaceRelPos1, 3, 1, [])); % [3 x 1 x nFaces]
dFfTimesRelPos = reshape(dFfTimesRelPos, 3, []); % [3 x nFaces]

% Potential: sum_f (relPos1' * Ff * relPos1) * omega_f
dFaceQuadForm = sum(dFaceRelPos1 .* dFfTimesRelPos, 1); % [1 x nFaces]
dPotentialFaces = sum(dFaceQuadForm .* dSolidAngles);

% Gradient: sum_f (Ff * relPos1) * omega_f
dGradientFaces = dFfTimesRelPos * dSolidAngles'; % [3 x 1]

% Hessian: sum_f Ff * omega_f
dHessianFaces = sum(dFaceDyadics .* reshape(dSolidAngles, 1, 1, []), 3); % [3 x 3]

% Laplacian: sum_f omega_f
dLaplacianFaces = sum(dSolidAngles);

%% Assemble outputs: combine edge and face contributions
% Potential: U = 0.5 * G * rho * (U_edges - U_faces)
dGravPotential = 0.5 * dGRho * (dPotentialEdges - dPotentialFaces);

% Acceleration (gradient of potential): DU = G * rho * (-DU_edges + DU_faces)
dAccGrav = dGRho * (-dGradientEdges + dGradientFaces);

% Jacobian (Hessian of potential): DDU = G * rho * (DDU_edges - DDU_faces)
dJacAccGrav = dGRho * (dHessianEdges - dHessianFaces);

% Laplacian: D2U = -G * rho * D2U_faces
dLaplacian = -dGRho * dLaplacianFaces;

end
