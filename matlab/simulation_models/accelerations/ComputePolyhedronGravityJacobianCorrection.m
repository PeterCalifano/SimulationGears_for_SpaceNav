function dJacPolyCorrection_IN = ComputePolyhedronGravityJacobianCorrection(dPosSC_IN, ...
                                                                            dDCMmainAtt_INfromTF, ...
                                                                            dMainGM, ...
                                                                            strPoly) %#codegen
arguments
    dPosSC_IN             (3,1) double {mustBeFinite}
    dDCMmainAtt_INfromTF  (3,3) double {mustBeFinite}
    dMainGM               (1,1) double {mustBeFinite, mustBeNonnegative}
    strPoly               (1,1) struct
end
%% PROTOTYPE
% dJacPolyCorrection_IN = ComputePolyhedronGravityJacobianCorrection(dPosSC_IN, dDCMmainAtt_INfromTF, dMainGM, strPoly)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute the inertial-frame position Jacobian of the polyhedron gravity correction.
% EvalPolyhedronGrav returns the total polyhedron gravity partial. Because central gravity is already included by
% the shared max-fidelity Jacobian path, this helper subtracts the central point-mass partial in target-fixed
% coordinates and rotates the correction back to the inertial frame:
%   da_correction/dr = R * (da_polyhedron_total/dr_TB - da_central/dr_TB) * R'
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN:             (3,1) double   Spacecraft target-relative inertial position [LU].
% dDCMmainAtt_INfromTF:  (3,3) double   Direction-cosine matrix rotating target-fixed vectors to inertial frame.
% dMainGM:               (1,1) double   Target gravitational parameter [LU^3/TU^2].
% strPoly:               (1,1) struct   Polyhedron gravity data consumed by EvalPolyhedronGrav().
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacPolyCorrection_IN: (3,3) double   Polyhedron-minus-central acceleration partial [1/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract polyhedron gravity Jacobian correction.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalPolyhedronGrav()
% -------------------------------------------------------------------------------------------------------------

%% Function code
dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;

[~, dJacPolyTotal_TB] = EvalPolyhedronGrav(dPosSC_TB, ...
                                           strPoly.ui32FaceVertexIds, ...
                                           strPoly.dVerticesPos, ...
                                           strPoly.dDensity, ...
                                           strPoly.ui32EdgeVertexIds, ...
                                           strPoly.dEdgeDyadics, ...
                                           strPoly.dFaceDyadics, ...
                                           strPoly.dGravConst);

dJacCentral_TB = EvalPointMassGravityJacobian_(dPosSC_TB, -dMainGM);
dJacPolyCorrection_TB = dJacPolyTotal_TB - dJacCentral_TB;
dJacPolyCorrection_IN = dDCMmainAtt_INfromTF * dJacPolyCorrection_TB * dDCMmainAtt_INfromTF.';

end

function dJac = EvalPointMassGravityJacobian_(dPos, dSignedGM)
% Evaluate point-mass acceleration partial with caller-signed gravitational parameter.
dRadius = norm(dPos);
dInvRadius = 1.0 / dRadius;
dInvRadius3 = dInvRadius^3;
dJac = dSignedGM * (dInvRadius3 * eye(3) - 3.0 * dInvRadius3 * dInvRadius^2 * (dPos * dPos.'));
end
