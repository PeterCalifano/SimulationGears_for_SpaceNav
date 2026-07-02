function dAccPolyhedronPert_IN = ComputePolyhedronGravityCorrection(dPosSC_IN, ...
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
% dAccPolyhedronPert_IN = ComputePolyhedronGravityCorrection(dPosSC_IN, dDCMmainAtt_INfromTF, dMainGM, strPoly)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute the inertial-frame polyhedron gravity correction used by max-fidelity dynamics.
% EvalPolyhedronGrav returns the total gravity acceleration of the target shape. The max-fidelity RHS already
% carries central gravity through the shared orbit dynamics kernel, so this helper returns:
%   a_correction = a_polyhedron_total - a_central
% expressed back in the inertial frame.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN:             (3,1) double   Spacecraft target-relative inertial position [LU].
% dDCMmainAtt_INfromTF:  (3,3) double   Direction-cosine matrix rotating target-fixed vectors to inertial frame.
% dMainGM:               (1,1) double   Target gravitational parameter [LU^3/TU^2].
% strPoly:               (1,1) struct   Polyhedron gravity data consumed by EvalPolyhedronGrav().
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccPolyhedronPert_IN: (3,1) double   Polyhedron-minus-central acceleration correction [LU/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract polyhedron gravity correction from max-fidelity RHS.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalPolyhedronGrav()
% -------------------------------------------------------------------------------------------------------------

%% Function code
dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;

[dAccPolyhedronTotal_TB, ~] = EvalPolyhedronGrav(dPosSC_TB, ...
                                                 strPoly.ui32FaceVertexIds, ...
                                                 strPoly.dVerticesPos, ...
                                                 strPoly.dDensity, ...
                                                 strPoly.ui32EdgeVertexIds, ...
                                                 strPoly.dEdgeDyadics, ...
                                                 strPoly.dFaceDyadics, ...
                                                 strPoly.dGravConst);

dRadius = norm(dPosSC_TB);
dAccCentral_TB = -dMainGM * dPosSC_TB / dRadius^3;
dAccPolyhedronPert_IN = dDCMmainAtt_INfromTF * (dAccPolyhedronTotal_TB - dAccCentral_TB);

end
