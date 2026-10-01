function [dSRPaccel_IN, dJacAccSRP_IN, dJacAccSRPatt_IN, dJacAccSRPmass_IN, bDerivativeRegular] = ...
    ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB, dMassSC, dSolarPressure, ...
    strResponseLut, bIncludeTransverse, bIncludeSRPressureJacobian, dJacDCMWrtPos_INfromSCB) %#codegen
%% SIGNATURE
% [dSRPaccel_IN, dJacAccSRP_IN, dJacAccSRPatt_IN, dJacAccSRPmass_IN, bDerivativeRegular] = ...
%     ComputeSrpLutAcceleration(dPosSCtoSun_IN, dDCM_INfromSCB, dMassSC, dSolarPressure, ...
%     strResponseLut, bIncludeTransverse, bIncludeSRPressureJacobian, dJacDCMWrtPos_INfromSCB)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Rotate and scale the spacecraft LUT response and its analytical partials.
% Treat the supplied pressure as the current value; optionally include its
% inverse-square position derivative. Hold attitude fixed unless the caller
% supplies dR/dr. Define attitude partials by right-multiplicative body-frame
% rotation error: R(delta) = R exp(skew(delta)). Gate target eclipse in the
% calling dynamics model. Retain spacecraft self-shadowing through the prepared
% LUT. Use metre/SI inputs throughout.
% Use IN for the inertial frame and SCB for the spacecraft body frame. Query
% the LUT with the SCB Sun displacement and interpret its response as force
% per unit solar pressure, not acceleration.
% Prune Jacobian work by the caller's output count: acceleration only, then
% position, attitude, mass and derivative regularity in that order.
% Example: dSRPaccel_IN = ComputeSrpLutAcceleration([1e11;2e10;3e10], ...
%     eye(3), 12, 4e-6, strResponseLut, true, true);
% Output: Inertial acceleration [m/s^2] and optional analytical partials.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_IN            Spacecraft-to-Sun displacement in IN [m].
% dDCM_INfromSCB            Proper rotation from SCB to IN [-].
% dMassSC                   Positive spacecraft mass [kg].
% dSolarPressure            Current solar pressure [N/m^2].
% strResponseLut            Prepared immutable numeric LUT.
% bIncludeTransverse        Compile-time transverse selection; default false.
% bIncludeSRPressureJacobian Include current pressure's range partial; default false.
% dJacDCMWrtPos_INfromSCB   d(DCM_INfromSCB)/dr_j for SC position in IN [1/m].
%                           Three 3-by-3 slices; default zero.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSRPaccel_IN              SRP acceleration in IN [m/s^2].
% dJacAccSRP_IN             IN acceleration/IN spacecraft-position partial [1/s^2].
% dJacAccSRPatt_IN          IN acceleration/SCB rotation-error partial [m/s^2/rad].
% dJacAccSRPmass_IN         IN acceleration/mass partial [m/s^2/kg].
% bDerivativeRegular        LUT derivative regularity; false at knots/poles.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Add shared SI acceleration and derivative handoff.
% 29-09-2026  Pietro Califano, Codex gpt-6  Skip unrequested physical partials.
% 01-10-2026  Pietro Califano, Codex gpt-6  Reuse the MathCore skew matrix implementation.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify vector frames and force-response partials.
% 01-10-2026  Pietro Califano, Codex gpt-6  Remove unused partial-eclipse inputs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvaluateSrpResponseLut, EvalJac_SrpResponseLut, skewSymm (MathCore).
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosSCtoSun_IN (3, 1) double {mustBeFinite}
    dDCM_INfromSCB (3, 3) double {mustBeFinite}
    dMassSC (1, 1) double {mustBeFinite, mustBePositive}
    dSolarPressure (1, 1) double {mustBeFinite, mustBeNonnegative}
    strResponseLut (1, 1) struct
    bIncludeTransverse (1, 1) logical {coder.mustBeConst} = false
    bIncludeSRPressureJacobian (1, 1) logical = false
    dJacDCMWrtPos_INfromSCB (3, 3, 3) double {mustBeFinite} = zeros(3, 3, 3)
end

arguments (Output)
    dSRPaccel_IN (3, 1) double
    dJacAccSRP_IN (3, 3) double
    dJacAccSRPatt_IN (3, 3) double
    dJacAccSRPmass_IN (3, 1) double
    bDerivativeRegular (1, 1) logical
end

% Return exact inactive force and partials without querying the table.
dSRPaccel_IN = zeros(3, 1);
dJacAccSRP_IN = zeros(3, 3);
dJacAccSRPatt_IN = zeros(3, 3);
dJacAccSRPmass_IN = zeros(3, 1);
bDerivativeRegular = true;

if dSolarPressure == 0
    return
end

% Query force per pressure with the unnormalized Sun displacement in SCB.
dPosSCtoSun_SCB = transpose(dDCM_INfromSCB) * dPosSCtoSun_IN;

if nargout < 2
    dForcePerPressure_SCB = EvaluateSrpResponseLut(dPosSCtoSun_SCB, ...
                                                strResponseLut, bIncludeTransverse);
elseif nargout < 5
    [dJacForcePerPressWrtSunPos_SCB, ~, ~, dForcePerPressure_SCB] = ...
        EvalJac_SrpResponseLut(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse);
else
    [dJacForcePerPressWrtSunPos_SCB, ~, ~, dForcePerPressure_SCB, ~, ~, bDerivativeRegular] = ...
        EvalJac_SrpResponseLut(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse);
end

% Rotate force per pressure to IN and apply current pressure over mass.
dAccelScaleSRP = dSolarPressure / dMassSC;
dForcePerPressure_IN = dDCM_INfromSCB * dForcePerPressure_SCB;
dSRPaccel_IN = dAccelScaleSRP * dForcePerPressure_IN;

if nargout < 2
    return
end

% Differentiate the SCB Sun displacement and rotated force per pressure w.r.t. IN
% spacecraft position. Include attitude-position dependence in both rotations;
% dJacForcePerPressWrtSunPos_SCB differentiates w.r.t. the SCB Sun displacement.
dJacSunPosWrtScPos_SCB = - transpose(dDCM_INfromSCB);
dJacForcePerPressRotWrtPos_IN = zeros(3, 3);

for ui32PosAxis = uint32(1):uint32(3)
    dJacDCMWrtPosAxis_INfromSCB = dJacDCMWrtPos_INfromSCB(:, :, ui32PosAxis);

    dJacSunPosWrtScPos_SCB(:, ui32PosAxis) = dJacSunPosWrtScPos_SCB(:, ui32PosAxis) + ...
        transpose(dJacDCMWrtPosAxis_INfromSCB) * dPosSCtoSun_IN;

    dJacForcePerPressRotWrtPos_IN(:, ui32PosAxis) = ...
        dJacDCMWrtPosAxis_INfromSCB * dForcePerPressure_SCB;
end

dJacAccSRP_IN = dAccelScaleSRP * (dJacForcePerPressRotWrtPos_IN + ...
    dDCM_INfromSCB * dJacForcePerPressWrtSunPos_SCB * dJacSunPosWrtScPos_SCB);

% Differentiate the current solar pressure with respect to IN spacecraft position.
dJacSolarPressureWrtPos_IN = zeros(1, 3);

if bIncludeSRPressureJacobian
    dJacSolarPressureWrtPos_IN = 2 * dSolarPressure * dPosSCtoSun_IN.' / ...
        dot(dPosSCtoSun_IN, dPosSCtoSun_IN);
end

dJacAccSRP_IN = dJacAccSRP_IN + ...
    dForcePerPressure_IN * dJacSolarPressureWrtPos_IN / dMassSC;

% Specialize optional attitude and mass partials to the requested output prefix.
if nargout > 2
    dForcePerPressureSkew_SCB = skewSymm(dForcePerPressure_SCB);
    dPosSCtoSunSkew_SCB = skewSymm(dPosSCtoSun_SCB);
    dJacAccSRPatt_IN = dAccelScaleSRP * dDCM_INfromSCB * ...
        (- dForcePerPressureSkew_SCB + dJacForcePerPressWrtSunPos_SCB * dPosSCtoSunSkew_SCB);
end

if nargout > 3
    dJacAccSRPmass_IN = - dSRPaccel_IN / dMassSC;
end
end
