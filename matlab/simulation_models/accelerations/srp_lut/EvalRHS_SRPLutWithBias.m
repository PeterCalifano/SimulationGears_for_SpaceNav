function [dSRPaccel_IN, dJacAccSRP_IN, dJacAccSRPWrtBias_IN, bDerivativeRegular] = ...
    EvalRHS_SRPLutWithBias(dPosSCtoSun_IN, strSrpData, strResponseLut, bIncludeTransverse) %#codegen
%% SIGNATURE
% [dSRPaccel_IN, dJacAccSRP_IN, dJacAccSRPWrtBias_IN, bDerivativeRegular] = ...
%     EvalRHS_SRPLutWithBias(dPosSCtoSun_IN, strSrpData, strResponseLut, bIncludeTransverse)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate LUT SRP and an additive acceleration bias in metre/kilometre units.
% Resolve pressure from the supplied reference at 1 AU and Sun displacement.
% Reuse the shared SI force model for interpolation and physical partials.
% Add bias along Sun-to-spacecraft, independent of pressure and transverse
% response. Include Sun-line rotation in bias position partials. Supply zero
% nominal bias for a considered parameter; retain its sensitivity whenever
% requested. Leave eclipse and ephemeris validity to the caller. Return zeros
% for zero pressure or range.
% Example: dAcceleration = EvalRHS_SRPLutWithBias(dPosSCtoSun_IN, strSrpData, strLut, true);
% Output: Inertial SRP acceleration in the selected length unit per second squared.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_IN         Inertial spacecraft-to-Sun displacement [LU].
% strSrpData             Resolved numerical spacecraft data: dReferencePressure at 1 AU
%                        [kg/(LU*s^2)], dMass [kg], dBiasAcceleration [LU/s^2],
%                        bUseKilometersScale and strPointing.
%                        Supply body-to-inertial dDCM_INfromSCB and dJacDCMWrtPos_INfromSCB
%                        [1/LU]. Gate target eclipse before calling this function.
% strResponseLut         Immutable numeric table; geometry/area use metres/square metres.
% bIncludeTransverse     Compile-time selection of transverse support.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSRPaccel_IN           Inertial SRP acceleration [LU/s^2].
% dJacAccSRP_IN          Acceleration/spacecraft-position partial [1/s^2].
% dJacAccSRPWrtBias_IN   Additive acceleration-bias sensitivity [-].
% bDerivativeRegular     False at knots/seam or adjusted pole lookups; true when inactive.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 30-09-2026  Pietro Califano, Codex gpt-6  Own generic LUT/bias physics outside filter state mapping.
% 01-10-2026  Pietro Califano, Codex gpt-6  Standardize SRP acronym in entry-point names.
% 01-10-2026  Pietro Califano, Codex gpt-6  Remove partial-eclipse scaling and gradient inputs.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify frames, physical inputs and generated struct types.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputeSolarRadPressure, ComputeSrpLutAcceleration.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosSCtoSun_IN (3, 1) double
    strSrpData (1, 1) struct
    strResponseLut (1, 1) struct {coder.mustBeConst}
    bIncludeTransverse (1, 1) logical {coder.mustBeConst}
end

arguments (Output)
    dSRPaccel_IN (3, 1) double
    dJacAccSRP_IN (3, 3) double
    dJacAccSRPWrtBias_IN (3, 1) double
    bDerivativeRegular (1, 1) logical
end

% Preserve fixed outputs while bypassing unavailable geometry or radiation.
dSRPaccel_IN = zeros(3, 1);
dJacAccSRP_IN = zeros(3, 3);
dJacAccSRPWrtBias_IN = zeros(3, 1);
bDerivativeRegular = true;

dSunRange = norm(dPosSCtoSun_IN);
if ~isfinite(dSunRange) || dSunRange <= eps('single')
    return
end

% Pass a fixed unit selector to the pressure helper in each runtime unit branch.
dLengthScale = 1;
if strSrpData.bUseKilometersScale
    dLengthScale = 1000;
    dSolarPressure = ComputeSolarRadPressure(1 / dSunRange, true, strSrpData.dReferencePressure);
else
    dSolarPressure = ComputeSolarRadPressure(1 / dSunRange, false, strSrpData.dReferencePressure);
end

if dSolarPressure == 0
    return
end

% Convert the resolved physical inputs once for the shared SI evaluator.
strPointing = strSrpData.strPointing;
dBiasAcceleration = strSrpData.dBiasAcceleration;
assert(isfinite(dBiasAcceleration), ...
    'EvalRHS_SRPLutWithBias:InvalidBias', 'Supply a finite additive acceleration bias.');

% Evaluate only the physical outputs requested by this caller.
% Specialize the output prefix at code-generation time.
if nargout < 2
    dSRPaccel_IN = ComputeSrpLutAcceleration(dPosSCtoSun_IN * dLengthScale, ...
                                            strPointing.dDCM_INfromSCB, strSrpData.dMass, ...
                                            dSolarPressure / dLengthScale, strResponseLut, ...
                                            bIncludeTransverse, true);
elseif nargout < 4
    [dSRPaccel_IN, dJacAccSRP_IN] = ...
        ComputeSrpLutAcceleration(dPosSCtoSun_IN * dLengthScale, strPointing.dDCM_INfromSCB, ...
                                 strSrpData.dMass, dSolarPressure / dLengthScale, strResponseLut, ...
                                 bIncludeTransverse, true, ...
                                 strPointing.dJacDCMWrtPos_INfromSCB / dLengthScale);
else
    [dSRPaccel_IN, dJacAccSRP_IN, ~, ~, bDerivativeRegular] = ...
        ComputeSrpLutAcceleration(dPosSCtoSun_IN * dLengthScale, strPointing.dDCM_INfromSCB, ...
                                 strSrpData.dMass, dSolarPressure / dLengthScale, strResponseLut, ...
                                 bIncludeTransverse, true, ...
                                 strPointing.dJacDCMWrtPos_INfromSCB / dLengthScale);
end

dSRPaccel_IN = dSRPaccel_IN / dLengthScale;

% Add nominal bias and retain its sensitivity without repeating LUT evaluation.
if nargout > 2 || dBiasAcceleration ~= 0
    dSunDir_IN = dPosSCtoSun_IN / dSunRange;
    dJacAccSRPWrtBias_IN = -dSunDir_IN;
    dSRPaccel_IN = dSRPaccel_IN + dBiasAcceleration * dJacAccSRPWrtBias_IN;

    % Account for the bias direction changing with spacecraft position.
    if nargout > 1 && dBiasAcceleration ~= 0
        dJacAccSRP_IN = dJacAccSRP_IN + ...
            dBiasAcceleration * (eye(3) - dSunDir_IN * dSunDir_IN.') / dSunRange;
    end
end
end
