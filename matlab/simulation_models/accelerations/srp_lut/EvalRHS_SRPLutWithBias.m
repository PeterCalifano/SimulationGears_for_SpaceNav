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
% Add a signed acceleration bias along the unbiased selected SRP response,
% including transverse force when enabled. Keep its magnitude independent of
% pressure. Differentiate this direction through the same physical position
% partial, including supplied attitude dependence, without evaluating the LUT
% again. Supply zero nominal bias for a considered parameter while retaining
% its sensitivity. Leave eclipse and ephemeris validity to the caller.
% Return zeros for zero pressure or range. Suppress bias and its sensitivity
% for a zero nominal response and mark that direction as derivative-irregular.
% Example: dAcceleration = EvalRHS_SRPLutWithBias(dPosSCtoSun_IN, strSrpData, strLut, true);
% Output: Inertial SRP acceleration in the selected length unit per second squared.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_IN         Inertial spacecraft-to-Sun displacement [LU].
% strSrpData             Resolved numerical spacecraft data: dReferencePressure at 1 AU
%                        [kg/(LU*s^2)], dMass [kg], signed dBiasAcceleration [LU/s^2],
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
% bDerivativeRegular     False at knots/seam, adjusted poles or zero active response;
%                        true when inactive through zero pressure or range.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 04-10-2026  Pietro Califano     Align additive bias and its partials with selected SRP.
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
    strResponseLut (1, 1) struct
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

% Normalize the unbiased force so signed bias never changes its own direction.
% Retain this sensitivity at zero bias for considered-parameter propagation.
if nargout > 2 || dBiasAcceleration ~= 0
    dNominalAccelNorm = norm(dSRPaccel_IN);
    if dNominalAccelNorm == 0
        bDerivativeRegular = false;
        return
    end

    dJacAccSRPWrtBias_IN = dSRPaccel_IN / dNominalAccelNorm;
    dSRPaccel_IN = dSRPaccel_IN + dBiasAcceleration * dJacAccSRPWrtBias_IN;

    % Project the existing physical derivative onto the plane normal to SRP.
    % Include pressure and supplied pointing chains through that same derivative.
    if nargout > 1 && dBiasAcceleration ~= 0
        dJacAccSRP_IN = dJacAccSRP_IN + ...
            (dBiasAcceleration / dNominalAccelNorm) * ...
            (dJacAccSRP_IN - dJacAccSRPWrtBias_IN * (dJacAccSRPWrtBias_IN.' * dJacAccSRP_IN));
    end
end
end
