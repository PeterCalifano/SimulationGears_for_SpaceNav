function [dDxDtCannonball_IN, dDxDtLut_IN, strCannonballInfo, strLutInfo] = ...
    CodegenOrbitalSrpTypesProbe(dxState_IN, dPosSun_IN, bIsInEclipse, ...
                               strSrpData, strResponseLut, bIncludeTransverse) %#codegen
%% SIGNATURE
% [dDxDtCannonball_IN, dDxDtLut_IN, strCannonballInfo, strLutInfo] = CodegenOrbitalSrpTypesProbe(dxState_IN, ...
%     dPosSun_IN, bIsInEclipse, strSrpData, strResponseLut, bIncludeTransverse)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compile cannonball and LUT orbital calls with shared diagnostics in one generated module.
% Exercise the unused empty LUT input alongside the named numeric payload;
% keep the shared physical and diagnostic types consistent across both calls.
% Use this test-only entry point through testSrpResponseLutCodegen.
% Example: [dCannonball, dLut] = CodegenOrbitalSrpTypesProbe(dxState_IN, ...
%     dPosSun_IN, false, strSrpData, strResponseLut, bIncludeTransverse);
% Output: Two six-element derivatives and matching SRP diagnostic records.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dxState_IN          Cartesian orbital state in IN [m; m/s].
% dPosSun_IN          Main-body-to-Sun position in IN [m].
% bIsInEclipse        Suppress both SRP models when true.
% strSrpData          Resolved SI pressure, mass, bias and spacecraft pointing.
% strResponseLut      Immutable numeric SRP payload.
% bIncludeTransverse  Compile-time transverse selection.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDxDtCannonball_IN   Cannonball orbital derivatives [m/s; m/s^2].
% dDxDtLut_IN          LUT orbital derivatives [m/s; m/s^2].
% strCannonballInfo    Cannonball force components, including selected dAccSRP [m/s^2].
% strLutInfo           LUT force components with the same diagnostic layout.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Return shared selected-SRP diagnostics.
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 01-10-2026  Pietro Califano, Codex gpt-6  Check legacy and LUT types in one build.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalRHS_InertialDynOrbit.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dxState_IN (6, 1) double
    dPosSun_IN (3, 1) double
    bIsInEclipse (1, 1) logical
    strSrpData (1, 1) struct
    strResponseLut (1, 1) struct {coder.mustBeConst}
    bIncludeTransverse (1, 1) logical {coder.mustBeConst}
end

arguments (Output)
    dDxDtCannonball_IN (6, 1) double
    dDxDtLut_IN (6, 1) double
    strCannonballInfo (1, 1) struct
    strLutInfo (1, 1) struct
end

% Name payloads before calls into non-entry-point functions.
coder.cstructname(strResponseLut, 'SSrpResponseLut');
coder.cstructname(strSrpData, 'SSrpData');
coder.cstructname(strSrpData.strPointing, 'SSrpPointing');

% Exercise the default empty LUT inputs and retain the selected-force diagnostics.
[dDxDtCannonball_IN, strCannonballInfo] = EvalRHS_InertialDynOrbit(dxState_IN, eye(3), 0, 1, 2e-8, 0, ...
    dPosSun_IN, [], uint32(0), uint16([1, 6]), zeros(3, 1), bIsInEclipse);

% Share the numeric LUT and physical types within the same generated module.
[dDxDtLut_IN, strLutInfo] = EvalRHS_InertialDynOrbit(dxState_IN, eye(3), 0, 1, 0, 0, ...
    dPosSun_IN, [], uint32(0), uint16([1, 6]), zeros(3, 1), bIsInEclipse, ...
    true, strResponseLut, strSrpData, bIncludeTransverse);
end
