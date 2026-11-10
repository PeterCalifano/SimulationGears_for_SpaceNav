function tests = testThirdBodyGravity
%% SIGNATURE
% tests = testThirdBodyGravity
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify target-relative third-body gravity against an independent physical
% oracle. Coverage includes the generic-body loop, the separate Sun block,
% arbitrary three-dimensional geometry, and the max-fidelity RHS/Jacobian consumers.
% Compare raw finite differences and preserve unavailable-ephemeris handling.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB function-test suite for third-body differential gravity.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-07-2026  Pietro Califano, Codex     Add physical sign regressions for all SimulationGears RHS paths.
% 15-09-2026  Pietro Califano, Codex     Cover the max-fidelity third-body Jacobian against its RHS.
% 15-09-2026  Pietro Califano, Codex     Match RHS/Jacobian handling of invalid enabled-body ephemerides.
% 11-10-2026  Pietro Califano  Adapt current APIs and guard small Sun tidal acceleration.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalRHS_InertialDynOrbit()
% EvalRHS_InertialDynMaxFidelity()
% -------------------------------------------------------------------------------------------------------------
tests = functiontests(localfunctions);
end

function setupOnce(~)
charTestDirectory = fileparts(mfilename('fullpath'));
charRepositoryRoot = fileparts(fileparts(fileparts(charTestDirectory)));
addpath(genpath(fullfile(charRepositoryRoot, 'matlab')));
end

function testGenericBodyAccelerationMatchesAxialOracle(testCase)
dxState = [1.0; 0.0; 0.0; 0.0; 0.0; 0.0];
dSunPosition_IN = [20.0; 0.0; 0.0];
dBodyPosition_IN = [10.0; 0.0; 0.0];
dBodyEphemerides = [dSunPosition_IN; dBodyPosition_IN];
dThirdBodyGMs = [0.0; 100.0];

dRhs = EvalOrbitRhs_(dxState, dThirdBodyGMs, dBodyEphemerides);
dExpectedAcceleration = ComputePhysicalThirdBodyAcceleration_( ...
    dxState(1:3), dBodyPosition_IN, dThirdBodyGMs(2));

testCase.verifyGreaterThan(dExpectedAcceleration(1), 0.0, ...
    'A body farther along +x must stretch the spacecraft in +x.');
testCase.verifyEqual(dRhs(4:6), dExpectedAcceleration, 'AbsTol', 1.0e-13);
end

function testGenericBodyAccelerationMatchesThreeDimensionalOracle(testCase)
dxState = [1.0; 0.2; -0.1; 0.0; 0.0; 0.0];
dSunPosition_IN = [20.0; 4.0; -3.0];
dBodyPosition_IN = [10.0; -2.0; 1.0];
dBodyEphemerides = [dSunPosition_IN; dBodyPosition_IN];
dThirdBodyGMs = [0.0; 100.0];

dRhs = EvalOrbitRhs_(dxState, dThirdBodyGMs, dBodyEphemerides);
dExpectedAcceleration = ComputePhysicalThirdBodyAcceleration_( ...
    dxState(1:3), dBodyPosition_IN, dThirdBodyGMs(2));

testCase.verifyGreaterThan(norm(dExpectedAcceleration), 0.0);
testCase.verifyEqual(dRhs(4:6), dExpectedAcceleration, 'AbsTol', 1.0e-13);
end

function testSunAccelerationMatchesThreeDimensionalOracle(testCase)
dxState = [1.0; 0.2; -0.1; 0.0; 0.0; 0.0];
dSunPosition_IN = [10.0; -2.0; 1.0];
dDisabledBodyPosition_IN = [20.0; 4.0; -3.0];
dBodyEphemerides = [dSunPosition_IN; dDisabledBodyPosition_IN];
dThirdBodyGMs = [100.0; 0.0];

dRhs = EvalOrbitRhs_(dxState, dThirdBodyGMs, dBodyEphemerides);
dExpectedAcceleration = ComputePhysicalThirdBodyAcceleration_( ...
    dxState(1:3), dSunPosition_IN, dThirdBodyGMs(1));

testCase.verifyGreaterThan(norm(dExpectedAcceleration), 0.0);
testCase.verifyEqual(dRhs(4:6), dExpectedAcceleration, 'AbsTol', 1.0e-13);
end

function testMaxFidelityRhsUsesPhysicalThirdBodyAcceleration(testCase)
dxState = [1.0; 0.2; -0.1; 0.0; 0.0; 0.0];
dEarthPosition_IN = [10.0; -2.0; 1.0];
strDynParams = BuildMaxFidelityDynParams_(dEarthPosition_IN);
strModelConfigFlags = BuildThirdBodyOnlyConfig_();

[dRhs, strAccelInfo] = EvalRHS_InertialDynMaxFidelity( ...
    0.0, dxState, strDynParams, strModelConfigFlags);
dExpectedAcceleration = ComputePhysicalThirdBodyAcceleration_( ...
    dxState(1:3), dEarthPosition_IN, strDynParams.strBody3rdData(2).dGM);

testCase.verifyEqual(dRhs(4:6), dExpectedAcceleration, 'AbsTol', 1.0e-13);
testCase.verifyEqual(strAccelInfo.dTotAcc3rdBody, dExpectedAcceleration, ...
    'AbsTol', 1.0e-13);
end

function testMaxFidelityJacobianMatchesThirdBodyRhs(testCase)
%% SIGNATURE
% testMaxFidelityJacobianMatchesThirdBodyRhs(testCase)
%% DESCRIPTION
% Compare the actual third-body RHS derivative with centred finite differences.
%% INPUT
% testCase   MATLAB assertion context.
%% OUTPUT
% None; verify the public dynamics contract.
%% CHANGELOG
% 11-10-2026  Pietro Califano  Consolidate the isolated gravity regression.
%% DEPENDENCIES
% EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity

dxState = [1.0; 0.2; -0.1; 0.0; 0.0; 0.0];
dEarthPosition_IN = [10.0; -2.0; 1.0];
strDynParams = BuildMaxFidelityDynParams_(dEarthPosition_IN);
strModelConfigFlags = BuildThirdBodyOnlyConfig_();

[~, strAccelInfo] = EvalRHS_InertialDynMaxFidelity( ...
    0.0, dxState, strDynParams, strModelConfigFlags);
dJacobian = EvalJac_InertialDynMaxFidelity( ...
    0.0, dxState, strDynParams, strModelConfigFlags, strAccelInfo);

dStep = 1.0e-5;
dJacobianFD = zeros(3, 3);
for idAxis = 1:3
    dPerturbation = zeros(6, 1);
    dPerturbation(idAxis) = dStep;
    dRhsPlus = EvalRHS_InertialDynMaxFidelity( ...
        0.0, dxState + dPerturbation, strDynParams, strModelConfigFlags);
    dRhsMinus = EvalRHS_InertialDynMaxFidelity( ...
        0.0, dxState - dPerturbation, strDynParams, strModelConfigFlags);
    dJacobianFD(:, idAxis) = (dRhsPlus(4:6) - dRhsMinus(4:6)) / (2.0 * dStep);
end

testCase.verifyEqual(dJacobian(4:6, 1:3), dJacobianFD, ...
    'RelTol', 1.0e-8, 'AbsTol', 1.0e-10);
end

function testMaxFidelityJacobianSkipsInvalidEnabledBody(testCase)
%% SIGNATURE
% testMaxFidelityJacobianSkipsInvalidEnabledBody(testCase)
%% DESCRIPTION
% Require matching force and Jacobian behavior for missing body ephemerides.
%% INPUT
% testCase   MATLAB assertion context.
%% OUTPUT
% None; verify the public dynamics contract.
%% CHANGELOG
% 11-10-2026  Pietro Califano  Consolidate the isolated gravity regression.
%% DEPENDENCIES
% EvalRHS_InertialDynMaxFidelity, EvalJac_InertialDynMaxFidelity

dxState = [1.0; 0.2; -0.1; 0.0; 0.0; 0.0];
strDynParams = BuildMaxFidelityDynParams_(zeros(3, 1));
strModelConfigFlags = BuildThirdBodyOnlyConfig_();

[dRhs, strAccelInfo] = EvalRHS_InertialDynMaxFidelity( ...
    0.0, dxState, strDynParams, strModelConfigFlags);
dJacobian = EvalJac_InertialDynMaxFidelity( ...
    0.0, dxState, strDynParams, strModelConfigFlags, strAccelInfo);

testCase.verifyEqual(dRhs(4:6), zeros(3, 1), 'AbsTol', 0.0);
testCase.verifyEqual(dJacobian(4:6, 1:3), zeros(3, 3), 'AbsTol', 0.0);
end

function testSunTidalAccelerationSurvivesLargeDistance(testCase)
%% SIGNATURE
% testSunTidalAccelerationSurvivesLargeDistance(testCase)
%% DESCRIPTION
% Retain differential gravity where the old orbital component cutoff returned zero.
%% INPUT
% testCase   MATLAB assertion context.
%% OUTPUT
% None; compare a small nonzero tidal acceleration with a factored axial oracle.
%% CHANGELOG
% 11-10-2026  Pietro Califano  Guard the shared kernel in the public orbital RHS.
%% DEPENDENCIES
% EvalRHS_InertialDynOrbit

dRadius = 2;
dSunRange = 1e13;
dSunGM = 1e11;
dxState = [dRadius; 0; 0; 0; 0; 0];
dRhs = EvalOrbitRhs_(dxState, dSunGM, [dSunRange; 0; 0]);
dExpected = dSunGM * dRadius * (2 * dSunRange - dRadius) / ...
    (dSunRange^2 * (dSunRange - dRadius)^2);
testCase.verifyEqual(dRhs(4), dExpected, 'RelTol', 1e-13);
testCase.verifyEqual(dRhs(5:6), zeros(2, 1));
end

function dRhs = EvalOrbitRhs_(dxState, dThirdBodyGMs, dBodyEphemerides)
dRhs = EvalRHS_InertialDynOrbit( ...
    dxState, eye(3), 0.0, 1.0, 0.0, dThirdBodyGMs, ...
    dBodyEphemerides, [], uint32(0), uint16([1, 6]), zeros(3, 1), false);
end

function dAcceleration_IN = ComputePhysicalThirdBodyAcceleration_( ...
        dSpacecraftPosition_IN, dBodyPosition_IN, dBodyGM)
% Evaluate spacecraft-minus-main-body differential gravity independently.
dBodyFromSpacecraft_IN = dBodyPosition_IN - dSpacecraftPosition_IN;
dAcceleration_IN = dBodyGM .* ( ...
    dBodyFromSpacecraft_IN ./ norm(dBodyFromSpacecraft_IN).^3 - ...
    dBodyPosition_IN ./ norm(dBodyPosition_IN).^3);
end

function strDynParams = BuildMaxFidelityDynParams_(dEarthPosition_IN)
% Build the smallest schema-valid max-fidelity payload with two ephemerides.
strDynParams = struct();
strDynParams.strMainData = struct( ...
    'dGM', 0.0, ...
    'dRefRadius', 1.0, ...
    'dSHcoeff', zeros(4, 2), ...
    'ui16MaxSHdegree', uint16(2));

strDynParams.strBody3rdData(1).dGM = 0.0;
strDynParams.strBody3rdData(1).dRefRadius = 696000.0;
strDynParams.strBody3rdData(1).strOrbitData = BuildConstantOrbitData_([20.0; 4.0; -3.0]);
strDynParams.strBody3rdData(2).dGM = 100.0;
strDynParams.strBody3rdData(2).dRefRadius = 6378.0;
strDynParams.strBody3rdData(2).strOrbitData = BuildConstantOrbitData_(dEarthPosition_IN);

strDynParams.strSRPdata = struct( ...
    'dP_SRP0', 0.0, ...
    'dP_SRP', 0.0, ...
    'dReferenceDistance', 1.0, ...
    'bRecomputePressureFromDistance', false);
strDynParams.strSCdata = struct( ...
    'dReflCoeff', 1.0, ...
    'dSCmass', 1.0, ...
    'dA_SRP', 0.0);
end

function strModelConfigFlags = BuildThirdBodyOnlyConfig_()
strModelConfigFlags = struct( ...
    'bIncludeMainGravity', false, ...
    'bIncludeThirdBodies', false, ...
    'bIncludeSunThirdBody', false, ...
    'bIncludeEarthThirdBody', true, ...
    'bIncludeSRP', false, ...
    'bIncludeSphericalHarmonics', false, ...
    'bIncludePolyhedronGravity', false);
end

function strOrbitData = BuildConstantOrbitData_(dPosition_IN)
% Encode a constant Cartesian ephemeris in the established Chebyshev schema.
strOrbitData = struct( ...
    'ui32PolyDeg', uint32(2), ...
    'dChbvPolycoeffs', [dPosition_IN(1); 0.0; 0.0; ...
                        dPosition_IN(2); 0.0; 0.0; ...
                        dPosition_IN(3); 0.0; 0.0], ...
    'dTimeLowBound', -100.0, ...
    'dTimeUpBound', 100.0);
end
