function strVerification = testOrbitalSrpLutModels
%% SIGNATURE
% strVerification = testOrbitalSrpLutModels
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify LUT/bias selection, force reporting and residual composition in the orbital RHS.
% Use a compact synthetic table and an independent radial-force reference.
% Example: strVerification = testOrbitalSrpLutModels;
% Output: Passing unit, bias, eclipse and inactive-force cases.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None; use offline fixtures without filter configuration or external assets.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification   Case count and maximum acceleration discrepancy.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Verify the shared selected-SRP diagnostic contract.
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 30-09-2026  Pietro Califano, Codex gpt-6  Verify shared orbital-RHS ownership of LUT SRP.
% 01-10-2026  Pietro Califano, Codex gpt-6  Keep eclipse handling in the orbital RHS.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpLutTestFixture, EvalRHS_SRPLutWithBias, EvalRHS_InertialDynOrbit.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

arguments (Output)
    strVerification (1, 1) struct
end

% Use coefficient two and reference area 0.5 m^2 for a one-square-metre oracle.
strResponseLut = BuildSrpLutTestFixture(true);
ui32CaseCount = uint32(0);
dMaxDiscrepancy = 0;
for bKilometers = [false, true]
    dLengthScale = 1;
    if bKilometers
        dLengthScale = 1000;
    end

    % Express the same physical geometry, residual and bias in either length scale.
    dxState = [1200; 500; 300; 0.01; 0.03; -0.01] / dLengthScale;
    dSunPosition_IN = [15200; 9200; 4800] / dLengthScale;
    dResidualAccel = [3e-10; -2e-10; 1e-10] / dLengthScale;
    dPosSCtoSun_IN = dSunPosition_IN - dxState(1:3);
    dSunRange = norm(dPosSCtoSun_IN);
    strPointing = struct('dDCM_INfromSCB', eye(3), 'dJacDCMWrtPos_INfromSCB', zeros(3, 3, 3));
    strSrpData = struct('dReferencePressure', 4e-6 * (1e4 / 1.495978707e11)^2 * dLengthScale, ...
                       'dMass', 12, 'dBiasAcceleration', 0, 'bUseKilometersScale', bKilometers, ...
                       'strPointing', strPointing);

    % Compare shared composition with the force kernel and an independent radial expression.
    for bTransverse = [false, true]
        for dBiasSign = [-1, 0, 1]
            strSrpData.dBiasAcceleration = dBiasSign * 2e-8 / dLengthScale;
            dExpectedSRPaccel_IN = EvalRHS_SRPLutWithBias(dPosSCtoSun_IN, strSrpData, strResponseLut, bTransverse);
            if ~bTransverse
                dPressure = strSrpData.dReferencePressure * ...
                    (1.495978707e11 / dLengthScale / dSunRange)^2;
                dCoefficient = dPressure / (dLengthScale^2 * strSrpData.dMass);
                dReferenceSRPaccel_IN = -(dCoefficient + strSrpData.dBiasAcceleration) * ...
                    dPosSCtoSun_IN / dSunRange;
                assert(norm(dExpectedSRPaccel_IN - dReferenceSRPaccel_IN) < 1e-20);
            end
            for bEclipse = [false, true]
                % Supply a nonzero cannonball coefficient to prove the selected model takes precedence.
                [dDerivative, strInfo] = EvalRHS_InertialDynOrbit(dxState, eye(3), 0, 1, ...
                    9, 0, dSunPosition_IN, [], uint32(0), uint16([1, 6]), dResidualAccel, ...
                    bEclipse, true, strResponseLut, strSrpData, bTransverse);
                dExpectedSrp = dExpectedSRPaccel_IN * ~bEclipse;
                dDiscrepancy = norm(dDerivative(4:6) - dExpectedSrp - dResidualAccel);
                assert(dDiscrepancy < 1e-20);
                assert(isequal(dDerivative(1:3), dxState(4:6)));
                assert(isequal(strInfo.dAccSRP, dExpectedSrp));
                assert(strInfo.bIsSRPActive == any(dExpectedSrp ~= 0));
                assert(strInfo.dSRPdistToSun == dSunRange);
                dMaxDiscrepancy = max(dMaxDiscrepancy, dDiscrepancy);
                ui32CaseCount = ui32CaseCount + 1;
            end
        end
    end

    % Suppress SRP without suppressing an unrelated residual acceleration.
    for ui8InactiveCase = uint8(1:2)
        strInactiveData = strSrpData;
        dInactiveSun = dSunPosition_IN;
        if ui8InactiveCase == 1
            strInactiveData.dReferencePressure = 0;
        else
            dInactiveSun(:) = 0;
        end
        [dDerivative, strInfo] = EvalRHS_InertialDynOrbit(dxState, eye(3), 0, 1, ...
            0, 0, dInactiveSun, [], uint32(0), uint16([1, 6]), dResidualAccel, ...
            false, true, strResponseLut, strInactiveData, false);
        assert(isequal(dDerivative(4:6), dResidualAccel));
        assert(isequal(strInfo.dAccSRP, zeros(3, 1)) && ~strInfo.bIsSRPActive);
        ui32CaseCount = ui32CaseCount + 1;
    end

    % Preserve positional calls and share one diagnostic layout across SRP models.
    [dPositional, strPositional] = EvalRHS_InertialDynOrbit(dxState, eye(3), 0, 1, ...
        2e-8 / dLengthScale, 0, dSunPosition_IN, [], uint32(0), uint16([1, 6]), dResidualAccel);
    [dExplicit, strExplicit] = EvalRHS_InertialDynOrbit(dxState, eye(3), 0, 1, ...
        2e-8 / dLengthScale, 0, dSunPosition_IN, [], uint32(0), uint16([1, 6]), dResidualAccel, false, false);
    assert(isequal(dPositional, dExplicit) && isequal(strPositional, strExplicit));
    assert(isequal(fieldnames(strPositional), fieldnames(strInfo)));
end

strVerification = struct('ui32CaseCount', ui32CaseCount, 'dMaxDiscrepancy', dMaxDiscrepancy);
end
