function strVerification = testSrpLutBias()
%% SIGNATURE
% strVerification = testSrpLutBias()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify signed additive bias along the unbiased scalar/transverse SRP response.
% Check an independent radial oracle and finite differences of the selected
% force with fixed and position-dependent pointing. Include zero response,
% cancellation, pressure independence and equivalent metre/kilometre inputs.
% Example: strVerification = testSrpLutBias();
% Output: Passing force/bias contracts and maximum relative derivative errors.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None; use synthetic optical geometry without production assets.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strVerification    Case count and measured position/bias discrepancies.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 04-10-2026  Pietro Califano     Verify model-aligned additive SRP bias.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSrpLutTestFixture, EvalRHS_SRPLutWithBias.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

arguments (Output)
    strVerification (1, 1) struct
end

strPanelLut = BuildSrpLutTestFixture(false);
strRadialLut = BuildSrpLutTestFixture(true, false);
dMaxPositionError = 0;
dMaxBiasError = 0;
ui32CaseCount = uint32(0);

for bKilometers = [false, true]
    dLengthScale = 1 + 999 * double(bKilometers);
    dSunDisplacement = [13000; 3700; 2800] / dLengthScale;
    dPointingAngle = 0.21;
    strPointing = struct('dDCM_INfromSCB', Rotation_(dPointingAngle), ...
        'dJacDCMWrtPos_INfromSCB', zeros(3, 3, 3));
    strSrpData = struct('dReferencePressure', ...
        4e-6 * (1e4 / 1.495978707e11)^2 * dLengthScale, ...
        'dMass', 12, 'dBiasAcceleration', 0, 'bUseKilometersScale', bKilometers, ...
        'strPointing', strPointing);

    % Recover the independent inverse-square radial force and its derivative.
    dSunRange = norm(dSunDisplacement);
    dSunDirection = dSunDisplacement / dSunRange;
    dPressure = strSrpData.dReferencePressure * ...
        (1.495978707e11 / (dLengthScale * dSunRange))^2;
    dRadialAccel = dPressure / (dLengthScale^2 * strSrpData.dMass);
    strSrpData.dBiasAcceleration = 2e-8 / dLengthScale;
    [dAcceleration, dJacPosition, dJacBias] = ...
        EvalRHS_SRPLutWithBias(dSunDisplacement, strSrpData, strRadialLut, false);
    dExpectedJac = ((dRadialAccel + strSrpData.dBiasAcceleration) * eye(3) - ...
        (3 * dRadialAccel + strSrpData.dBiasAcceleration) * ...
        (dSunDirection * dSunDirection.')) / dSunRange;
    assert(norm(dAcceleration + (dRadialAccel + strSrpData.dBiasAcceleration) * ...
        dSunDirection) < 1e-20);
    assert(norm(dJacPosition - dExpectedJac, 'fro') < 1e-23);
    assert(norm(dJacBias + dSunDirection) < 1e-14);

    % Recover the single-plate optical law independently at an exact table node.
    % Use the fixture's area 0.5 m^2, diffuse 0.15 and specular 0.75 coefficients.
    dNodeDirection = [cosd(20) * cosd(40); cosd(20) * sind(40); sind(20)];
    dIncidenceCosine = dNodeDirection(1);
    dPlateResponse = -0.5 * dIncidenceCosine * ...
        (0.25 * dNodeDirection + (1.5 * dIncidenceCosine + 0.1) * [1; 0; 0]);
    dPlateAcceleration = 4e-6 / (strSrpData.dMass * dLengthScale) * dPlateResponse;
    dPlateDirection = dPlateResponse / norm(dPlateResponse);
    strPlateData = strSrpData;
    strPlateData.strPointing.dDCM_INfromSCB = eye(3);
    [dPlateActual, ~, dPlateSensitivity] = EvalRHS_SRPLutWithBias( ...
        (1e4 / dLengthScale) * dNodeDirection, strPlateData, strPanelLut, true);
    assert(norm(dPlateActual - dPlateAcceleration - ...
        strPlateData.dBiasAcceleration * dPlateDirection) < 1e-20);
    assert(norm(dPlateSensitivity - dPlateDirection) < 1e-14);

    for bTransverse = [false, true]
        for bPointingChain = [false, true]
            % Supply a smooth rotation law and differentiate the same law independently.
            dAngleGradient = bPointingChain * [2e-4, -1e-4, 3e-4] * dLengthScale;
            for ui32Axis = uint32(1):uint32(3)
                strSrpData.strPointing.dJacDCMWrtPos_INfromSCB(:, :, ui32Axis) = ...
                    strPointing.dDCM_INfromSCB * [0, -1, 0; 1, 0, 0; 0, 0, 0] * ...
                    dAngleGradient(ui32Axis);
            end

            strSrpData.dBiasAcceleration = 0;
            [dUnbiased, dUnbiasedJac, dZeroBiasSensitivity, bRegular] = ...
                EvalRHS_SRPLutWithBias(dSunDisplacement, strSrpData, strPanelLut, bTransverse);
            assert(bRegular && norm(dUnbiased) > 0);
            dExpectedDirection = dUnbiased / norm(dUnbiased);
            assert(norm(dZeroBiasSensitivity - dExpectedDirection) < 1e-14);
            if bTransverse
                assert(norm(cross(dExpectedDirection, dSunDirection)) > 0.05);
            end

            % Include exact cancellation and reversal of the resulting total acceleration.
            for dBiasFactor = [-2, -1, 0, 0.4]
                strSrpData.dBiasAcceleration = dBiasFactor * norm(dUnbiased);
                [dAcceleration, dJacPosition, dJacBias] = EvalRHS_SRPLutWithBias( ...
                    dSunDisplacement, strSrpData, strPanelLut, bTransverse);
                assert(norm(dAcceleration - (1 + dBiasFactor) * dUnbiased) < 1e-20);
                assert(norm(dJacBias - dExpectedDirection) < 1e-14);
                if dBiasFactor == 0
                    assert(isequal(dJacPosition, dUnbiasedJac));
                end

                % Request every public output prefix to guard nargout specialization.
                dForceOnly = EvalRHS_SRPLutWithBias( ...
                    dSunDisplacement, strSrpData, strPanelLut, bTransverse);
                [dForceWithJac, dPositionOnly] = EvalRHS_SRPLutWithBias( ...
                    dSunDisplacement, strSrpData, strPanelLut, bTransverse);
                assert(isequal(dAcceleration, dForceOnly, dForceWithJac));
                assert(isequal(dJacPosition, dPositionOnly));

                dStep = 1e-3 / dLengthScale;
                dNumerical = zeros(3, 3);
                for ui32Axis = uint32(1):uint32(3)
                    dOffset = zeros(3, 1);
                    dOffset(ui32Axis) = dStep;
                    strPlus = strSrpData;
                    strMinus = strSrpData;
                    strPlus.strPointing.dDCM_INfromSCB = ...
                        Rotation_(dPointingAngle + dAngleGradient * dOffset);
                    strMinus.strPointing.dDCM_INfromSCB = ...
                        Rotation_(dPointingAngle - dAngleGradient * dOffset);
                    dNumerical(:, ui32Axis) = (EvalRHS_SRPLutWithBias( ...
                        dSunDisplacement - dOffset, strPlus, strPanelLut, bTransverse) - ...
                        EvalRHS_SRPLutWithBias(dSunDisplacement + dOffset, ...
                        strMinus, strPanelLut, bTransverse)) / (2 * dStep);
                end
                dRelativeError = norm(dNumerical - dJacPosition, 'fro') / ...
                    max(norm(dJacPosition, 'fro'), realmin);
                dMaxPositionError = max(dMaxPositionError, dRelativeError);
                assert(dRelativeError < 2e-7);

                % Differentiate bias independently, including at total-force cancellation.
                dBiasStep = 1e-9 / dLengthScale;
                strPlus = strSrpData;
                strMinus = strSrpData;
                strPlus.dBiasAcceleration = strSrpData.dBiasAcceleration + dBiasStep;
                strMinus.dBiasAcceleration = strSrpData.dBiasAcceleration - dBiasStep;
                dNumericalBias = (EvalRHS_SRPLutWithBias(dSunDisplacement, ...
                    strPlus, strPanelLut, bTransverse) - EvalRHS_SRPLutWithBias( ...
                    dSunDisplacement, strMinus, strPanelLut, bTransverse)) / (2 * dBiasStep);
                dMaxBiasError = max(dMaxBiasError, norm(dNumericalBias - dJacBias));
                assert(norm(dNumericalBias - dJacBias) < 1e-12);
                ui32CaseCount = ui32CaseCount + 1;
            end

            % Keep bias magnitude/direction independent of positive pressure scaling.
            for dPressureScale = [0.01, 3]
                strScaled = strSrpData;
                strScaled.dReferencePressure = strSrpData.dReferencePressure * dPressureScale;
                [dScaled, ~, dScaledSensitivity] = EvalRHS_SRPLutWithBias( ...
                    dSunDisplacement, strScaled, strPanelLut, bTransverse);
                assert(norm(dScaled - dPressureScale * dUnbiased - ...
                    strScaled.dBiasAcceleration * dExpectedDirection) < 1e-20);
                assert(norm(dScaledSensitivity - dExpectedDirection) < 1e-14);
            end
        end

        % Distinguish absent radiation from an active query with no illuminated surface.
        strInactive = strSrpData;
        strInactive.dBiasAcceleration = 1e-8 / dLengthScale;
        strInactive.strPointing = struct('dDCM_INfromSCB', eye(3), ...
            'dJacDCMWrtPos_INfromSCB', zeros(3, 3, 3));
        for ui8Inactive = uint8(1):uint8(3)
            dQuery = -dSunDisplacement;
            if ui8Inactive == 2
                strInactive.dReferencePressure = 0;
            elseif ui8Inactive == 3
                strInactive.dReferencePressure = strSrpData.dReferencePressure;
                dQuery(:) = 0;
            end
            [dAcceleration, dJacPosition, dJacBias, bRegular] = ...
                EvalRHS_SRPLutWithBias(dQuery, strInactive, strPanelLut, bTransverse);
            assert(all(dAcceleration == 0) && all(dJacPosition == 0, 'all') && all(dJacBias == 0));
            assert(bRegular == (ui8Inactive ~= 1));
            ui32CaseCount = ui32CaseCount + 1;
        end
    end
end

strVerification = struct('bPassed', true, 'ui32CaseCount', ui32CaseCount, ...
    'dMaxPositionRelativeError', dMaxPositionError, 'dMaxBiasError', dMaxBiasError);
disp(strVerification);
end

function dDCM = Rotation_(dAngle)
% Rotate around body Z for an independent position-dependent pointing fixture.
arguments (Input)
    dAngle (1, 1) double
end

arguments (Output)
    dDCM (3, 3) double
end

dDCM = [cos(dAngle), -sin(dAngle), 0; sin(dAngle), cos(dAngle), 0; 0, 0, 1];
end
