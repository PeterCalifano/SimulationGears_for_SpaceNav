function dDV_WithError = ApplyImpulsiveDeltaVError( dNominalDV, ...
                                                    dSigmaMagnitudeFrac, ...
                                                    dSigmaDirectionInRad ) %#codegen
arguments
    dNominalDV              (3,1) double {mustBeReal}
    dSigmaMagnitudeFrac     (1,1) double {mustBeNonnegative}
    dSigmaDirectionInRad    (1,1) double {mustBeNonnegative}
end
%% SIGNATURE
% [objOutputPack] = main_navsystem_simulation_core(objTrialSetup, ...
%                                                  objNavSystemAlgorithmsConfig, ...
%                                                  objNavSystemPathsAndOptionsManager, ...
%                                                  bLoadSetupOnly)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function applying magnitude and direction error to a nominal delta V vector, given 1-sigma errors in
% magnitude and direction of DeltaV vector.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dNominalDV              (3,1) double {mustBeReal}
% dSigmaMagnitudeFrac     (1,1) double {mustBeNonnegative}
% dSigmaDirectionInRad    (1,1) double {mustBeNonnegative}
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDV_WithError (3,1) double
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-12-2025    Pietro Califano     Implement first version.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------


% -------------------------------------------------------------
% Normalize nominal delta-V (avoid division by zero)
dNormDV = norm(dNominalDV);
if abs(dNormDV) < eps
    dDV_WithError = dNominalDV; % DV is very small!
    return
end

dUnitVecDV = dNominalDV / dNormDV;

% Magnitude error (parallel component)
dMagErr = dSigmaMagnitudeFrac * dNormDV * randn(1,1);
dDV_MagErrorAlongDir = dMagErr * dUnitVecDV;

% -------------------------------------------------------------
% Direction error (orthogonal component)
% Build orthonormal basis (u1,u2) orthogonal to ΔV direction
if dSigmaDirectionInRad > 0
    % Generate random axis orthogonal to dUnitVecDV
    
    % Start from a random vector
    dAxisTmp = randn(3,1);

    % Remove component along dUnitVecDV
    dAxisTmp = dAxisTmp - (transpose(dAxisTmp) * dUnitVecDV) * dUnitVecDV;
    dAxisNorm = norm(dAxisTmp);

    if dAxisNorm < 1e-12
        % Fallback deterministic axis if random is unlucky
        if abs(dUnitVecDV(1)) < 0.9
            dAxisTmp = [1;0;0] - dUnitVecDV(1)*dUnitVecDV;
        else
            dAxisTmp = [0;1;0] - dUnitVecDV(2)*dUnitVecDV;
        end
        dAxisTmp = dAxisTmp / norm(dAxisTmp);
    else
        dAxisTmp = dAxisTmp / dAxisNorm;
    end

    % Small random rotation angle (Gaussian, rad)
    dThetaErr = dSigmaDirectionInRad * randn(1,1);

    % For small angles: dv_dir ≈ dtheta (k × v_nom)
    dDV_DirErrorTangential = dThetaErr * cross(dAxisTmp, dNominalDV);
else
    dDV_DirErrorTangential = zeros(3,1);
end

% -------------------------------------------------------------
% Compute deltaV with errors
dDV_WithError = dNominalDV + dDV_MagErrorAlongDir + dNormDV * dDV_DirErrorTangential;

end
