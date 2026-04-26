function [dPotentialPert, dGradU, dPlm] = EvalExtSphericalHarmExpCore( ...
    dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, dCSlmCoeffCols, ...
    dGravParam, dBodyRadiusRef) %#codegen
arguments
    dPosSCnorm          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dSCLat              (1,1) double {mustBeFinite, mustBeReal}
    dSCLong             (1,1) double {mustBeFinite, mustBeReal}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeFinite, mustBeReal}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dPotentialPert, dGradU, dPlm] = EvalExtSphericalHarmExpCore( ...
%     dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, dCSlmCoeffCols, ...
%     dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Shared core for the Exterior Spherical Harmonics Expansion model.
%
% This function evaluates the non-spherical potential together with its
% spherical-coordinate gradient at a target-body-fixed position specified
% through radius, latitude, and longitude.
%
% ACHTUNG: Coefficients must be UN-NORMALIZED and stored as [Clm, Slm]
% column pairs starting from (l,m) = (1,1). Degree-1 storage is accepted
% for format compatibility but ignored by the model, which starts at
% degree 2.
%
% This is the common implementation used by the target-frame evaluator,
% the world-frame wrapper, and the SH fitting utilities, so MATLAB and
% codegen paths share the same core math.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCnorm:        [1]       Position norm in the target-body fixed frame.
% dSCLat:            [1]       Geocentric latitude [rad].
% dSCLong:           [1]       Longitude [rad].
% ui32MaxDegree:     [1]       Maximum harmonic degree.
% dCSlmCoeffCols:    [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
% dGravParam:        [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:    [1]       Reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:    [1]       Non-spherical potential [LU^2/TU^2].
% dGradU:            [3x1]     Gradient in spherical coordinates
%                              [dU/dr; dU/dLat; dU/dLong].
% dPlm:              [N x 1]   Flattened Legendre lookup table.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-04-2026    Pietro Califano     Add shared SHE core for potential and gradient.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% buildSHEtrigLUT()
% buildSHElegendreLUT()
% -------------------------------------------------------------------------------------------------------------

ui32RequiredRows = (ui32MaxDegree + uint32(1)) * (ui32MaxDegree + uint32(2)) / uint32(2) - uint32(2);
if size(dCSlmCoeffCols, 1) < double(ui32RequiredRows)
    error('EvalExtSphericalHarmExpCore:InsufficientCoefficients', ...
        'Coefficient table must contain at least %u rows for the requested degree.', ...
        ui32RequiredRows);
end

% Build and cache the trigonometric and Legendre function values (shared by potential and gradient evaluations for the same position).
dTrigLUT = buildSHEtrigLUT(ui32MaxDegree, dSCLat, dSCLong);
[dPlm, ui32IdPlm0] = buildSHElegendreLUT(sin(dSCLat), cos(dSCLat), ui32MaxDegree);

% Initialize output variables and intermediate series accumulators
dSinMLongLUT = dTrigLUT(:, 1);
dCosMLongLUT = dTrigLUT(:, 2);
dMTanLatLUT = dTrigLUT(:, 3);

dPotentialSeries = 0.0;
dUdrSeries = 0.0;
dUdLatSeries = 0.0;
dUdLongSeries = 0.0;

if ui32MaxDegree >= uint32(2)

    % Compute constants used in the series expansion
    dRadiusRatio = dBodyRadiusRef / dPosSCnorm;
    dRadiusRatioPow = dRadiusRatio * dRadiusRatio;
    idPair = uint32(1);

    % Evaluate the series expansion for each (l, m) pair, accumulating the potential and gradient contributions
    for idxDeg = uint32(2):ui32MaxDegree

        dDegree = double(idxDeg);
        idLm0 = ui32IdPlm0(idxDeg + uint32(1));

        % Loop over orders m = 0 to l, using the precomputed Legendre and trigonometric values
        for idxOrd = uint32(0):idxDeg

            idm = idxOrd + uint32(1);

            % Get coeffs and compute amplitude for the current harmonic
            dPlmVal = dPlm(idLm0 + idxOrd);
            dPlmPlus1 = dPlm(idLm0 + idm);

            dClm = dCSlmCoeffCols(idPair + uint32(1), 1);
            dSlm = dCSlmCoeffCols(idPair + uint32(1), 2);

            dHarmonicAmplitude = dClm * dCosMLongLUT(idm) + dSlm * dSinMLongLUT(idm);

            % Accumulate the potential and gradient contributions from this harmonic
            dPotentialSeries = dPotentialSeries + dRadiusRatioPow * dPlmVal * dHarmonicAmplitude;

            % Accumulate the radial, latitudinal, and longitudinal gradient contributions
            % RADIAL: dU/dr = (l+1) * (R_ref/r)^(l+2) * P_lm * (C_lm cos(m*long) + S_lm sin(m*long))
            dUdrSeries = dUdrSeries + dRadiusRatioPow * (dDegree + 1.0) * dPlmVal * dHarmonicAmplitude;
            
            % LATITUDINAL: dU/dLat = (R_ref/r)^(l+1) * [dP_lm/dLat * (C_lm cos(m*long) + S_lm sin(m*long)) + P_lm * m * tan(lat) * (C_lm sin(m*long) - S_lm cos(m*long))]
            dUdLatSeries = dUdLatSeries + dRadiusRatioPow * ...
                (dPlmPlus1 - dMTanLatLUT(idm) * dPlmVal) * dHarmonicAmplitude;
            
            % LONGITUDINAL: dU/dLong = (R_ref/r)^(l+1) * P_lm * m * (C_lm sin(m*long) - S_lm cos(m*long))
            dUdLongSeries = dUdLongSeries + dRadiusRatioPow * double(idxOrd) * dPlmVal * ...
                (dSlm * dCosMLongLUT(idm) - dClm * dSinMLongLUT(idm));

            idPair = idPair + uint32(1);
        end

        dRadiusRatioPow = dRadiusRatioPow * dRadiusRatio;
    end
end

% Combine the series contributions with the gravitational parameter and radius to compute the final potential and gradient values
dInvR = 1.0 / dPosSCnorm;
dInvR2 = dInvR * dInvR;

dPotentialPert = dGravParam * dInvR * dPotentialSeries;
dGradU = [-dGravParam * dInvR2 * dUdrSeries; ...
          dGravParam * dInvR * dUdLatSeries; ...
          dGravParam * dInvR * dUdLongSeries];

end
