function [dGradU, dPlm] = ExtSHE_GradUCore(dPosSCnorm, dSCLat, dSCLong, ...
                                            ui32lMax, dCSlmCoeffCols, ...
                                            dGravParam, dBodyRref) %#codegen
%% PROTOTYPE
% [dGradU, dPlm] = ExtSHE_GradUCore(dPosSCnorm, dSCLat, dSCLong, ui32lMax, ...
%                                  dCSlmCoeffCols, dGravParam, dBodyRref) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Shared core for the Exterior Spherical Harmonics Expansion model.
%
% This function computes the gradient of the non-spherical potential in
% spherical coordinates and returns the Legendre lookup table used during
% the accumulation. It is the common implementation used by ExtSHE_AccTB
% and by the legacy wrapper ExtSHE_GradU.
%
% ACHTUNG: Coefficients must be UN-NORMALIZED and stored as [Clm, Slm]
% column pairs starting from (l,m) = (1,1). Degree-1 entries are ignored
% by the model, which starts at degree 2.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUTS
% dPosSCnorm:      [1]      Position vector norm in the target-body fixed frame.
% dSCLat:          [1]      Geocentric latitude [rad].
% dSCLong:         [1]      Longitude [rad].
% ui32lMax:        [1]      Maximum harmonic degree.
% dCSlmCoeffCols:  [Nl, 2]  Unnormalized [Clm, Slm] coefficient lookup table.
% dGravParam:      [1]      Body gravitational parameter [LU^3/TU^2]
% dBodyRref:       [1]      Reference body radius [LU]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUTS
% dGradU:          [3x1]    Gradient of the non-spherical potential in
%                           spherical coordinates [dU/dr; dU/dLat; dU/dLong].
% dPlm:            [N x1]   Flattened Legendre polynomial lookup table.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% buildSHEtrigLUT()
% buildSHElegendreLUT()
% -------------------------------------------------------------------------------------------------------------

% Get data from input arrays
ui32lMax = uint32(ui32lMax);

dClmLUT = dCSlmCoeffCols(:, 1);
dSlmLUT = dCSlmCoeffCols(:, 2);

% Precompute LUT quantities
dTrigLUT = buildSHEtrigLUT(ui32lMax, dSCLat, dSCLong);
[dPlm, ui32IdPlm0] = buildSHElegendreLUT(sin(dSCLat), cos(dSCLat), ui32lMax);

dSinMLongLUT = dTrigLUT(:, 1);
dCosMLongLUT = dTrigLUT(:, 2);
dMTanLatLUT = dTrigLUT(:, 3);

dUdr = 0.0;
dUdLat = 0.0;
dUdLong = 0.0;

if ui32lMax >= uint32(2)

    dRadiusRatio = dBodyRref / dPosSCnorm;
    dRadiusRatioPow = dRadiusRatio * dRadiusRatio;
    idPair = uint32(1);

    for idxDeg = uint32(2):ui32lMax

        dDegree = double(idxDeg);
        dRadiusRatioPowDeg = dRadiusRatioPow;
        dRadialScale = dRadiusRatioPowDeg * (dDegree + 1.0);

        for idxOrd = uint32(0):idxDeg

            % Determine indices
            idm = idxOrd + uint32(1);
            idLm0 = ui32IdPlm0(idxDeg + uint32(1));

            dPlmVal = dPlm(idLm0 + idxOrd);
            dPlmPlus1 = dPlm(idLm0 + idm);

            dClm = dClmLUT(idPair + uint32(1));
            dSlm = dSlmLUT(idPair + uint32(1));

            % Compute amplitude of harmonic
            dHarmonicAmplitude = dClm * dCosMLongLUT(idm) + dSlm * dSinMLongLUT(idm);

            % Compute gradient wrt range, lat, long
            dUdr = dUdr + dRadialScale * dPlmVal * dHarmonicAmplitude;

            dUdLat = dUdLat + dRadiusRatioPowDeg * ...
                (dPlmPlus1 - dMTanLatLUT(idm) * dPlmVal) * dHarmonicAmplitude;

            dUdLong = dUdLong + dRadiusRatioPowDeg * double(idxOrd) * dPlmVal * ...
                (dSlm * dCosMLongLUT(idm) - dClm * dSinMLongLUT(idm));

            idPair = idPair + uint32(1);
        end

        dRadiusRatioPow = dRadiusRatioPow * dRadiusRatio;
    end
end

% Build gradient vector
dInvR = 1.0 / dPosSCnorm;
dInvR2 = dInvR * dInvR;

dGradU = [-dGravParam * dInvR2 * dUdr; ...
        dGravParam * dInvR * dUdLat; ...
        dGravParam * dInvR * dUdLong];
end
