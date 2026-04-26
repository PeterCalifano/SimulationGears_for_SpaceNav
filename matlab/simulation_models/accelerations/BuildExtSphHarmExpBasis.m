function [dPotentialBasis, dGradUBasis, ui32CoeffRowIds, ui8CoeffColIds, dPlm] = BuildExtSphHarmExpBasis( ...
    dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, dGravParam, dBodyRadiusRef) %#codegen
%% PROTOTYPE
% [dPotentialBasis, dGradUBasis, ui32CoeffRowIds, ui8CoeffColIds, dPlm] = BuildExtSphHarmExpBasis( ...
%     dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Builds the linear exterior spherical-harmonics basis at one target-frame
% position. Each column is the perturbative potential and spherical-gradient
% contribution produced by one active coefficient in the repo-native
% [Clm, Slm] storage.
%
% Degree-1 storage is preserved for compatibility but omitted from the
% active basis. Sl0 terms are also omitted because they do not contribute
% to the real-valued field.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCnorm:        [1]       Position norm in the target-body fixed frame.
% dSCLat:            [1]       Geocentric latitude [rad].
% dSCLong:           [1]       Longitude [rad].
% ui32MaxDegree:     [1]       Maximum harmonic degree.
% dGravParam:        [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:    [1]       Reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialBasis:   [1 x Nu]  Potential basis row.
% dGradUBasis:       [3 x Nu]  Spherical-gradient basis rows
%                              [dU/dr; dU/dLat; dU/dLong].
% ui32CoeffRowIds:   [Nu x 1]  Storage row for each active unknown.
% ui8CoeffColIds:    [Nu x 1]  Storage column, 1 for Clm and 2 for Slm.
% dPlm:              [N x 1]   Flattened Legendre lookup table.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Extract shared ExtSphHarm basis used by evaluator and fitter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% buildSHEtrigLUT()
% buildSHElegendreLUT()
% -------------------------------------------------------------------------------------------------------------

if ui32MaxDegree < uint32(2)
    error('BuildExtSphHarmExpBasis:InvalidMaxDegree', ...
        'The maximum degree must be at least 2.');
end

ui32NumUnknowns = ui32MaxDegree * ui32MaxDegree + 2 * ui32MaxDegree - uint32(3);

dPotentialBasis = zeros(1, double(ui32NumUnknowns));
dGradUBasis = zeros(3, double(ui32NumUnknowns));
ui32CoeffRowIds = zeros(double(ui32NumUnknowns), 1, 'uint32');
ui8CoeffColIds = zeros(double(ui32NumUnknowns), 1, 'uint8');

% Build cached trig and Legendre values for this position, which are shared across all basis functions
dTrigLUT = buildSHEtrigLUT(ui32MaxDegree, dSCLat, dSCLong);
[dPlm, ui32IdPlm0] = buildSHElegendreLUT(sin(dSCLat), cos(dSCLat), ui32MaxDegree);

dSinMLongLUT = dTrigLUT(:, 1);
dCosMLongLUT = dTrigLUT(:, 2);
dMTanLatLUT = dTrigLUT(:, 3);

dRadiusRatio = dBodyRadiusRef / dPosSCnorm;
dRadiusRatioPow = dRadiusRatio * dRadiusRatio;

idPair = uint32(2);
idUnknown = uint32(1);

% Loop over the active coefficient pairs and fill the basis columns and storage mapping
for idxDeg = uint32(2):ui32MaxDegree

    dDegree = double(idxDeg);
    idLm0 = ui32IdPlm0(idxDeg + uint32(1));

    for idxOrd = uint32(0):idxDeg
        % Compute the unscaled potential and gradient contributions for this coefficient pair
        idm = idxOrd + uint32(1);

        % Get the associated Legendre function value and its latitude derivative factor for this order
        dPlmVal = dPlm(idLm0 + idxOrd);
        dPlmPlus1 = dPlm(idLm0 + idm);
        dLatFactor = dPlmPlus1 - dMTanLatLUT(idm) * dPlmVal;

        % Compute the unscaled potential and gradient contributions for this degree and order
        dPotBase = dGravParam / dPosSCnorm * dRadiusRatioPow * dPlmVal;
        dUdrBase = -dGravParam / (dPosSCnorm^2) * (dDegree + 1.0) * dRadiusRatioPow * dPlmVal;
        dUdLatBase = dGravParam / dPosSCnorm * dRadiusRatioPow * dLatFactor;
        dUdLongBase = dGravParam / dPosSCnorm * dRadiusRatioPow * double(idxOrd) * dPlmVal;

        dCosTerm = dCosMLongLUT(idm);
        dSinTerm = dSinMLongLUT(idm);

        ui32CoeffRowIds(idUnknown) = idPair;
        ui8CoeffColIds(idUnknown) = uint8(1);

        % Compute the unscaled potential and gradient contributions
        dPotentialBasis(idUnknown) = dPotBase * dCosTerm;
        dGradUBasis(:, idUnknown) = [dUdrBase * dCosTerm; ...
                                     dUdLatBase * dCosTerm; ...
                                    -dUdLongBase * dSinTerm];

        idUnknown = idUnknown + uint32(1);

        if idxOrd > uint32(0)

            ui32CoeffRowIds(idUnknown) = idPair;
            ui8CoeffColIds(idUnknown) = uint8(2);
            dPotentialBasis(idUnknown) = dPotBase * dSinTerm;
            dGradUBasis(:, idUnknown) = [dUdrBase * dSinTerm; ...
                                         dUdLatBase * dSinTerm; ...
                                         dUdLongBase * dCosTerm];
            idUnknown = idUnknown + uint32(1);

        end

        idPair = idPair + uint32(1);
    end

    dRadiusRatioPow = dRadiusRatioPow * dRadiusRatio;
end

end
