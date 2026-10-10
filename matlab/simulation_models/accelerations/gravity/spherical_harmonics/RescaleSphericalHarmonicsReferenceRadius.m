function strRescaledGravityData = ...
        RescaleSphericalHarmonicsReferenceRadius( ...
            strGravityData, dTargetReferenceRadius)
%% SIGNATURE
% strRescaledGravityData = RescaleSphericalHarmonicsReferenceRadius( ...
%     strGravityData, dTargetReferenceRadius)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Rescale repo-native unnormalized [Clm, Slm] coefficient rows to a new
% spherical-harmonics reference radius while preserving the represented
% exterior potential and acceleration field.
%
% For each degree l, coefficients are transformed as
% C_lm(new) = C_lm(old) * (R_old / R_new)^l, with the same rule for S_lm.
% The function changes the normalization convention only; it does not change
% the source geometry or the exterior series convergence domain.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strGravityData               Gravity-data struct containing canonical coefficient rows, maximum degree,
%                              and the current positive reference radius.
% dTargetReferenceRadius       Positive finite target reference radius in the same length units.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strRescaledGravityData       Input gravity data with rescaled coefficients and target reference radius.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-08-2026  Pietro Califano, Codex     Promote the Itokawa demo radius transformation for shared reuse.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strGravityData (1, 1) struct
    dTargetReferenceRadius (1, 1) double {mustBeFinite, mustBeReal, mustBePositive}
end

arguments (Output)
    strRescaledGravityData (1, 1) struct
end

cellRequiredFields = { ...
    'dCSlmCoeffCols', 'ui32MaxDegree', 'dBodyRadiusRef'};
for ui32FieldId = uint32(1):uint32(numel(cellRequiredFields))
    charFieldName = cellRequiredFields{ui32FieldId};
    if ~isfield(strGravityData, charFieldName)
        error('RescaleSphericalHarmonicsReferenceRadius:MissingField', ...
            'strGravityData is missing required field "%s".', ...
            charFieldName);
    end
end

ui32MaxDegree = strGravityData.ui32MaxDegree;
dSourceReferenceRadius = strGravityData.dBodyRadiusRef;
if ~isa(ui32MaxDegree, 'uint32') || ~isscalar(ui32MaxDegree)
    error('RescaleSphericalHarmonicsReferenceRadius:InvalidMaximumDegree', ...
        'ui32MaxDegree must be a uint32 scalar.');
end
ui32ExpectedRows = CountCoeffRows_(ui32MaxDegree);
if ui32MaxDegree < uint32(2) || ...
        size(strGravityData.dCSlmCoeffCols, 1) ~= double(ui32ExpectedRows) || ...
        size(strGravityData.dCSlmCoeffCols, 2) ~= 2 || ...
        any(~isfinite(strGravityData.dCSlmCoeffCols), 'all')
    error('RescaleSphericalHarmonicsReferenceRadius:InvalidCoefficientRows', ...
        'Coefficient rows must contain one finite canonical family of degree at least 2.');
end
if ~isscalar(dSourceReferenceRadius) || ...
        ~isfinite(dSourceReferenceRadius) || ...
        ~(dSourceReferenceRadius > 0.0)
    error('RescaleSphericalHarmonicsReferenceRadius:InvalidSourceRadius', ...
        'The source reference radius must be a positive finite scalar.');
end

% Apply one degree scale to every order belonging to that degree. The first
% canonical row is the degree-1 compatibility term; degrees >=2 contain
% orders zero through degree in triangular row order.
strRescaledGravityData = strGravityData;
dRadiusRatio = dSourceReferenceRadius / dTargetReferenceRadius;
ui32FirstRow = uint32(1);
for ui32Degree = uint32(1):ui32MaxDegree
    if ui32Degree == uint32(1)
        ui32NumDegreeRows = uint32(1);
    else
        ui32NumDegreeRows = ui32Degree + uint32(1);
    end

    ui32LastRow = ui32FirstRow + ui32NumDegreeRows - uint32(1);
    dDegreeScale = dRadiusRatio ^ double(ui32Degree);
    strRescaledGravityData.dCSlmCoeffCols( ...
        ui32FirstRow:ui32LastRow, :) = ...
        strGravityData.dCSlmCoeffCols(ui32FirstRow:ui32LastRow, :) .* ...
        dDegreeScale;
    ui32FirstRow = ui32LastRow + uint32(1);
end

strRescaledGravityData.dBodyRadiusRef = dTargetReferenceRadius;
if isfield(strRescaledGravityData, 'strFitStats') && ...
        isstruct(strRescaledGravityData.strFitStats)
    strRescaledGravityData.strFitStats.dOriginalFitRadius = ...
        dSourceReferenceRadius;
    strRescaledGravityData.strFitStats.charRadiusTransform = ...
        'C_lm(new)=C_lm(old)*(R_old/R_new)^l';
end

end


function ui32NumRows = CountCoeffRows_(ui32MaxDegree)
%% DESCRIPTION
% Count canonical degree/order rows through the requested maximum degree.
% -------------------------------------------------------------------------------------------------------------

ui32NumRows = (ui32MaxDegree + uint32(1)) * ...
    (ui32MaxDegree + uint32(2)) / uint32(2) - uint32(2);

end
