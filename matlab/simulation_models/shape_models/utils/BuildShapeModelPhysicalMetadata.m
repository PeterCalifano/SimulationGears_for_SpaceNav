function [strPhysicalMetadata, strSHinputs] = BuildShapeModelPhysicalMetadata(objShapeModel, options)
arguments
    objShapeModel (1,1) CShapeModel
    options.charLengthUnits {mustBeA(options.charLengthUnits, ["string", "char", "EnumLengthUnits"])} = "m"
    options.dMass_kg (1,1) double = NaN
    options.dDensity_kgm3 (1,1) double = NaN
    options.dVolume_m3 (1,1) double = NaN
    options.dGravParam_m3mps2 (1,1) double = NaN
    options.dLegacyGravParam (1,1) double = NaN
    options.dLegacyDensity (1,1) double = NaN
    options.dGravConstSI (1,1) double {mustBeFinite, mustBePositive} = 6.67430e-11
end
%% DESCRIPTION
% Resolves shape physical metadata from the explicit SI custom-shape
% contract plus legacy SH-native aliases. Returned SH inputs are expressed
% in the active shape-model length units.
% -------------------------------------------------------------------------------------------------------------

charLengthUnits = EnumLengthUnits.toString(options.charLengthUnits);
dLengthUnitInMeters = GetLengthUnitScale_(charLengthUnits);
dVolumeScaleToM3 = dLengthUnitInMeters^3;

dMass_kg = options.dMass_kg;
dDensity_kgm3 = options.dDensity_kgm3;
dVolume_m3 = options.dVolume_m3;
dGravParam_m3mps2 = options.dGravParam_m3mps2;

bHasLegacyGravParam = isfinite(options.dLegacyGravParam);
bHasLegacyDensity = isfinite(options.dLegacyDensity);

if bHasLegacyGravParam
    dLegacyGravParam_m3mps2 = options.dLegacyGravParam * dVolumeScaleToM3;
    dGravParam_m3mps2 = SelectOrValidate_(dGravParam_m3mps2, dLegacyGravParam_m3mps2, ...
        'BuildShapeModelPhysicalMetadata:InconsistentPhysicalInputs', ...
        'Legacy and SI gravitational parameters are inconsistent.');
end
if bHasLegacyDensity
    dLegacyDensity_kgm3 = options.dLegacyDensity / dVolumeScaleToM3;
    dDensity_kgm3 = SelectOrValidate_(dDensity_kgm3, dLegacyDensity_kgm3, ...
        'BuildShapeModelPhysicalMetadata:InconsistentPhysicalInputs', ...
        'Legacy and SI densities are inconsistent.');
end

ValidatePositiveOrNaN_(dMass_kg, 'dMass_kg');
ValidatePositiveOrNaN_(dDensity_kgm3, 'dDensity_kgm3');
ValidatePositiveOrNaN_(dVolume_m3, 'dVolume_m3');
ValidatePositiveOrNaN_(dGravParam_m3mps2, 'dGravParam_m3mps2');

bHasPhysicalInput = any(isfinite([dMass_kg, dDensity_kgm3, dGravParam_m3mps2, dVolume_m3]));
if bHasPhysicalInput && ~isfinite(dVolume_m3)
    assert(objShapeModel.hasData(), 'BuildShapeModelPhysicalMetadata:MissingMeshVolume', ...
        'A loaded mesh or dVolume_m3 is required to derive custom physical metadata.');

    dVolumeModelUnits = ComputeMeshModelVolumeAndCoM( ...
        uint32(objShapeModel.ui32triangVertexPtr'), objShapeModel.dVerticesPos');
    dVolume_m3 = dVolumeModelUnits * dVolumeScaleToM3;
end

if isfinite(dVolume_m3)
    ValidatePositiveOrNaN_(dVolume_m3, 'dVolume_m3');
end

dMassCandidates = zeros(1, 0);
if isfinite(dMass_kg)
    dMassCandidates(end + 1) = dMass_kg;
end
if isfinite(dGravParam_m3mps2)
    dMassCandidates(end + 1) = dGravParam_m3mps2 / options.dGravConstSI;
end
if isfinite(dDensity_kgm3) && isfinite(dVolume_m3)
    dMassCandidates(end + 1) = dDensity_kgm3 * dVolume_m3;
end

bHasMassModel = ~isempty(dMassCandidates);
if bHasMassModel
    dMass_kg = ValidateConsistentCandidates_(dMassCandidates, ...
        'BuildShapeModelPhysicalMetadata:InconsistentPhysicalInputs', ...
        'Custom mass, density, volume, and gravitational parameter inputs are inconsistent.');
    dGravParam_m3mps2 = options.dGravConstSI * dMass_kg;

    if isfinite(dVolume_m3)
        dDensity_kgm3 = dMass_kg / dVolume_m3;
    end
end

strPhysicalMetadata = struct( ...
    'bHasPhysicalInput', bHasPhysicalInput, ...
    'bHasPhysicalMetadata', bHasMassModel && isfinite(dDensity_kgm3) && isfinite(dGravParam_m3mps2), ...
    'bHasSphericalHarmonicsGravityData', false, ...
    'dVolume_m3', dVolume_m3, ...
    'dMass_kg', dMass_kg, ...
    'dDensity_kgm3', dDensity_kgm3, ...
    'dGravParam_m3mps2', dGravParam_m3mps2);

strSHinputs = struct( ...
    'dGravParam', dGravParam_m3mps2 / dVolumeScaleToM3, ...
    'dDensity', dDensity_kgm3 * dVolumeScaleToM3, ...
    'dGravConst', options.dGravConstSI / dVolumeScaleToM3);
end

function dScale = GetLengthUnitScale_(charLengthUnits)
if strcmpi(charLengthUnits, "km")
    dScale = 1000.0;
else
    dScale = 1.0;
end
end

function dValue = SelectOrValidate_(dPrimary, dCandidate, charErrorId, charMessage)
if isfinite(dPrimary)
    ValidateConsistentCandidates_([dPrimary, dCandidate], charErrorId, charMessage);
    dValue = dPrimary;
else
    dValue = dCandidate;
end
end

function dValue = ValidateConsistentCandidates_(dCandidates, charErrorId, charMessage)
dCandidates = dCandidates(isfinite(dCandidates));
assert(~isempty(dCandidates), charErrorId, charMessage);
dValue = dCandidates(1);

for idxCandidate = 2:numel(dCandidates)
    dRelMismatch = abs(dCandidates(idxCandidate) - dValue) / max(abs(dValue), eps(dValue));
    if dRelMismatch > 1.0e-8
        error(charErrorId, charMessage);
    end
end
end

function ValidatePositiveOrNaN_(dValue, charName)
if isfinite(dValue) && ~(dValue > 0.0)
    error('BuildShapeModelPhysicalMetadata:InvalidPhysicalInput', ...
        '%s must be positive when specified.', charName);
end
end
