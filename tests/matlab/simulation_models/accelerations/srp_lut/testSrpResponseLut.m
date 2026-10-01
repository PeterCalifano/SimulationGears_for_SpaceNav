function testSrpResponseLut()
%% SIGNATURE
% testSrpResponseLut()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify optional transverse compensation independently of a mission, mesh or
% source kernel. Exercise nodal reproduction, bilinear scalar parity, normalized
% directions, periodic seam/poles, retained parallel response, and identified
% malformed-table failures. Reject field-type, complex and sparse payloads that
% would change the generated interface. Use an analytic passive response fixture.
% Example: addpath('tests'); testSrpResponseLut();
% Output: Print a passing summary or raise the failed contract assertion.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None. Assert the independent optional scalar/vector interpolation contract.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Cover nodal transverse data and constant inclusion.
% 28-09-2026  Pietro Califano, Codex gpt-6  Verify the generic SRP response LUT.
% 28-09-2026  Pietro Califano, Codex gpt-6  Cover fixed real/dense schema types.
% 29-09-2026  Pietro Califano, Codex gpt-6  Distinguish adjusted poles from interior nodes.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvaluateSrpResponseLut, ValidateSrpResponseLut, GenerateSrpLutDirections.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

charOriginalPath = path;
objCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
% Resolve owner APIs through SetupSimGears before invoking this harness.
strLut = AnalyticTable_();
ValidateSrpResponseLut(strLut);

% Recover interior nodal response and bound the intended tiny pole adjustment.
for ui32Elevation = uint32(1):strLut.ui32ElevationCount
    for ui32Azimuth = uint32(1):strLut.ui32AzimuthCount
        dSunDir_SCB = Direction_(strLut.dAzimuth(ui32Azimuth), strLut.dElevation(ui32Elevation));
        [dVector, dEffectiveCr, dTransverse] = EvaluateSrpResponseLut(dSunDir_SCB, strLut, true);
        dExpected = -strLut.dReferenceArea_m2 * dEffectiveCr * dSunDir_SCB + ...
            strLut.dTransverseForcePerPressure(:, ui32Elevation, ui32Azimuth);
        dNodalError = norm(dVector - dExpected);
        if ui32Elevation == 1 || ui32Elevation == strLut.ui32ElevationCount
            assert(dNodalError < 1e-7);  % Square metres for this analytic unit-area fixture.
        else
            assert(dNodalError < 1e-12);
        end
        assert(abs(dot(dTransverse, dSunDir_SCB)) < 1e-12);
        [dScalar, dScalarCr, dDisabled] = EvaluateSrpResponseLut(dSunDir_SCB, strLut);
        assert(dScalarCr == dEffectiveCr && isequal(dDisabled, zeros(3, 1)));
        assert(norm(dScalar+dSunDir_SCB*dEffectiveCr) < 1e-12);
    end
end

% Preserve the existing bilinear scalar result and add only perpendicular force.
objScalar = griddedInterpolant({strLut.dElevation(1:strLut.ui32ElevationCount), ...
    strLut.dAzimuth(1:strLut.ui32AzimuthCount)}, ...
    strLut.dEffectiveCr(1:strLut.ui32ElevationCount, 1:strLut.ui32AzimuthCount), 'linear', 'none');
dDirections = [GenerateSrpLutDirections(uint32(96)), Direction_(45, 45)];
for ui32Direction = uint32(1):uint32(size(dDirections, 2))
    dSunDir_SCB = dDirections(:, ui32Direction);
    [dScalar, dScalarCr] = EvaluateSrpResponseLut(dSunDir_SCB, strLut, false);
    [dVector, dVectorCr, dTransverse] = EvaluateSrpResponseLut(dSunDir_SCB, strLut, true);
    dExpectedCr = objScalar(asind(dSunDir_SCB(3)), atan2d(dSunDir_SCB(2), dSunDir_SCB(1)));
    assert(abs(dScalarCr-dExpectedCr) < 1e-12 && dScalarCr == dVectorCr);
    assert(norm(dVector-dScalar-dTransverse) < 1e-12);
    assert(abs(dot(dVector-dScalar, dSunDir_SCB)) < 1e-12);
    assert(dot(dVector, -dSunDir_SCB) >= 0);
    assert(norm(EvaluateSrpResponseLut(17*dSunDir_SCB, strLut, true)-dVector) < 1e-12);
end
[~, dMidpointCr] = EvaluateSrpResponseLut(Direction_(45, 45), strLut, true);
assert(abs(dMidpointCr-1.1) < 1e-12);
assert(norm(EvaluateSrpResponseLut([0.5;0.5;sqrt(0.5)]*1e300, strLut, true)- ...
    EvaluateSrpResponseLut(Direction_(45, 45), strLut, true)) < 1e-12);

% Test near-boundary continuity and exact pole independence across azimuth.
assert(norm(EvaluateSrpResponseLut(Direction_(-180, 21), strLut, true)- ...
    EvaluateSrpResponseLut(Direction_(180, 21), strLut, true)) < 1e-12);
assert(norm(EvaluateSrpResponseLut(Direction_(-180+1e-7, 21), strLut, true)- ...
    EvaluateSrpResponseLut(Direction_(180-1e-7, 21), strLut, true)) < 1e-8);
for dPole = [-90, 90]
    assert(norm(EvaluateSrpResponseLut(Direction_(-137, dPole), strLut, true)- ...
        EvaluateSrpResponseLut(Direction_(41, dPole), strLut, true)) < 1e-12);
end

% Reject identified contract violations instead of incidental array errors.

strScalar = rmfield(strLut, 'dTransverseForcePerPressure');
ValidateSrpResponseLut(strScalar);
assert(isequal(EvaluateSrpResponseLut([1;2;3], strScalar), ...
               EvaluateSrpResponseLut([1;2;3], strLut)));

strInvalid = rmfield(strLut, 'dEffectiveCr');
ExpectFailure_(strInvalid, 'MissingField');

strInvalid = strLut;
strInvalid.dAzimuth(1) = -179;
ExpectFailure_(strInvalid, 'InvalidAxes');

strInvalid = strLut;
strInvalid.dElevation(2) = 1;
ExpectFailure_(strInvalid, 'InvalidAxes');

strInvalid = strLut;
strInvalid.dReferenceArea_m2 = -1;
ExpectFailure_(strInvalid, 'InvalidUnits');

strInvalid = strLut;
strInvalid.dTransverseForcePerPressure(2, 2, 2) = NaN;
ExpectFailure_(strInvalid, 'InvalidValues');

strInvalid = strLut;
strInvalid.dTransverseForcePerPressure(2, 2, strLut.ui32AzimuthCount) = 0.1;
ExpectFailure_(strInvalid, 'InvalidBoundary');

strInvalid = strLut;
strInvalid.dTransverseForcePerPressure(1, 1, 2) = 1;
ExpectFailure_(strInvalid, 'InvalidBoundary');

strInvalid = strLut;
strInvalid.dTransverseForcePerPressure(2, 2, 2) = 1;
ExpectFailure_(strInvalid, 'InconsistentProjection');

strInvalid = strLut;
strInvalid.charMetadata = 'Host only';
ExpectFailure_(strInvalid, 'InvalidSchema');

strInvalid = strLut;
strInvalid.ui32AzimuthCount = double(strInvalid.ui32AzimuthCount);
ExpectFailure_(strInvalid, 'InvalidSchema');

strInvalid = strLut;
strInvalid.dEffectiveCr = single(strInvalid.dEffectiveCr);
ExpectFailure_(strInvalid, 'InvalidSchema');

strInvalid = strLut;
strInvalid.dReferenceArea_m2 = 1+1i;
ExpectFailure_(strInvalid, 'InvalidSchema');

strInvalid = strLut;
strInvalid.dAzimuth = sparse(strInvalid.dAzimuth);
ExpectFailure_(strInvalid, 'InvalidSchema');

strInvalid = strLut;
strInvalid.ui32AzimuthCount = uint32(362);
ExpectFailure_(strInvalid, 'CapacityExceeded');

strInvalid = strLut;
strInvalid.dAzimuth = strInvalid.dAzimuth(1:360);
ExpectFailure_(strInvalid, 'InvalidSchema');

strInvalid = strLut;
strInvalid.dEffectiveCr(end, end) = 1;
ExpectFailure_(strInvalid, 'InvalidPadding');
try
    EvaluateSrpResponseLut(zeros(3, 1), strLut, true);
    error('testSrpResponseLut:MissingError', 'The zero direction was accepted.');
catch objError
    assert(strcmp(objError.identifier, 'EvaluateSrpResponseLut:ZeroDirection'));
end
fprintf('SRP response LUT passed: optional vector invariants, fixed schema, interpolation, boundaries and 17 rejections.\n');
end

function strLut = AnalyticTable_()
% Sample a unit spherical response plus an illuminated fixed normal component.
arguments (Output)
    strLut (1, 1) struct
end

dAzimuth = -180:90:180;
dElevation = -90:90:90;
dTransverseGrid = zeros(3, numel(dElevation), numel(dAzimuth));
dEffectiveCr = zeros(numel(dElevation), numel(dAzimuth));
for ui32Elevation = uint32(1):uint32(numel(dElevation))
    for ui32Azimuth = uint32(1):uint32(numel(dAzimuth))
        dSunDir_SCB = Direction_(dAzimuth(ui32Azimuth), dElevation(ui32Elevation));
        dForcePerPressure_SCB = -dSunDir_SCB - [0.15;0;0.2] * max(0, dSunDir_SCB(3));
        dTransverseGrid(:, ui32Elevation, ui32Azimuth) = ...
            dForcePerPressure_SCB - dSunDir_SCB * dot(dForcePerPressure_SCB, dSunDir_SCB);
        dEffectiveCr(ui32Elevation, ui32Azimuth) = -dot(dForcePerPressure_SCB, dSunDir_SCB);
    end
end
dTransverseGrid(:, :, end) = dTransverseGrid(:, :, 1);
dTransverseGrid(:, 1, :) = repmat(dTransverseGrid(:, 1, 1), 1, 1, numel(dAzimuth));
dTransverseGrid(:, end, :) = repmat(dTransverseGrid(:, end, 1), 1, 1, numel(dAzimuth));
strLut = struct('dAzimuth', dAzimuth, 'dElevation', dElevation, 'dEffectiveCr', dEffectiveCr, ...
    'dTransverseForcePerPressure', dTransverseGrid, 'dReferenceArea_m2', 1, 'charVectorValueUnits', 'm^2');
strLut = PackSrpResponseLut(strLut, bIncludeTransverse=true);
end

function dDirection = Direction_(dAzimuth, dElevation)
% Convert grid angles to a unit Sun direction for the interpolation oracle.
arguments (Input)
    dAzimuth (1, 1) double
    dElevation (1, 1) double
end

arguments (Output)
    dDirection (3, 1) double
end

dDirection = [cosd(dElevation)*cosd(dAzimuth);cosd(dElevation)*sind(dAzimuth);sind(dElevation)];
end

function ExpectFailure_(strLut, charSuffix)
% Require the advertised schema rejection instead of an incidental indexing error.
arguments (Input)
    strLut (1, 1) struct
    charSuffix (1, :) char
end

try
    ValidateSrpResponseLut(strLut);
catch objError
    assert(strcmp(objError.identifier, ['ValidateSrpResponseLut:', charSuffix]));
    return
end
error('testSrpResponseLut:MissingError', 'The malformed table was accepted.');
end
