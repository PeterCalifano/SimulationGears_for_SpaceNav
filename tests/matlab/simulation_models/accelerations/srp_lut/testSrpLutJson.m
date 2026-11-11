function tests = testSrpLutJson
%% SIGNATURE
% tests = testSrpLutJson
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate portable compact/fixed force payloads and malformed data.
% Example: results = runtests('testSrpLutJson.m'); assertSuccess(results);
% Output: MATLAB unit tests for native JSON interchange.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests Function-based MATLAB tests.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2026  Pietro Califano  Add portable LUT boundary tests.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; move to srp_lut tests.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ExportSrpResponseLutJson, ImportSrpResponseLutJson.
% -------------------------------------------------------------------------------------------------------------
% MATLAB functiontests rejects argument-validation blocks on tests and their factory.
tests = functiontests(localfunctions);
end

function TestRoundTrip_(objTest)
% Verify all active entries and the explicit force-only extraction from fixed storage.
charDirectory = tempname;
mkdir(charDirectory);
objCleanup = onCleanup(@() rmdir(charDirectory, 's'));
charPath = fullfile(charDirectory, 'response.json');
strTable = struct('dAzimuth', [-180, -90, 0, 90, 180], ...
                  'dElevation', [-90, 0, 90], ...
                  'dEffectiveCr', [1, 1, 1, 1, 1; 2, 3, 4, 5, 2; 6, 6, 6, 6, 6], ...
                  'dReferenceArea_m2', 0.5329);
ExportSrpResponseLutJson(strTable, charPath);
strRestored = ImportSrpResponseLutJson(charPath);
objTest.verifyEqual(strRestored.dEffectiveCr, strTable.dEffectiveCr);
objTest.verifyEqual(strRestored.dAzimuth, strTable.dAzimuth);

% Export a fixed-capacity payload with padding and torque; active force entries return, torque does not
strFixed = strTable;
strFixed.ui32AzimuthCount = uint32(5);
strFixed.ui32ElevationCount = uint32(3);
strFixed.dAzimuth = [strTable.dAzimuth, 0, 0];
strFixed.dElevation = [strTable.dElevation, 0, 0];
strFixed.dEffectiveCr = zeros(5, 7);
strFixed.dEffectiveCr(1:3, 1:5) = strTable.dEffectiveCr;
strFixed.dTransverseForcePerPressure = zeros(3, 5, 7);
strFixed.dTransverseForcePerPressure(:, 2, 2) = [1; 0; 2];
strFixed.dTorquePerPressure = ones(3, 5, 7);
ExportSrpResponseLutJson(strFixed, charPath);
strRestored = ImportSrpResponseLutJson(charPath);
objTest.verifyEqual(strRestored.dEffectiveCr, strTable.dEffectiveCr);
objTest.verifyEqual(strRestored.dTransverseForcePerPressure, ...
    strFixed.dTransverseForcePerPressure(:, 1:3, 1:5));
objTest.verifyFalse(isfield(strRestored, 'dTorquePerPressure'));
end

function TestRejectMalformed_(objTest)
% Reject numerical schema errors before publishing an artifact.
charPath = [tempname, '.json'];
strTable = struct('dAzimuth', [-180, 0, 180], ...
                  'dElevation', [-90, 0, 90], ...
                  'dEffectiveCr', ones(3), ...
                  'dReferenceArea_m2', 1);

% Break the pole agreement, then the Cr sign
strTable.dEffectiveCr(1, 2) = 2;
objTest.verifyError(@() ExportSrpResponseLutJson(strTable, charPath), 'SrpLutJson:Boundaries');
strTable.dEffectiveCr = -ones(3);
objTest.verifyError(@() ExportSrpResponseLutJson(strTable, charPath), 'SrpLutJson:Values');
end

function TestPortableSchemaTypes_(objTest)
% Native and MATLAB consumers must accept the same string/count/axis types.
charDirectory = tempname;
mkdir(charDirectory);
objCleanup = onCleanup(@() rmdir(charDirectory, 's'));
charPath = fullfile(charDirectory, 'response.json');
strTable = struct('dAzimuth', [-180, -90, 0, 90, 180], ...
                  'dElevation', [-90, 0, 90], ...
                  'dEffectiveCr', ones(3, 5), ...
                  'dReferenceArea_m2', 1);
ExportSrpResponseLutJson(strTable, charPath);
strArtifact = jsondecode(fileread(charPath));

% Reject a numeric generator and a sample count above uint32; accept intmax('uint32')
strTable.strJsonProvenance = strArtifact.provenance;
strTable.strJsonProvenance.generator = 42;
objTest.verifyError(@() ExportSrpResponseLutJson(strTable, charPath), 'SrpLutJson:Provenance');
strTable.strJsonProvenance = strArtifact.provenance;
strTable.strJsonProvenance.shadow_samples = 2^32;
objTest.verifyError(@() ExportSrpResponseLutJson(strTable, charPath), 'SrpLutJson:Provenance');
strTable.strJsonProvenance.shadow_samples = double(intmax('uint32'));
ExportSrpResponseLutJson(strTable, charPath);
objTest.verifyEqual(ImportSrpResponseLutJson(charPath).strJsonProvenance.shadow_samples, ...
    double(intmax('uint32')));

% Reject a channel list decoded as a char row instead of a string array
strArtifact.channels = 'Cr';
writelines(jsonencode(strArtifact), charPath);
objTest.verifyError(@() ImportSrpResponseLutJson(charPath), 'SrpLutJson:Channels');

% Accept an axis deviation below the shared 1e-10 tolerance and reject one above it
strTable.dAzimuth(2) = -90 + 1e-11;
ExportSrpResponseLutJson(strTable, charPath);
objTest.verifyEqual(ImportSrpResponseLutJson(charPath).dAzimuth, strTable.dAzimuth);
strTable.dAzimuth(2) = -90 + 1e-8;
objTest.verifyError(@() ExportSrpResponseLutJson(strTable, charPath), 'SrpLutJson:Axes');
end
