function strTable = ImportSrpResponseLutJson(charInputPath)
%% SIGNATURE
% strTable = ImportSrpResponseLutJson(charInputPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Load and validate the portable native force-table JSON into a compact MATLAB
% table. Preserve SI values and provenance without constructing codegen storage.
% Example: strTable = ImportSrpResponseLutJson('response.json');
% Output: dEffectiveCr(elevation,azimuth) and optional transverse(3,elevation,azimuth).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charInputPath Path to a SimulationGears.SrpResponseLut.v1 JSON file.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strTable Compact force table with optional transverse samples and JSON provenance.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2026  Pietro Califano  Add native portable force-table interchange.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; move to srp/lut.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ValidateSrpLutJsonArtifact_, jsondecode, fileread.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    charInputPath (1,:) char
end
arguments (Output)
    strTable (1,1) struct
end

% Decode the file and convert it through the shared validator
strArtifact = jsondecode(fileread(charInputPath));
strTable = ValidateSrpLutJsonArtifact_(strArtifact);
end
