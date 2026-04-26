function [objShapeModel, strSHgravityData] = FitSpherHarmonicsToPolyhedronGravityFromObj(charObjFilePath, ui32MaxDegree, options)
arguments
    charObjFilePath                 (1,:) string {mustBeA(charObjFilePath, ["string", "char"])}
    ui32MaxDegree                   (1,1) uint32
    options.charInputUnit           (1,:) string {mustBeA(options.charInputUnit, ["string", "char"]), ...
        mustBeMember(options.charInputUnit, ["m", "km"])} = "m"
    options.charTargetUnitOutput    (1,:) string {mustBeA(options.charTargetUnitOutput, ["string", "char"]), ...
        mustBeMember(options.charTargetUnitOutput, ["m", "km"])} = "m"
    options.bVertFacesOnly          (1,1) logical = true
    options.charModelName           (1,:) string {mustBeA(options.charModelName, ["string", "char"])} = ""
    options.dGravParam              (1,1) double = NaN
    options.dDensity                (1,1) double = NaN
    options.dGravConst              (1,1) double = 6.67430e-11
    options.dBodyRadiusRef          (1,1) double = NaN
    options.ui32MaxFitIterations    (1,1) uint32 = uint32(5)
    options.bCacheOnShapeModel      (1,1) logical = true
end
%% PROTOTYPE
% [objShapeModel, strSHgravityData] = FitSpherHarmonicsToPolyhedronGravityFromObj(charObjFilePath, ui32MaxDegree, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute-only wrapper that loads a triangulated mesh from a Wavefront
% .obj file and fits exterior spherical harmonics coefficients from the
% exact polyhedron gravity field.
%
% The function is intentionally shaped like a static builder: no plotting,
% no workflow-side reporting, no diagnostics assembly. Use
% RunFitSpherHarmonicsToPolyhedronGravityFromObj() for the user-facing runnable entry
% point.
%
% Example:
% [objShapeModel, strSHgravityData] = FitSpherHarmonicsToPolyhedronGravityFromObj( ...
%     "/path/to/body.obj", uint32(8), ...
%     dDensity=2100.0, ...
%     ui32MaxFitIterations=uint32(4));
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charObjFilePath:                [1]         Path to the input .obj file.
% ui32MaxDegree:                  [1]         Maximum SH degree.
% options.charInputUnit:          [1]         Input mesh unit ('m' or 'km').
% options.charTargetUnitOutput:   [1]         Internal/output mesh unit ('m' or 'km').
% options.bVertFacesOnly:         [1]         Load only vertices and faces from the .obj file.
% options.charModelName:          [1]         Optional model name.
% options.dGravParam:             [1]         Optional gravitational parameter.
% options.dDensity:               [1]         Optional density.
% options.dGravConst:             [1]         Gravitational constant.
% options.dBodyRadiusRef:         [1]         Optional SH reference radius override.
% options.ui32MaxFitIterations:   [1]         Maximum adaptive fit iterations.
% options.bCacheOnShapeModel:     [1]         Store the SH fit on the returned CShapeModel object.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objShapeModel:                  CShapeModel  Loaded shape model object.
% strSHgravityData:               struct       Fitted spherical harmonics data.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Add end-to-end OBJ-to-SH example entry point.
% 24-04-2026    Pietro Califano     Refactor to compute-only builder-style utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CShapeModel()
% CShapeModel.BuildSphericalHarmonicsGravityDataFromObj()
% -------------------------------------------------------------------------------------------------------------

% DEVNOTE: kept as a standalone function rather than moving the user-facing
% workflow into CShapeModel. This wrapper mirrors the static builder style
% and delegates the actual work to CShapeModel for reuse.
[objShapeModel, strSHgravityData] = CShapeModel.BuildSphericalHarmonicsGravityDataFromObj(charObjFilePath, ui32MaxDegree, ...
                                                                                        charInputUnit=options.charInputUnit, ...
                                                                                        charTargetUnitOutput=options.charTargetUnitOutput, ...
                                                                                        bVertFacesOnly=options.bVertFacesOnly, ...
                                                                                        charModelName=options.charModelName, ...
                                                                                        dGravParam=options.dGravParam, ...
                                                                                        dDensity=options.dDensity, ...
                                                                                        dGravConst=options.dGravConst, ...
                                                                                        dBodyRadiusRef=options.dBodyRadiusRef, ...
                                                                                        ui32MaxFitIterations=options.ui32MaxFitIterations, ...
                                                                                        bCacheOnShapeModel=options.bCacheOnShapeModel);

end
