function strModelConfig = ResolveInertialDynMaxFidelityConfig(strDynParams, strModelConfigFlags) %#codegen
arguments
    strDynParams (1,1) struct
    strModelConfigFlags (1,1) struct = struct()
end
%% PROTOTYPE
% strModelConfig = ResolveInertialDynMaxFidelityConfig(strDynParams, strModelConfigFlags)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve compile-time model-configuration flags and static force-model availability for max-fidelity inertial
% dynamics. These flags select the generated model structure; they are not intended to vary during propagation.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strDynParams:          (1,1) struct   Dynamics payload with main-body, third-body, SRP, and spacecraft data.
% strModelConfigFlags:   (1,1) struct   Optional compile-time model-configuration overrides.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strModelConfig:        (1,1) struct   Resolved static model configuration consumed by RHS/Jacobian entry points.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-05-2026    Pietro Califano, Codex 5.5      Extract max-fidelity model configuration resolution.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
bIncludeMainGravity = GetConfigFlag_(strModelConfigFlags, 'bIncludeMainGravity', true);
bIncludeSphericalHarmonics = GetConfigFlag_(strModelConfigFlags, 'bIncludeSphericalHarmonics', true);
bIncludeThirdBodies = GetConfigFlag_(strModelConfigFlags, 'bIncludeThirdBodies', true);
bIncludeSunThirdBody = GetConfigFlag_(strModelConfigFlags, 'bIncludeSunThirdBody', bIncludeThirdBodies);
bIncludeEarthThirdBody = GetConfigFlag_(strModelConfigFlags, 'bIncludeEarthThirdBody', bIncludeThirdBodies);
bIncludeSRP = GetConfigFlag_(strModelConfigFlags, 'bIncludeSRP', true);
bIncludeEclipse = GetConfigFlag_(strModelConfigFlags, 'bIncludeEclipse', true);
bUsePanelSRP = GetConfigFlag_(strModelConfigFlags, 'bUsePanelSRP', true);
bIncludePolyhedronGravity = GetConfigFlag_(strModelConfigFlags, 'bIncludePolyhedronGravity', true);

bRecomputeSRPpressureFromDistance = true;
if coder.const(isfield(strModelConfigFlags, 'bRecomputeSRPpressureFromDistance'))

    bRecomputeSRPpressureFromDistance = ...
        coder.const(logical(strModelConfigFlags.bRecomputeSRPpressureFromDistance));

elseif coder.const(isfield(strDynParams, 'strSRPdata')) && ...
        coder.const(isfield(strDynParams.strSRPdata, 'bRecomputePressureFromDistance'))

    bRecomputeSRPpressureFromDistance = ...
        coder.const(logical(strDynParams.strSRPdata.bRecomputePressureFromDistance));
end

% SH activation is schema-driven: empty coefficients disable SH, non-empty coefficients require a valid degree.
bHasSphericalHarmonicsData = false;
ui32MaxSHdegree = uint32(0);

if bIncludeSphericalHarmonics && ...
        coder.const(isfield(strDynParams.strMainData, 'dSHcoeff')) && ...
        ~isempty(strDynParams.strMainData.dSHcoeff)

    bHasSphericalHarmonicsData = true;
    if ~coder.const(isfield(strDynParams.strMainData, 'ui16MaxSHdegree')) || ...
            isempty(strDynParams.strMainData.ui16MaxSHdegree)
        error('ResolveInertialDynMaxFidelityConfig:MissingSHDegree', ...
            'Non-empty dSHcoeff requires strMainData.ui16MaxSHdegree.');
    end

    ui32MaxSHdegree = uint32(strDynParams.strMainData.ui16MaxSHdegree);
    if ui32MaxSHdegree < uint32(2)
        error('ResolveInertialDynMaxFidelityConfig:InvalidSHDegree', ...
            'Non-empty dSHcoeff requires ui16MaxSHdegree >= 2.');
    end
end

% Static availability checks stay here so RHS/Jac use identical model structure decisions.
bHasPolyhedronGravity = bIncludePolyhedronGravity && ...
    coder.const(isfield(strDynParams.strMainData, 'strPolyhedronGravityData')) && ...
    ~isempty(strDynParams.strMainData.strPolyhedronGravityData);

bHasPanelSRP = bIncludeSRP && bUsePanelSRP && ...
    coder.const(isfield(strDynParams, 'strSCdata')) && ...
    coder.const(isfield(strDynParams.strSCdata, 'strSRPpanelData')) && ...
    ~isempty(strDynParams.strSCdata.strSRPpanelData);

bNeedMainAttitude = bHasSphericalHarmonicsData || bHasPolyhedronGravity;

% Construct typed structure
strModelConfig = struct();
coder.cstructname(strModelConfig,'strInertialDynModelConfig')

strModelConfig.bIncludeMainGravity = bIncludeMainGravity;
strModelConfig.bIncludeSphericalHarmonics = bIncludeSphericalHarmonics;
strModelConfig.bIncludeThirdBodies = bIncludeThirdBodies;
strModelConfig.bIncludeSunThirdBody = bIncludeSunThirdBody;
strModelConfig.bIncludeEarthThirdBody = bIncludeEarthThirdBody;
strModelConfig.bIncludeSRP = bIncludeSRP;
strModelConfig.bIncludeEclipse = bIncludeEclipse;
strModelConfig.bUsePanelSRP = bUsePanelSRP;
strModelConfig.bIncludePolyhedronGravity = bIncludePolyhedronGravity;
strModelConfig.bRecomputeSRPpressureFromDistance = bRecomputeSRPpressureFromDistance;
strModelConfig.bHasSphericalHarmonicsData = bHasSphericalHarmonicsData;
strModelConfig.ui32MaxSHdegree = ui32MaxSHdegree;
strModelConfig.bHasPolyhedronGravity = bHasPolyhedronGravity;
strModelConfig.bHasPanelSRP = bHasPanelSRP;
strModelConfig.bNeedMainAttitude = bNeedMainAttitude;


end

function bFlag = GetConfigFlag_(strModelConfigFlags, charFieldName, bDefault)
% Return optional model-configuration flag value or caller-provided default.
bFlag = bDefault;
if coder.const(isfield(strModelConfigFlags, charFieldName))
    bFlag = coder.const(logical(strModelConfigFlags.(charFieldName)));
end
end

