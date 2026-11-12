function [dQuaternionHistory_TBfromIN, dAngVelHistory_TB] = RunTorqueFreeBodyHistory(dTimegrid, ...
    dInitialQuaternion_TBfromIN, ...
    dInitialAngVel_TB, ...
    dInertiaTensor_TB, ...
    dMaxStep, ...
    kwargs)
%% SIGNATURE
% [dQuaternionHistory_TBfromIN, dAngVelHistory_TB] = RunTorqueFreeBodyHistory(dTimegrid, ...
%     dInitialQuaternion_TBfromIN, dInitialAngVel_TB, dInertiaTensor_TB, dMaxStep, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Execute the whole-history torque-free rigid-body propagator through the explicitly selected backend. MATLAB
% mode is the artifact-free default. MEX mode builds or reuses one source-fingerprinted profile, resolves that
% exact binary, and never silently falls back after a requested build or load failure.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTimegrid                    (1,N) double strictly increasing output epochs [s]
% dInitialQuaternion_TBfromIN  (4,1) double scalar-first Hamilton/passive quaternion
% dInitialAngVel_TB            (3,1) double initial body-frame angular velocity [rad/s]
% dInertiaTensor_TB            (3,3) double physical center-of-mass inertia tensor
% dMaxStep                     (1,1) double maximum internal integration step [s]
% kwargs.enumBackend           (1,1) EnumTargetRotationalStateBackend explicit MATLAB/MEX selection
% kwargs.charMexOutputRoot     (1,:) char optional generated-artifact root used only for MEX
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dQuaternionHistory_TBfromIN  (4,N) double normalized, sign-continuous quaternion history
% dAngVelHistory_TB            (3,N) double body-frame angular-velocity history [rad/s]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     Select whole-history MEX with MATLAB fallback.
% 24-08-2026  Pietro Califano, Codex     Replace path-presence dispatch with explicit build/reuse policy.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PropagateTorqueFreeBodyHistory, BuildMexTargets_TargetRotation,
% EnumTargetRotationalStateBackend, PropagateTorqueFreeBodyHistory_mex (optional)
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dTimegrid (1,:) double {mustBeReal, mustBeFinite}
    dInitialQuaternion_TBfromIN (4,1) double {mustBeReal, mustBeFinite}
    dInitialAngVel_TB (3,1) double {mustBeReal, mustBeFinite}
    dInertiaTensor_TB (3,3) double {mustBeReal, mustBeFinite}
    dMaxStep (1,1) double {mustBePositive, mustBeFinite}
    kwargs.enumBackend (1,1) EnumTargetRotationalStateBackend = EnumTargetRotationalStateBackend.MATLAB
    kwargs.charMexOutputRoot (1,:) char = ''
end
arguments (Output)
    dQuaternionHistory_TBfromIN (4,:) double
    dAngVelHistory_TB (3,:) double
end

switch kwargs.enumBackend
    case EnumTargetRotationalStateBackend.MATLAB
        % Keep reference execution independent from code-generation tooling, output directories, and any MEX
        % already present on the path.
        [dQuaternionHistory_TBfromIN, dAngVelHistory_TB] = PropagateTorqueFreeBodyHistory(dTimegrid, ...
            dInitialQuaternion_TBfromIN, ...
            dInitialAngVel_TB, ...
            dInertiaTensor_TB, ...
            dMaxStep);

    case EnumTargetRotationalStateBackend.MEX
        % Build or reuse one exact source profile only after generated execution has been explicitly selected.
        try
            strMexInfo = BuildMexTargets_TargetRotation(kwargs.charMexOutputRoot);
        catch objBuildException
            objDiagnostic = MException('RunTorqueFreeBodyHistory:MexBuildFailed', ...
                ['Failed to build the requested target rotational-state ', ...
                'MEX profile. No MATLAB fallback was selected.']);
            objDiagnostic = addCause(objDiagnostic, objBuildException);
            throwAsCaller(objDiagnostic);
        end

        % Resolve and invoke the exact artifact returned by the builder; another same-named binary elsewhere
        % on the path is never accepted.
        cellPathEntries = strsplit(path, pathsep);
        bArtifactPathWasPresent = any(strcmp(cellPathEntries, strMexInfo.charArtifactDir));
        try
            addpath(strMexInfo.charArtifactDir, '-begin');
            rehash path;
            charTargetFunctionName = strMexInfo.charTargetFunctionName;
            clear(charTargetFunctionName);
            charResolvedTargetPath = which(charTargetFunctionName);

            if ~strcmp(CanonicalPath_(charResolvedTargetPath), CanonicalPath_(strMexInfo.charArtifactPath))
                error('RunTorqueFreeBodyHistory:MexResolutionMismatch', ...
                    ['Generated target %s resolved to %s instead of the ', ...
                    'validated artifact %s.'], ...
                    charTargetFunctionName, ...
                    charResolvedTargetPath, ...
                    strMexInfo.charArtifactPath);
            end

            handleGeneratedHistory = str2func(charTargetFunctionName);
            [dQuaternionHistory_TBfromIN, dAngVelHistory_TB] = handleGeneratedHistory(dTimegrid, ...
                dInitialQuaternion_TBfromIN, ...
                dInitialAngVel_TB, ...
                dInertiaTensor_TB, ...
                dMaxStep);

        catch objLoadException
            % Remove the artifact folder from the path again if this call added it
            if ~bArtifactPathWasPresent && any(strcmp(strsplit(path, pathsep), strMexInfo.charArtifactDir))
                rmpath(strMexInfo.charArtifactDir);
            end

            objDiagnostic = MException('RunTorqueFreeBodyHistory:MexLoadFailed', ...
                ['Failed to load or invoke the requested target rotational-', ...
                'state MEX profile from %s. No MATLAB fallback was selected.'], ...
                strMexInfo.charArtifactPath);
            objDiagnostic = addCause(objDiagnostic, objLoadException);
            throwAsCaller(objDiagnostic);
        end

    otherwise
        error('RunTorqueFreeBodyHistory:InvalidBackend', ...
            'Unsupported target rotational-state backend.');
end

end

function charCanonicalPath = CanonicalPath_(charInputPath)
%% SIGNATURE
% charCanonicalPath = CanonicalPath_(charInputPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Normalize an artifact path before exact generated-target comparison. An empty path stays empty.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charInputPath       Path to normalize.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charCanonicalPath   Canonical absolute path, or '' for an empty input.
% -------------------------------------------------------------------------------------------------------------

if isempty(charInputPath)
    charCanonicalPath = '';
    return
end

charCanonicalPath = char(java.io.File(charInputPath).getCanonicalPath());
end
