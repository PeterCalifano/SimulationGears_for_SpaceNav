function strReport = testNadirSunPointingWithJacobian(charBuildFolder)
%% SIGNATURE
% strReport = testNadirSunPointingWithJacobian(charBuildFolder)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate the shared pointing frame, its exact smooth-branch derivatives and
% explicit rejection of auxiliary-axis ties. Compile the public function with
% constant derivative selection and compare source/MEX outputs. No mission assets.
% Example: strReport = testNadirSunPointingWithJacobian(tempname);
% Output: A report with dMaximumDerivativeError < 2e-5 and dMexError < 1e-12.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charBuildFolder   Caller-owned disposable MEX directory.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strReport         Maximum raw derivative error and generated-code parity error.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-10-2026  Pietro Califano  Validate the shared analytical pointing contract.
% 10-10-2026  Pietro Califano  Follow the Jacobian interface and clarify validation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalNadirSunPointingWithJacobian, MATLAB Coder.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charBuildFolder (1, :) char
end

arguments (Output)
    strReport (1, 1) struct
end

% Keep the second case inside a collinear branch with a unique auxiliary axis.
dPositions = [[1.6; -0.3; 0.4], [1; 2; 3]];
dSunPositions = [[1e8; 2e7; -3e7], 1e8 * [1; 2; 3]];
dPerturbationSteps = [1e-6, 1e-10];
dMaximumDerivativeError = 0;
for dCaseIndex = 1:2
    dPosition = dPositions(:, dCaseIndex);
    dSunPosition = dSunPositions(:, dCaseIndex);
    [dRotation, dJacobian] = EvalNadirSunPointingWithJacobian(dPosition, dSunPosition, true);

    % Check frame handedness and the required +Z nadir direction.
    assert(norm(dRotation.' * dRotation - eye(3), 'fro') < 1e-13 && det(dRotation) > 0);
    assert(norm(dRotation(:, 3) + dPosition / norm(dPosition)) < 1e-13);

    % Recompute the complete frame for each finite-difference reference query.
    dReferenceJacobian = zeros(3, 3, 3);
    for dPositionAxis = 1:3
        dPositionPerturbation = zeros(3, 1);
        dPositionPerturbation(dPositionAxis) = dPerturbationSteps(dCaseIndex);
        dReferenceJacobian(:, :, dPositionAxis) = ...
            (EvalNadirSunPointingWithJacobian(dPosition + dPositionPerturbation, dSunPosition) - ...
             EvalNadirSunPointingWithJacobian(dPosition - dPositionPerturbation, dSunPosition)) / ...
            (2 * dPerturbationSteps(dCaseIndex));
    end

    dRelativeError = norm(dReferenceJacobian(:) - dJacobian(:)) / norm(dJacobian(:));
    assert(dRelativeError < 2e-5, 'testNadirSunPointingWithJacobian:Derivative', ...
        'Raw derivative error %.9g.', dRelativeError);
    dMaximumDerivativeError = max(dMaximumDerivativeError, dRelativeError);
end

% Allow a finite frame at an auxiliary-axis tie, but reject its derivative.
EvalNadirSunPointingWithJacobian([0; 0; 1], [0; 0; 1e8], false);
bRejected = false;
try
    EvalNadirSunPointingWithJacobian([0; 0; 1], [0; 0; 1e8], true);
catch objException
    bRejected = strcmp(objException.identifier, ...
        'EvalNadirSunPointingWithJacobian:UndefinedDerivative');
end
assert(bRejected, 'testNadirSunPointingWithJacobian:Boundary', ...
    'Reject an undefined auxiliary-axis derivative.');

% Compile the public numerical interface with constant derivative selection.
assert(license('test', 'MATLAB_Coder') == 1);
if ~isfolder(charBuildFolder)
    mkdir(charBuildFolder);
end
codegen('-config', coder.config('mex'), '-d', charBuildFolder, ...
    'EvalNadirSunPointingWithJacobian', ...
    '-args', {dPositions(:, 1), dSunPositions(:, 1), coder.Constant(true)});
addpath(charBuildFolder);

% Compare both generated outputs with the source implementation at the same query.
fcnPointingMex = str2func('EvalNadirSunPointingWithJacobian_mex');
[dSourceRotation, dSourceJacobian] = ...
    EvalNadirSunPointingWithJacobian(dPositions(:, 1), dSunPositions(:, 1), true);
[dMexRotation, dMexJacobian] = fcnPointingMex(dPositions(:, 1), dSunPositions(:, 1), true);
dMexError = max(norm(dSourceRotation - dMexRotation, 'fro'), ...
    norm(dSourceJacobian(:) - dMexJacobian(:)));
assert(dMexError < 1e-12);

strReport = struct('dMaximumDerivativeError', dMaximumDerivativeError, 'dMexError', dMexError);
fprintf('NADIR_POINTING_VALIDATED derivative_error=%.9g mex_error=%.9g\n', ...
    dMaximumDerivativeError, dMexError);
end
