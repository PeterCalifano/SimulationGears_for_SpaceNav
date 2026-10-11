function [dDCM_INfromSCB, dJacDCMWrtPos] = EvalNadirSunPointingWithJacobian(dPosition_IN, ...
    dSunPosition_IN, bWithPartials) %#codegen
%% SIGNATURE
% [dDCM_INfromSCB, dJacDCMWrtPos] = ...
%     EvalNadirSunPointingWithJacobian(dPosition_IN, dSunPosition_IN, bWithPartials)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve ideal +Z nadir/+Y Sun-orthogonal pointing and its exact position derivative.
% Supply the frame and optional position Jacobian to guidance/design dynamics
% linearization. Reuse the attitude generator for the secondary-axis constraint;
% keep analytical linearization separate from its attitude-generation interface.
% Differentiate with respect to spacecraft position at fixed target and Sun epoch.
% Inside the collinear fallback region, hold the selected least-aligned auxiliary
% axis on its smooth branch. A derivative does not exist at a branch-selection
% boundary; reject such a request explicitly instead of returning a surrogate.
% Example: [dFrame, dJacobian] = ...
%     EvalNadirSunPointingWithJacobian([1; 0; 0], [1; 1; 0], true);
% Output: dFrame = [0, 0, -1; 1, 0, 0; 0, -1, 0], dJacobian has size [3, 3, 3].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosition_IN      Target-relative spacecraft position [LU].
% dSunPosition_IN   Target-relative Sun position [LU].
% bWithPartials     Compile-time request for the analytical frame derivative; default false.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDCM_INfromSCB    Body-to-inertial orthogonal frame.
% dJacDCMWrtPos     Three d(DCM)/d(position_i) slices [1/LU], or zero when not requested.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Share guidance pointing and exact position partials.
% 10-10-2026  Pietro Califano     Name the Jacobian interface and clarify computation blocks.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CAttitudePointingGenerator.DefineYaxisFromConstraint, skewSymm.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosition_IN (3, 1) double {mustBeFinite}
    dSunPosition_IN (3, 1) double {mustBeFinite}
    bWithPartials (1, 1) logical {coder.mustBeConst} = false
end

arguments (Output)
    dDCM_INfromSCB (3, 3) double
    dJacDCMWrtPos (3, 3, 3) double
end

% Resolve the two physical directions before selecting the roll constraint.
dPositionRange = norm(dPosition_IN);
dSpacecraftToSun = dSunPosition_IN - dPosition_IN;
dSunRange = norm(dSpacecraftToSun);
assert(dPositionRange > eps('single') && dSunRange > eps('single'), ...
    'EvalNadirSunPointingWithJacobian:InvalidGeometry', ...
    'Nadir and Sun directions must be nonsingular.');

% Compute the normalized nadir and Sun directions, and check for collinear fallback
dAxisZ = -dPosition_IN / dPositionRange;
dSunDirection = dSpacecraftToSun / dSunRange;
dSecondaryDirection = dSunDirection;
dCrossNorm = norm(cross(dAxisZ, dSunDirection));
bFallback = dCrossNorm <= 1e-8;

% Resolve collinear geometry with the least-aligned auxiliary axis.
% Hold that axis fixed when differentiating inside its selected branch.
if bFallback
    [~, dAuxiliaryAxisIndex] = min(abs(dAxisZ));
    dSecondaryDirection = zeros(3, 1);
    dSecondaryDirection(dAuxiliaryAxisIndex) = 1;
    dAxisY = CAttitudePointingGenerator.DefineYaxisFromConstraint(dPosition_IN, dAxisZ, ...
        dAuxiliaryAxis=dSecondaryDirection, ...
        enumConstraintType="auxiliaryAxis");
else
    dAxisY = CAttitudePointingGenerator.DefineYaxisFromConstraint(dPosition_IN, dAxisZ, ...
        dSunPosition_Frame=dSunPosition_IN, ...
        enumConstraintType="YorthogonalSun");
end

% Complete the right-handed frame and omit derivative work for frame-only calls.
dRawAxisX = cross(dAxisY, dAxisZ);
dAxisX = dRawAxisX / norm(dRawAxisX);
dDCM_INfromSCB = [dAxisX, dAxisY, dAxisZ];
dJacDCMWrtPos = zeros(3, 3, 3);
if ~coder.const(bWithPartials)
    return
end

% Reject branch boundaries where the selected frame has no smooth derivative.
assert(abs(dCrossNorm - 1e-8) > 32 * eps(max(dCrossNorm, 1e-8)), ...
    'EvalNadirSunPointingWithJacobian:UndefinedDerivative', ...
    'Sun/nadir fallback boundary has no smooth derivative.');
if bFallback
    dSortedAxisMagnitudes = sort(abs(dAxisZ));
    assert(dSortedAxisMagnitudes(2) - dSortedAxisMagnitudes(1) > 32 * eps(1.0), ...
        'EvalNadirSunPointingWithJacobian:UndefinedDerivative', ...
        'Auxiliary-axis selection tie has no smooth derivative.');
end

% Differentiate normalized nadir and Sun directions at fixed Sun position.
dJacAxisZ = -(eye(3) - dAxisZ * dAxisZ.') / dPositionRange;
dJacSecondary = zeros(3, 3);
if ~bFallback
    dJacSecondary = -(eye(3) - dSunDirection * dSunDirection.') / dSunRange;
end

% Propagate both direction derivatives through the normalized cross products.
dRawAxisY = cross(dAxisZ, dSecondaryDirection);
dJacRawAxisY = -skewSymm(dSecondaryDirection) * dJacAxisZ + ...
    skewSymm(dAxisZ) * dJacSecondary;
dJacAxisY = (eye(3) - dAxisY * dAxisY.') * dJacRawAxisY / norm(dRawAxisY);

dJacRawAxisX = -skewSymm(dAxisZ) * dJacAxisY + skewSymm(dAxisY) * dJacAxisZ;
dJacAxisX = (eye(3) - dAxisX * dAxisX.') * dJacRawAxisX / norm(dRawAxisX);

% Store one frame-derivative slice for each spacecraft position component.
for ui32PositionAxis = uint32(1):uint32(3)
    dJacDCMWrtPos(:, :, ui32PositionAxis) = ...
        [dJacAxisX(:, ui32PositionAxis), dJacAxisY(:, ui32PositionAxis), ...
         dJacAxisZ(:, ui32PositionAxis)];
end
end
