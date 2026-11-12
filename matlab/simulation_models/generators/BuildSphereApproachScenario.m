function strScenario = BuildSphereApproachScenario(dTime_s, dInitialPosition_IN, dInitialVelocity_IN, options)
%% SIGNATURE
% strScenario = BuildSphereApproachScenario(dTime_s, dInitialPosition_IN, dInitialVelocity_IN, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build the deterministic truth of a camera-carrying spacecraft approaching a spherical target, on any time grid
% that starts at the scenario epoch. The function holds no random draw: the caller passes the (possibly
% dispersed) initial state, so every random stream stays with the caller. The scene is the one of the
% image-motion synthetic recordings (nav-backend GenerateImageMotionRecordingSet, sets r107 and later) and of
% the nav-frontend emulator evaluation (EvaluateFrontEndEmulator); both call this function.
%
% Target   sphere of radius dTargetRadius_m centred at the inertial origin, represented by the convex hull of
%          ui32NumMeshPoints Fibonacci-lattice points. Vertices are the landmarks, vertex normals are radial.
%          The triangle list is one-based, outward-facing as returned by convhull, and the prepared BVH ray data
%          of BuildTriangleRayData is attached for LaserRangefinderModel.
%          Rotation: BuildTargetRotationalStateHistory, KINEMATIC_CONST_INERTIAL_RATE, initial quaternion
%          [1 0 0 0] (T coincides with I at the epoch) and inertial rate dTargetAngVel_IN.
% Motion   two-body motion about the target centre with gravitational parameter dGravParam_m3_s2, evaluated on
%          the grid by the closed-form MathCore Kepler solution: rv2kepl gives the classical elements of the
%          initial state and PropagateKeplerianElems (anomaly tolerance 1e-12 rad, bConvert2Cart) returns the
%          Cartesian state at each grid time in one Kepler step from the epoch. The state at a time therefore
%          does not depend on the other grid times: a dense grid (gyro edges) reproduces a coarse grid (truth
%          samples) bit for bit at the common times, and no step error accumulates along the grid (with the
%          former sequential propagation the 10 ms and the 1 s grids of the default arc differed by 3.8e-10 m
%          within 30 s).
%          An anomaly error delta is a time error delta |r| / sqrt(GM / |a|) for every conic (the eccentric and
%          hyperbolic anomalies advance at sqrt(GM / |a|) / |r|): about 1.5e-9 s on the default approach arc,
%          i.e. about 3e-9 m in position. dGravParam_m3_s2 = 0 gives the straight line of constant velocity.
% Attitude look-at: the camera boresight z_C points from the body origin to the target centre and
%          x_C = normalise(dUpReference_IN x z_C), y_C = z_C x x_C. A pointing slew is appended in C:
%          R_IC(tau) = R_IC_lookat(tau) Exp(s(tau) phi_C), with phi_C = dPointingSlewRotation_C and the cosine
%          profile s(tau) = 0 for tau <= tau_start, (1 - cos(pi w)) / 2 with w = (tau - tau_start) /
%          (tau_end - tau_start) inside the interval, and 1 for tau >= tau_end. This is the STEP profile of the
%          Python synthetic generator (synthetic_sphere_descent.PointingSlew). At s = 1 the factor is
%          RotationVectorToDCM(phi_C) itself, so a grid that skips the slew interval sees the instantaneous step
%          of the r107 recordings bit for bit. Attitude control is ideal: the commanded attitude is the true one.
%
% Design notes. The scenario is analytic and has no asset files, so it is not registered in CScenarioRegistry.
% CAttitudePointingGenerator ("auxiliaryAxis") builds the in-plane camera axes from the auxiliary direction with
% a different rule, so it would not reproduce the existing recordings; the look-at rule above is kept instead.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTime_s                          (1,N) double strictly increasing grid times from the epoch [s], N >= 2,
%                                               dTime_s(1) = 0 (the epoch of the initial state and of T = I)
% dInitialPosition_IN              (3,1) double body position at the epoch in I [m]
% dInitialVelocity_IN              (3,1) double body velocity at the epoch in I [m/s]
% options.dTargetRadius_m          (1,1) double sphere radius [m], > 0
% options.ui32NumMeshPoints        (1,1) uint32 mesh vertices (= landmarks), >= 20
% options.dTargetAngVel_IN         (3,1) double target inertial angular velocity [rad/s]
% options.dGravParam_m3_s2         (1,1) double gravitational parameter [m^3/s^2], >= 0 (0 = straight line)
% options.dUpReference_IN          (3,1) double reference direction of the camera x axis (x_C = up x z_C)
% options.dPointingSlewRotation_C  (3,1) double total slew rotation vector phi_C in C [rad]
% options.dPointingSlewInterval_s  (:,1) double [tau_start; tau_end] of the slew from the epoch [s],
%                                               tau_end > tau_start; empty = no slew (phi_C must then be 0)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strScenario (1,1) struct
%   .dTime_s               (1,N) grid times from the epoch [s] (the input)
%   .dPosition_IN          (3,N) body position in I [m]
%   .dVelocity_IN          (3,N) body velocity in I [m/s]
%   .dDCM_INfromCAM        (3,3,N) camera attitude R_IC (commanded = true)
%   .dPointingSlewFraction (1,N) slew fraction s(tau) in [0, 1]
%   .strTargetRotation     BuildTargetRotationalStateHistory output on the grid (dDCM_INfromTB, rates, ...)
%   .strTargetMesh         (1,1) struct LaserRangefinderModel mesh: .dVerticesPositions (3,M) [m] in T,
%                          .i32triangVertexPtrs (3,F) one-based, .strRayData (BuildTriangleRayData)
%   .dVertexNormals_TB     (3,M) outward unit normals of the vertices in T
%   .strDefinition         (1,1) struct echo of the scene parameters and rule descriptions (for manifests)
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-10-2026  Pietro Califano, Claude Opus 5.5  First implementation, moved from the local functions of
%                                               nav-backend GenerateImageMotionRecordingSet (image-motion task
%                                               #47); cosine pointing slew added for dense grids.
% 29-10-2026  Pietro Califano, Claude Opus 5.5  Each grid time propagated from the epoch (grid-independent
%                                               state, task #47 dense gyro grid).
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; loop counters typed uint32.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTargetRotationalStateHistory, STargetRotationalStateConfig, EnumTargetRotationalStateSource,
% EnumTargetRotationalDynamicsModel, BuildTriangleRayData; MathCore: rv2kepl, PropagateKeplerianElems,
% RotationVectorToDCM.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dTime_s (1,:) double {mustBeReal, mustBeFinite}
    dInitialPosition_IN (3,1) double {mustBeReal, mustBeFinite}
    dInitialVelocity_IN (3,1) double {mustBeReal, mustBeFinite}
    options.dTargetRadius_m (1,1) double {mustBePositive, mustBeFinite} = 500
    options.ui32NumMeshPoints (1,1) uint32 {mustBeGreaterThanOrEqual(options.ui32NumMeshPoints, 20)} = 400
    options.dTargetAngVel_IN (3,1) double {mustBeReal, mustBeFinite} = [0; 0; 2e-4]
    options.dGravParam_m3_s2 (1,1) double {mustBeNonnegative, mustBeFinite} = 69.89
    options.dUpReference_IN (3,1) double {mustBeReal, mustBeFinite} = [1; 0; 0]
    options.dPointingSlewRotation_C (3,1) double {mustBeReal, mustBeFinite} = [0; 0; 0]
    options.dPointingSlewInterval_s (:,1) double {mustBeReal} = zeros(0, 1)
end
arguments (Output)
    strScenario (1,1) struct
end

% The epoch anchors the initial state and the target attitude T = I, so the grid must start there.
if numel(dTime_s) < 2 || dTime_s(1) ~= 0 || any(diff(dTime_s) <= 0)
    error('BuildSphereApproachScenario:InvalidTimeGrid', ...
        ['dTime_s must hold at least two strictly ', ...
        'increasing times starting at the epoch (dTime_s(1) = 0).']);
end

% The slew interval is empty or two finite increasing times, and a nonzero slew needs one
if ~isempty(options.dPointingSlewInterval_s) && (numel(options.dPointingSlewInterval_s) ~= 2 || ...
        any(~isfinite(options.dPointingSlewInterval_s)) || ...
        options.dPointingSlewInterval_s(2) <= options.dPointingSlewInterval_s(1))
    error('BuildSphereApproachScenario:InvalidSlewInterval', ...
        ['dPointingSlewInterval_s must be empty or ', ...
        'two finite times [tau_start; tau_end] with tau_end > tau_start.']);
end
if isempty(options.dPointingSlewInterval_s) && any(options.dPointingSlewRotation_C ~= 0)
    error('BuildSphereApproachScenario:MissingSlewInterval', ...
        ['A nonzero dPointingSlewRotation_C needs ', ...
        'dPointingSlewInterval_s.']);
end
CheckDependencies_();

% Target: mesh, landmark normals and rotation history.
[~, dVertexNormals_TB, strTargetMesh] = BuildSphereMesh_(options.dTargetRadius_m, options.ui32NumMeshPoints);
objRotationConfig = STargetRotationalStateConfig();
objRotationConfig.enumSource = EnumTargetRotationalStateSource.GENERATED;
objRotationConfig.enumDynamicsModel = EnumTargetRotationalDynamicsModel.KINEMATIC_CONST_INERTIAL_RATE;
objRotationConfig.dInitialQuat_INfromTB = [1; 0; 0; 0];
objRotationConfig.dInitialAngVel_IN = options.dTargetAngVel_IN;
strTargetRotation = BuildTargetRotationalStateHistory(objRotationConfig, dTime_s);

% Body motion and camera attitude.
[dPosition_IN, dVelocity_IN] = PropagateTwoBodyTruth_(dInitialPosition_IN, ...
    dInitialVelocity_IN, ...
    options.dGravParam_m3_s2, ...
    dTime_s);
dPointingSlewFraction = ComputeCosineSlewFraction_(dTime_s, options.dPointingSlewInterval_s);
dDCM_INfromCAM = BuildCommandedCameraAttitude_(dPosition_IN, ...
    options.dUpReference_IN, ...
    options.dPointingSlewRotation_C, ...
    dPointingSlewFraction);

% Echo the scene parameters and the rules for manifests
strDefinition = struct('target_radius_m', options.dTargetRadius_m, ...
    'num_mesh_points', options.ui32NumMeshPoints, ...
    'num_mesh_triangles', size(strTargetMesh.i32triangVertexPtrs, 2), ...
    'target_angular_velocity_IN_rad_s', options.dTargetAngVel_IN, ...
    'gravitational_parameter_m3_s2', options.dGravParam_m3_s2, ...
    'up_reference_IN', options.dUpReference_IN, ...
    'pointing_slew_rotation_C_rad', options.dPointingSlewRotation_C, ...
    'pointing_slew_interval_s', options.dPointingSlewInterval_s, ...
    'mesh_rule', ['convex hull of Fibonacci-lattice points on the sphere; vertices are landmarks with ', ...
        'radial normals'], ...
    'dynamics_rule', ['two-body motion about the target centre, closed-form Kepler solution (MathCore ', ...
        'PropagateKeplerianElems, anomaly tolerance 1e-12 rad; 0 = straight line)'], ...
    'attitude_rule', ['look-at: z_C toward the target centre, x_C = normalise(up x z_C); slew ', ...
        'R_IC = R_IC_lookat Exp(s phi_C), s = (1 - cos(pi w)) / 2 over the slew interval'], ...
    'target_rotation_rule', ['BuildTargetRotationalStateHistory KINEMATIC_CONST_INERTIAL_RATE, initial ', ...
        'quaternion [1 0 0 0] (T = I at the epoch)']);

strScenario = struct('dTime_s', dTime_s, ...
    'dPosition_IN', dPosition_IN, ...
    'dVelocity_IN', dVelocity_IN, ...
    'dDCM_INfromCAM', dDCM_INfromCAM, ...
    'dPointingSlewFraction', dPointingSlewFraction, ...
    'strTargetRotation', strTargetRotation, ...
    'strTargetMesh', strTargetMesh, ...
    'dVertexNormals_TB', dVertexNormals_TB, ...
    'strDefinition', strDefinition);
end

%% Local functions
function CheckDependencies_()
%% SIGNATURE
% CheckDependencies_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Raise a named error listing the functions missing from the MATLAB path.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------

cellRequired = {'BuildTargetRotationalStateHistory', ...
    'BuildTriangleRayData', ...
    'RotationVectorToDCM', ...
    'rv2kepl', ...
    'PropagateKeplerianElems'};
bMissing = cellfun(@(charName) exist(charName, 'file') == 0, cellRequired);
if any(bMissing)
    error('BuildSphereApproachScenario:MissingDependency', ...
        'Not on the MATLAB path: %s.', ...
        strjoin(cellRequired(bMissing), ', '));
end
end

function [dVertices, dNormals, strMesh] = BuildSphereMesh_(dRadius, ui32Count)
%% SIGNATURE
% [dVertices, dNormals, strMesh] = BuildSphereMesh_(dRadius, ui32Count)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Fibonacci-lattice vertices on the sphere, radial normals and the convex-hull triangle mesh (one-based), with
% the prepared BVH ray data of BuildTriangleRayData (built once, reused by every lidar shot).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRadius     Sphere radius [m].
% ui32Count   Number of vertices.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVertices   (3,M) vertex positions in T [m].
% dNormals    (3,M) outward unit normals.
% strMesh     LaserRangefinderModel mesh with the prepared ray data.
% -------------------------------------------------------------------------------------------------------------

% Place the vertices on a Fibonacci lattice
dIndex = (0:double(ui32Count) - 1) + 0.5;
dPolar = acos(1 - 2 * dIndex / double(ui32Count));
dAzimuth = pi * (1 + sqrt(5)) * dIndex;
dNormals = [cos(dAzimuth) .* sin(dPolar); sin(dAzimuth) .* sin(dPolar); cos(dPolar)];
dVertices = dRadius * dNormals;

% Triangulate by convex hull and prepare the ray data
dTriangleVertexIdx = convhull(dVertices(1, :)', dVertices(2, :)', dVertices(3, :)');
dFaceVertices = reshape(dVertices(:, dTriangleVertexIdx'), 3, 3, []);   % (:, j, n) = vertex j of triangle n
strMesh = struct('dVerticesPositions', dVertices, ...
    'i32triangVertexPtrs', int32(dTriangleVertexIdx'), ...
    'strRayData', BuildTriangleRayData(dFaceVertices, true));
end

function [dPosition_IN, dVelocity_IN] = PropagateTwoBodyTruth_(dPosition0_IN, dVelocity0_IN, dGravParam, dTime_s)
%% SIGNATURE
% [dPosition_IN, dVelocity_IN] = PropagateTwoBodyTruth_(dPosition0_IN, dVelocity0_IN, dGravParam, dTime_s)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Closed-form two-body state at every grid time, each in one Kepler step from the epoch elements (scalar
% horizon of PropagateKeplerianElems), so the state at a time does not depend on the rest of the grid; see the
% DESCRIPTION of the main function for the accuracy of the anomaly tolerance. The MathCore propagator was fixed
% on 2026-10-28 (Kepler's equation took the true anomaly for the eccentric or hyperbolic anomaly) and MathCore
% testPropagateKeplerianElems checks it against ode113 (image-motion decisions log R106, R107).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosition0_IN   (3,1) initial position in I [m].
% dVelocity0_IN   (3,1) initial velocity in I [m/s].
% dGravParam      Gravitational parameter [m^3/s^2]; 0 gives the straight line.
% dTime_s         (1,N) grid times from the epoch [s].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPosition_IN    (3,N) position in I [m].
% dVelocity_IN    (3,N) velocity in I [m/s].
% -------------------------------------------------------------------------------------------------------------

% Without gravity the body moves on a straight line
dKeplerAnomalyTol_rad = 1e-12;
if dGravParam == 0
    dPosition_IN = dPosition0_IN + dVelocity0_IN * dTime_s;
    dVelocity_IN = repmat(dVelocity0_IN, 1, numel(dTime_s));
    return
end

% Propagate the epoch elements to each grid time
dKeplerElements0 = rv2kepl([dPosition0_IN; dVelocity0_IN], dGravParam, 'rad');
dStates = zeros(6, numel(dTime_s));
for ui32TimeIdx = uint32(1):uint32(numel(dTime_s))
    % A scalar horizon returns [initial, final] states, or the initial state alone for a zero horizon.
    dStatesToTime = PropagateKeplerianElems(dKeplerElements0, ...
        dGravParam, ...
        dTime_s(ui32TimeIdx), ...
        'bConvert2Cart', true, ...
        'dRadAngleTol', dKeplerAnomalyTol_rad);
    dStates(:, ui32TimeIdx) = dStatesToTime(:, end);
end
dPosition_IN = dStates(1:3, :);
dVelocity_IN = dStates(4:6, :);
end

function dFraction = ComputeCosineSlewFraction_(dTime_s, dInterval_s)
%% SIGNATURE
% dFraction = ComputeCosineSlewFraction_(dTime_s, dInterval_s)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Cosine slew fraction s(tau): 0 up to tau_start, (1 - cos(pi w)) / 2 inside, exactly 1 from tau_end on.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTime_s       (1,N) grid times from the epoch [s].
% dInterval_s   [tau_start; tau_end] [s], or empty for no slew.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dFraction     (1,N) slew fraction in [0, 1].
% -------------------------------------------------------------------------------------------------------------

dFraction = zeros(size(dTime_s));
if isempty(dInterval_s)
    return
end

dProgress = (dTime_s - dInterval_s(1)) / (dInterval_s(2) - dInterval_s(1));
bInside = dTime_s > dInterval_s(1) & dTime_s < dInterval_s(2);
dFraction(bInside) = 0.5 * (1 - cos(pi * dProgress(bInside)));
dFraction(dTime_s >= dInterval_s(2)) = 1;
end

function dDCM_IC = BuildCommandedCameraAttitude_(dPosition_IN, dUpReference_IN, dSlewRotation_C, dSlewFraction)
%% SIGNATURE
% dDCM_IC = BuildCommandedCameraAttitude_(dPosition_IN, dUpReference_IN, dSlewRotation_C, dSlewFraction)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Look-at camera attitude toward the inertial origin with the slew Exp(s phi_C) appended in C. At s = 1 the
% factor is the precomputed RotationVectorToDCM(phi_C), which keeps grids that skip the slew bit-identical to
% the instantaneous step.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosition_IN      (3,N) body position in I [m].
% dUpReference_IN   (3,1) reference direction of the camera x axis.
% dSlewRotation_C   (3,1) total slew rotation vector in C [rad].
% dSlewFraction     (1,N) slew fraction at each grid time.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDCM_IC           (3,3,N) camera attitude R_IC.
% -------------------------------------------------------------------------------------------------------------

dNumTimes = size(dPosition_IN, 2);
dDCM_IC = zeros(3, 3, dNumTimes);
dStepDCM = RotationVectorToDCM(dSlewRotation_C);
for ui32TimeIdx = uint32(1):uint32(dNumTimes)

    % Point z_C at the target centre and build x_C from the up reference
    dBoresight_IN = -dPosition_IN(:, ui32TimeIdx) / norm(dPosition_IN(:, ui32TimeIdx));
    dRight_IN = cross(dUpReference_IN, dBoresight_IN);
    if norm(dRight_IN) < 1.0e-3 * norm(dUpReference_IN)
        error('BuildSphereApproachScenario:DegenerateLookAt', ...
            'dUpReference_IN is within 0.06 deg of the boresight at grid index %d.', ui32TimeIdx);
    end
    dRight_IN = dRight_IN / norm(dRight_IN);
    dDCM_IC(:, :, ui32TimeIdx) = [dRight_IN, cross(dBoresight_IN, dRight_IN), dBoresight_IN];

    % Append the slew rotation in C
    if dSlewFraction(ui32TimeIdx) == 1
        dDCM_IC(:, :, ui32TimeIdx) = dDCM_IC(:, :, ui32TimeIdx) * dStepDCM;
    elseif dSlewFraction(ui32TimeIdx) > 0
        dDCM_IC(:, :, ui32TimeIdx) = dDCM_IC(:, :, ui32TimeIdx) * ...
            RotationVectorToDCM(dSlewFraction(ui32TimeIdx) * dSlewRotation_C);
    end
end
end
