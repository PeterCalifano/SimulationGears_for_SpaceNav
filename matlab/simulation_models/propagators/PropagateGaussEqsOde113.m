function [dKepTrajectory, dIntegratedTimegrid] = PropagateGaussEqsOde113(dKepState0, ...
                                                                    dRequestedTimegrid, ...
                                                                    dGravParam, ...
                                                                    dBodyRadius, ...
                                                                    dJ2coeff, ...
                                                                    dBcoeffPoly, ...
                                                                    dPerturbationType, ...
                                                                    kwargs)
%orbitpropagator_gauss Propagate Keplerian elements with Gauss planetary equations.
%
% The propagated state is [a; e; i; RAAN; omega; trueAnomaly]. The
% ballistic coefficient input follows `polyval` convention; pass a scalar
% for a constant coefficient or a vector of coefficients for a time-varying
% profile.
arguments (Input)
    dKepState0          {mustBeNumeric, mustBeFinite, mustBeVector, mustHaveNumElements_(dKepState0, 6)}
    dRequestedTimegrid  {mustBeNumeric, mustBeFinite, mustBeVector, mustHaveAtLeastTwoElements_(dRequestedTimegrid), mustBeStrictlyIncreasing_(dRequestedTimegrid)}
    dGravParam          (1,1) double {mustBeNumeric, mustBePositive}
    dBodyRadius         (1,1) double {mustBeNumeric, mustBePositive}
    dJ2coeff            (1,1) double {mustBeNumeric}
    dBcoeffPoly         {mustBeNumeric, mustBeFinite, mustBeVector, mustHaveAtLeastOneElement_(dBcoeffPoly)}
    dPerturbationType   (1,1) double {mustBeNumeric, mustBeInteger, mustBeMember(dPerturbationType, [0, 1, 2, 3])}
end
arguments (Input)
    kwargs.objOdeOpts = []
end
arguments (Output)
    dKepTrajectory      (:,6) double
    dIntegratedTimegrid (:,1) double
end
%% PROTOTYPE
% [dKepTrajectory, dIntegratedTimegrid] = orbitpropagator_gauss( ...
%     dKepState0, ...
%     dRequestedTimegrid, ...
%     dGravParam, ...
%     dBodyRadius, ...
%     dJ2coeff, ...
%     dBcoeffPoly, ...
%     dPerturbationType, ...
%     objOdeOpts=value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Integrates the Gauss planetary equations in Keplerian elements using
% `ode113` and `EvalRHS_GaussPlanetary`. The perturbation selector is:
%   0 --> Unperturbed (Keplerian motion only)
%   1 --> J2 perturbation only
%   2 --> J2 + atmospheric drag
%   3 --> Atmospheric drag only
% If `objOdeOpts` is not provided, the legacy tolerance heuristic is kept:
% scalar ballistic coefficient --> tighter tolerances, polynomial profile
% --> slightly looser tolerances.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dKepState0          [6x1] or [1x6] double   Initial Keplerian state [a; e; i; RAAN; omega; trueAnomaly] [km, rad]
% dRequestedTimegrid  vector double           Requested ODE evaluation time grid [s]
% dGravParam          (1,1) double            Central body gravitational parameter [km^3/s^2]
% dBodyRadius         (1,1) double            Central body reference radius [km]
% dJ2coeff            (1,1) double            J2 zonal harmonic coefficient [-]
% dBcoeffPoly         vector double           Ballistic coefficient scalar or polynomial coeffs in `polyval` format [m^2/kg]
% dPerturbationType   (1,1) integer           Perturbation type selector (0, 1, 2, 3)
% objOdeOpts          struct                  Optional ODE options structure created with `odeset`
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dKepTrajectory      [Nx6] double            Propagated Keplerian trajectory [km, rad]
% dIntegratedTimegrid [Nx1] double            Time grid returned by the integrator [s]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-01-2022    Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro Rizzo   Initial version.
% 22-04-2026    OpenAI Codex     Modernized header, naming, validation, and wrapper interface.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalRHS_GaussPlanetary()
% -------------------------------------------------------------------------------------------------------------

dKepState0 = dKepState0(:);
dRequestedTimegrid = dRequestedTimegrid(:);
dBcoeffPoly = dBcoeffPoly(:);
ui8PerturbationType = uint8(dPerturbationType);

objOdeOpts = resolveOdeOptions_(kwargs.objOdeOpts, dBcoeffPoly);

objDynamicsFcnHandle = @(dTime, dKepState) EvalRHS_GaussPlanetary( ...
    dTime, ...
    dKepState, ...
    dGravParam, ...
    dBodyRadius, ...
    dJ2coeff, ...
    dBcoeffPoly, ...
    ui8PerturbationType);

[dIntegratedTimegrid, dKepTrajectory] = ode113( ...
    objDynamicsFcnHandle, ...
    dRequestedTimegrid, ...
    dKepState0, ...
    objOdeOpts);

end

function objOdeOpts = resolveOdeOptions_(objUserOdeOpts, dBcoeffPoly)
if isempty(objUserOdeOpts)
    if numel(dBcoeffPoly) == 1
        objOdeOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
    else
        objOdeOpts = odeset('RelTol', 1e-11, 'AbsTol', 1e-12);
    end
    return;
end

if ~isstruct(objUserOdeOpts)
    error('orbitpropagator_gauss:InvalidOdeOptions', ...
        'objOdeOpts must be an ODE options struct created with odeset.');
end

objOdeOpts = objUserOdeOpts;
end

function mustHaveNumElements_(dValue, dExpectedCount)
if numel(dValue) ~= dExpectedCount
    error('orbitpropagator_gauss:InvalidStateSize', ...
        'The Keplerian state must contain exactly %d elements.', dExpectedCount);
end
end

function mustHaveAtLeastOneElement_(dValue)
if isempty(dValue)
    error('orbitpropagator_gauss:InvalidBallisticCoeff', ...
        'The ballistic coefficient input cannot be empty.');
end
end

function mustHaveAtLeastTwoElements_(dValue)
if numel(dValue) < 2
    error('orbitpropagator_gauss:InvalidTimegrid', ...
        'The time grid must contain at least two elements.');
end
end

function mustBeStrictlyIncreasing_(dValue)
if any(diff(dValue(:)) <= 0)
    error('orbitpropagator_gauss:InvalidTimegrid', ...
        'The time grid must be strictly increasing.');
end
end
