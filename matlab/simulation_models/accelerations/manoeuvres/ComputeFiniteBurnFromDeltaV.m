function strBurnData = ComputeFiniteBurnFromDeltaV(dDeltaV_IN, ...
                                                   dInitialMass, ...
                                                   dThrust, ...
                                                   dIsp, ...
                                                   kwargs)
arguments
    dDeltaV_IN    (3,1) double {mustBeFinite}
    dInitialMass  (1,1) double {mustBePositive}
    dThrust       (1,1) double {mustBeNonnegative}
    dIsp          (1,1) double {mustBePositive}
end
arguments
    kwargs.dStartTime       (1,1) double {mustBeFinite} = 0.0
    kwargs.charLengthUnits  {mustBeA(kwargs.charLengthUnits, ["string", "char", "EnumLengthUnits"])} = EnumLengthUnits.km
end
%% PROTOTYPE
% strBurnData = ComputeFiniteBurnFromDeltaV(dDeltaV_IN, dInitialMass, dThrust, dIsp, kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build a deterministic finite-burn profile that realizes a requested inertial DeltaV with constant thrust and
% constant Isp. dDeltaV_IN is expressed in the requested length units per second, while thrust, mass, and Isp use
% SI propulsion units. The returned acceleration fields use the same length unit as dDeltaV_IN.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dDeltaV_IN:             (3,1) double   Requested inertial DeltaV [charLengthUnits/s].
% dInitialMass:           (1,1) double   Initial spacecraft mass [kg].
% dThrust:                (1,1) double   Constant thrust magnitude [N].
% dIsp:                   (1,1) double   Specific impulse [s].
% kwargs.dStartTime:      (1,1) double   Burn start time [s].
% kwargs.charLengthUnits: [1]            Length units for DeltaV and output acceleration ('m' or 'km').
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strBurnData:            (1,1) struct   Finite-burn profile consumed by EvalFiniteBurnAccel().
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Add deterministic finite-burn profile builder.
% 02-07-2026    Pietro Califano, Codex 5.5      Generalize DeltaV and acceleration units through EnumLengthUnits.
% 18-07-2026    Pietro Califano, Codex 5.5      Permit zero thrust for zero-DeltaV profiles while requiring positive thrust for active burns.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EnumLengthUnits
% -------------------------------------------------------------------------------------------------------------

%% Function code
dG0 = 9.80665; % [m/s^2]
charLengthUnits = char(EnumLengthUnits.toString(kwargs.charLengthUnits));
dLengthUnitInMeters = coder.const(ResolveLengthUnitInMeters_(charLengthUnits));

dDeltaVNorm = norm(dDeltaV_IN);

if dDeltaVNorm < eps('single')
    % No burn is required; keep a zero-duration profile with no propellant consumption.
    dBurnDirection_IN = zeros(3,1);
    dFinalMass = dInitialMass;
    dPropellantMass = 0.0;
    dMassFlowRate = 0.0;
    dBurnDuration = 0.0;
else
    assert(dThrust > 0.0, ...
        'ComputeFiniteBurnFromDeltaV:InvalidThrust', ...
        'Nonzero DeltaV finite burns require strictly positive thrust.');

    % Convert DeltaV to m/s for the rocket equation, while preserving the
    % caller-facing direction and magnitude in the selected length unit.
    dBurnDirection_IN = dDeltaV_IN ./ dDeltaVNorm;
    dDeltaVNormMsec = dLengthUnitInMeters * dDeltaVNorm;

    % Constant-thrust mass flow comes from SI propulsion units. Burn duration
    % is the propellant mass divided by the constant mass-flow rate.
    dFinalMass = dInitialMass * exp(-dDeltaVNormMsec / (dIsp * dG0));
    dPropellantMass = dInitialMass - dFinalMass;
    dMassFlowRate = dThrust / (dIsp * dG0);
    dBurnDuration = dPropellantMass / dMassFlowRate;
end

dEndTime = kwargs.dStartTime + dBurnDuration;

strBurnData = struct();
strBurnData.dStartTime = kwargs.dStartTime;
strBurnData.dEndTime = dEndTime;
strBurnData.dBurnDuration = dBurnDuration;
strBurnData.dDeltaV_IN = dDeltaV_IN;
strBurnData.dDeltaVNorm = dDeltaVNorm;
strBurnData.dBurnDirection_IN = dBurnDirection_IN;
strBurnData.charLengthUnits = charLengthUnits;
strBurnData.dLengthUnitInMeters = dLengthUnitInMeters;
strBurnData.dInitialMass = dInitialMass;
strBurnData.dFinalMass = dFinalMass;
strBurnData.dPropellantMass = dPropellantMass;
strBurnData.dMassFlowRate = dMassFlowRate;
strBurnData.dThrust = dThrust;
strBurnData.dIsp = dIsp;
strBurnData.dThrustAccelStart_IN = dBurnDirection_IN .* (dThrust / dInitialMass / dLengthUnitInMeters);
end

function dLengthUnitInMeters = ResolveLengthUnitInMeters_(charLengthUnits)
% Return the number of SI meters represented by one selected length unit.
switch EnumLengthUnits.toString(charLengthUnits)
    case "m"
        dLengthUnitInMeters = 1.0;
    case "km"
        dLengthUnitInMeters = 1000.0;
    otherwise
        error('ComputeFiniteBurnFromDeltaV:UnsupportedLengthUnit', ...
            'Unsupported length unit "%s".', char(charLengthUnits));
end
end
