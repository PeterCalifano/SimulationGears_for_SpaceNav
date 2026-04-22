classdef testPropagateClohessyWiltshireAnalytical < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for PropagateClohessyWiltshireAnalytical (Clohessy-Wiltshire analytical solution).
    % Tests initial condition recovery, ODE consistency, drift properties, and randomized checks.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth = 398600.4418;
        dSmaLEO = 6871.0;
    end

    methods (Test)

        function testInitialConditionRecovery(testCase)
            % At t=0, the solution must return exactly the initial state
            dMeanAngRate = sqrt(testCase.dMuEarth / testCase.dSmaLEO^3);
            dInitState = [1; 0.5; -0.2; 0.001; -0.002; 0.0005];

            dResult = PropagateClohessyWiltshireAnalytical(0, dMeanAngRate, dInitState);
            testCase.verifyEqual(dResult, dInitState, 'AbsTol', 1e-14, ...
                'CW solution at t=0 must equal the initial state');
        end

        function testPeriodicRelativeOrbit(testCase)
            % A bounded CW orbit satisfies vy0 = -2*n*x0 and vx0 = 0 with z-decoupled
            % After one orbital period, it should return to the initial state
            dMeanAngRate = sqrt(testCase.dMuEarth / testCase.dSmaLEO^3);
            dPeriod = 2 * pi / dMeanAngRate;

            % Periodic condition: vx0 = 0, vy0 = -2*n*x0 (no secular drift)
            dX0 = 0.5; % km
            dInitState = [dX0; 0; 0.1; 0; -2*dMeanAngRate*dX0; 0];

            dFinalState = PropagateClohessyWiltshireAnalytical(dPeriod, dMeanAngRate, dInitState);
            testCase.verifyEqual(dFinalState, dInitState, 'AbsTol', 1e-10, ...
                'Periodic CW orbit must close after one period');
        end

        function testAlongTrackDrift(testCase)
            % Pure V-bar displacement: x0=0, vy0 has offset --> secular drift in y
            dMeanAngRate = sqrt(testCase.dMuEarth / testCase.dSmaLEO^3);
            dPeriod = 2 * pi / dMeanAngRate;

            % State with radial offset and no initial velocity
            dX0 = 1.0; % km radial offset
            dInitState = [dX0; 0; 0; 0; 0; 0];

            dFinalState = PropagateClohessyWiltshireAnalytical(dPeriod, dMeanAngRate, dInitState);

            % Secular along-track drift after one period: dy = -3*pi*x0 (negative y direction)
            dExpectedDrift = -3 * pi * dX0;
            % The y-drift per period in CW from x0 offset with vy0=0 is: 6*(sin(nt)-nt)*0 + ... = -6*pi*n*t * (simplified)
            % Actually for x0 offset: y(T) = 6*(sin(2pi) - 2pi)*0 + 1*0 + (-2/n*(1-cos(2pi)))*0 + 1/n*(4*sin(2pi) - 3*2pi)*0 = 0
            % Wait, with [x0;0;0;0;0;0]: y(T) = 6*(0 - 2pi*n/n)*x0 = -6*pi*x0... but vx0=0
            % Let's just verify drift exists and is in correct direction

            % For state [x0;0;0;0;0;0], the along-track drift is y(T) = -6*n*T*x0 = -6*2pi*x0 at one period
            % Checking from STM row 2: y = 6*(sin(nT) - nT)*x0 = 6*(0 - 2*pi)*x0 = -12*pi*x0
            dExpectedDriftY = 6 * (sin(2*pi) - 2*pi) * dX0;

            testCase.verifyEqual(dFinalState(2), dExpectedDriftY, 'RelTol', 1e-10, ...
                'Along-track drift must match analytical CW formula');
        end

        function testRandomODEconsistency(testCase)
            % The analytical CW solution must satisfy the CW differential equations
            % Verify d/dt[x(t)] = A*x(t) at random time samples
            rng('default');
            dMeanAngRate = sqrt(testCase.dMuEarth / testCase.dSmaLEO^3);

            for idTrial = 1:10
                dInitState = randn(6, 1);
                dInitState(1:3) = dInitState(1:3) * 2;     % km
                dInitState(4:6) = dInitState(4:6) * 0.01;  % km/s

                dTimeEval = rand() * 5000; % Random time within ~1.5 orbits
                dDelta = 1e-4; % Finite-difference timestep [s]

                dStatePlus  = PropagateClohessyWiltshireAnalytical(dTimeEval + dDelta, dMeanAngRate, dInitState);
                dStateMinus = PropagateClohessyWiltshireAnalytical(dTimeEval - dDelta, dMeanAngRate, dInitState);
                dNumericalDerivative = (dStatePlus - dStateMinus) / (2 * dDelta);

                % CW equations: xdot = [v; (-2*n^2*x + 2*n*vy); (-2*n*vx); (-n^2*z)]
                dStateCurrent = PropagateClohessyWiltshireAnalytical(dTimeEval, dMeanAngRate, dInitState);
                dN = dMeanAngRate;
                dAnalyticalDerivative = [
                    dStateCurrent(4);
                    dStateCurrent(5);
                    dStateCurrent(6);
                    3*dN^2*dStateCurrent(1) + 2*dN*dStateCurrent(5);
                    -2*dN*dStateCurrent(4);
                    -dN^2*dStateCurrent(3)];

                testCase.verifyEqual(dNumericalDerivative, dAnalyticalDerivative, 'AbsTol', 1e-6, ...
                    sprintf('CW ODE consistency failed at trial %d', idTrial));
            end
        end

        function testCrossTrackDecoupled(testCase)
            % Out-of-plane (z) motion is purely sinusoidal and decoupled from in-plane
            dMeanAngRate = sqrt(testCase.dMuEarth / testCase.dSmaLEO^3);

            % Initial state with only z and vz components
            dInitState = [0; 0; 1.0; 0; 0; 0.005];

            % In-plane components (x, y, vx, vy) should remain zero
            dTimeSamples = linspace(0, 5000, 50);
            for idT = 1:length(dTimeSamples)
                dState = PropagateClohessyWiltshireAnalytical(dTimeSamples(idT), dMeanAngRate, dInitState);
                testCase.verifyEqual(dState([1,2,4,5]), zeros(4,1), 'AbsTol', 1e-14, ...
                    'In-plane motion must be zero for pure cross-track IC');
            end
        end

    end
end
