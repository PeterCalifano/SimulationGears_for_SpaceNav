classdef testRHS_CR3BP_planar < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RHS_CR3BP_planar and EvalJac_CR3BP_planar.
    % Validates against the full 3D CR3BP (z=0 slice), Jacobi conservation, and finite-difference Jacobian.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarthMoon = 0.01215058560962404;
    end

    methods (Test)

        function testConsistencyWith3DCRBP(testCase)
            % Planar RHS must match the 3D CR3BP evaluated at z=vz=0
            dMu = testCase.dMuEarthMoon;

            rng('default');
            for idT = 1:15
                dState4 = randn(4, 1) * 0.3;
                dState4(1) = dState4(1) + 0.8;

                dxdt_planar = RHS_CR3BP_planar(dState4, dMu);

                dState6 = [dState4(1); dState4(2); 0; dState4(3); dState4(4); 0];
                dxdt_3D = RHS_CR3BP(dState6, dMu);

                dExpected = [dxdt_3D(4); dxdt_3D(5)]; % accelerations x,y from 3D
                testCase.verifyEqual(dxdt_planar(3:4), dExpected, 'AbsTol', 1e-13, ...
                    sprintf('Planar/3D mismatch at trial %d', idT));
            end
        end

        function testPlanarJacobiConservation(testCase)
            % Planar Jacobi constant: C = 2*U(x,y) - (vx^2+vy^2) must be conserved
            dMu = testCase.dMuEarthMoon;
            dState0 = [0.8; 0.1; 0.01; -0.02];

            dJacobi0 = testCase.computePlanarJacobi(dState0, dMu);

            objOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
            [~, dTraj] = ode113(@(dTime, dState) RHS_CR3BP_planar(dState, dMu), ...
                                linspace(0, 5, 200), dState0, objOpts);

            for idT = 1:size(dTraj, 1)
                dJacobiCurr = testCase.computePlanarJacobi(dTraj(idT, :)', dMu);
                dRelErr = abs(dJacobiCurr - dJacobi0) / abs(dJacobi0);
                testCase.verifyLessThan(dRelErr, 1e-10, ...
                    sprintf('Planar Jacobi drift at point %d', idT));
            end
        end

        function testPlanarJacobianFiniteDifference(testCase)
            % EvalJac_CR3BP_planar must match finite differences of RHS_CR3BP_planar
            dMu = testCase.dMuEarthMoon;
            dEps = 1e-7;

            rng(42);
            for idTrial = 1:10
                dState = randn(4, 1) * 0.3;
                dState(1) = dState(1) + 0.8;

                dJacAnalytical = EvalJac_CR3BP_planar(dState, dMu);
                dJacNumerical = zeros(4, 4);

                for idCol = 1:4
                    dSp = dState; dSp(idCol) = dSp(idCol) + dEps;
                    dSm = dState; dSm(idCol) = dSm(idCol) - dEps;
                    dJacNumerical(:, idCol) = (RHS_CR3BP_planar(dSp, dMu) - RHS_CR3BP_planar(dSm, dMu)) / (2*dEps);
                end

                dRelErr = norm(dJacAnalytical - dJacNumerical, 'fro') / norm(dJacNumerical, 'fro');
                testCase.verifyLessThan(dRelErr, 1e-6, ...
                    sprintf('Planar Jacobian FD mismatch at trial %d', idTrial));
            end
        end

        function testPlanarJacobianStructure(testCase)
            % Upper-left 2x2 = zeros, upper-right 2x2 = identity, Coriolis terms correct
            dMu = testCase.dMuEarthMoon;
            dState = [0.8; 0.1; 0; 0];

            dJac = EvalJac_CR3BP_planar(dState, dMu);

            testCase.verifyEqual(dJac(1:2, 1:2), zeros(2), 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(1:2, 3:4), eye(2), 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(3, 4), 2, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(4, 3), -2, 'AbsTol', 1e-15);
        end

    end

    methods (Static)
        function dJacobi = computePlanarJacobi(dState, dMu)
            dPx = dState(1); dPy = dState(2);
            dVsq = dState(3)^2 + dState(4)^2;
            dR1 = sqrt((dPx + dMu)^2 + dPy^2);
            dR2 = sqrt((dPx - 1 + dMu)^2 + dPy^2);
            dU = 0.5*(dPx^2 + dPy^2) + (1-dMu)/dR1 + dMu/dR2;
            dJacobi = 2*dU - dVsq;
        end
    end
end
