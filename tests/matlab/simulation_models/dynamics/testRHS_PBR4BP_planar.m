classdef testRHS_PBR4BP_planar < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RHS_PBR4BP_planar and EvalJac_PBR4BP_planar.
    % Validates against CR3BP limit, Jacobian finite differences, and structural properties.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarthMoon = 0.01215058560962404;
        dSunAngRate = 9.25195985e-1;
        dSunDist = 3.88811143e+2;
        dSunGravParam = 3.289005410e+5;
    end

    methods (Test)

        function testSunContributionCancelsAtBarycenter(testCase)
            % At the synodic barycenter, the Sun direct and indirect terms must cancel exactly.
            dMu = testCase.dMuEarthMoon;
            dState = zeros(4, 1);
            dTimes = [0; pi/(2*testCase.dSunAngRate); pi/testCase.dSunAngRate];

            for idT = 1:numel(dTimes)
                dxdt_PBR4BP = RHS_PBR4BP_planar(dState, dMu, dTimes(idT));
                dxdt_CR3BP  = RHS_CR3BP_planar(dState, dMu);

                testCase.verifyEqual(dxdt_PBR4BP, dxdt_CR3BP, 'AbsTol', 1e-12, ...
                    sprintf('Sun forcing must cancel at the barycenter (sample %d)', idT));
            end
        end

        function testReducesToCR3BPWhenSunFar(testCase)
            % With a very large Sun distance (effectively zero Sun influence), PBR4BP --> CR3BP
            % The Sun parameters are hardcoded in the function, so we test that the CR3BP
            % contribution dominates at states far from the Sun
            dMu = testCase.dMuEarthMoon;
            dState = [0.8; 0.1; 0.01; -0.02];

            dxdt_PBR4BP = RHS_PBR4BP_planar(dState, dMu, 0);
            dxdt_CR3BP  = RHS_CR3BP_planar(dState, dMu);

            % Sun perturbation should be small compared to CR3BP terms near the primaries
            dSunContribution = norm(dxdt_PBR4BP - dxdt_CR3BP);
            dCR3BPmagnitude  = norm(dxdt_CR3BP(3:4));

            dRatio = dSunContribution / dCR3BPmagnitude;
            testCase.verifyLessThan(dRatio, 0.05, ...
                'Sun perturbation should be small near the primaries (< 5%% of CR3BP accel)');
        end

        function testJacobianFiniteDifference(testCase)
            % EvalJac_PBR4BP_planar must match finite differences of RHS_PBR4BP_planar
            dMu = testCase.dMuEarthMoon;
            dEps = 1e-7;

            rng('default');
            for idTrial = 1:10
                dState = randn(4, 1) * 0.3;
                dState(1) = dState(1) + 0.8;
                dTime = rand() * 10;

                dJacAnalytical = EvalJac_PBR4BP_planar(dState, dMu, dTime);
                dJacNumerical = zeros(4, 4);

                for idCol = 1:4
                    dSp = dState; dSp(idCol) = dSp(idCol) + dEps;
                    dSm = dState; dSm(idCol) = dSm(idCol) - dEps;
                    dJacNumerical(:, idCol) = (RHS_PBR4BP_planar(dSp, dMu, dTime) ...
                                             - RHS_PBR4BP_planar(dSm, dMu, dTime)) / (2*dEps);
                end

                dRelErr = norm(dJacAnalytical - dJacNumerical, 'fro') / norm(dJacNumerical, 'fro');
                testCase.verifyLessThan(dRelErr, 1e-5, ...
                    sprintf('PBR4BP Jacobian FD mismatch at trial %d', idTrial));
            end
        end

        function testJacobianStructure(testCase)
            % Same block structure as CR3BP: [0 I; Uxx C]
            dMu = testCase.dMuEarthMoon;
            dState = [0.8; 0.1; 0; 0];

            dJac = EvalJac_PBR4BP_planar(dState, dMu, 0);

            testCase.verifyEqual(dJac(1:2, 1:2), zeros(2), 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(1:2, 3:4), eye(2), 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(3, 4), 2, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(4, 3), -2, 'AbsTol', 1e-15);
        end

        function testSunCouplingJacobianMatchesQuarterPhaseAnalyticalValue(testCase)
            % At quarter Sun phase, the legacy planar bicircular model places the Sun on the -y axis.
            % For y=0, the CR3BP contribution to Uxy vanishes, so the Jacobian off-diagonal is purely
            % the Sun Hessian contribution and its sign is a sensitive regression check.
            dMu = testCase.dMuEarthMoon;
            dTime = pi / (2 * testCase.dSunAngRate);
            dState = [0.5; 0.0; 0.0; 0.0];

            dJac = EvalJac_PBR4BP_planar(dState, dMu, dTime);

            dRelX_Sun = dState(1);
            dRelY_Sun = testCase.dSunDist;
            dDistSunSq = dRelX_Sun^2 + dRelY_Sun^2;
            dExpectedUxy = 3 * testCase.dSunGravParam * dRelX_Sun * dRelY_Sun / dDistSunSq^(5/2);

            testCase.verifyGreaterThan(dExpectedUxy, 0, ...
                'Quarter-phase Sun coupling should be positive in the planar bicircular model');
            testCase.verifyEqual(dJac(3, 2), dExpectedUxy, 'RelTol', 1e-12, ...
                'Uxy must match the analytical Sun coupling term at quarter phase');
            testCase.verifyEqual(dJac(4, 1), dExpectedUxy, 'RelTol', 1e-12, ...
                'Jacobian lower-left block must remain symmetric');
        end

        function testHessianSymmetry(testCase)
            % Lower-left 2x2 Hessian block (minus centrifugal) must be symmetric
            dMu = testCase.dMuEarthMoon;

            rng(7);
            for idT = 1:10
                dState = randn(4, 1) * 0.3;
                dState(1) = dState(1) + 0.8;
                dTime = rand() * 10;

                dJac = EvalJac_PBR4BP_planar(dState, dMu, dTime);

                dUxxBlock = dJac(3:4, 1:2);
                dUxxBlock(1,1) = dUxxBlock(1,1) - 1; % Remove centrifugal
                dUxxBlock(2,2) = dUxxBlock(2,2) - 1;

                testCase.verifyEqual(dUxxBlock(1,2), dUxxBlock(2,1), 'AbsTol', 1e-12, ...
                    'Hessian off-diagonals must be symmetric');
            end
        end

        function testTimePeriodicSunInfluence(testCase)
            % RHS should differ between t=0 and t=pi/dSunAngRate (half Sun period)
            dMu = testCase.dMuEarthMoon;
            dState = [0.8; 0.1; 0.01; -0.02];

            dxdt_t0 = RHS_PBR4BP_planar(dState, dMu, 0);
            dSunAngRate = 9.25195985e-1;
            dHalfSunPeriod = pi / dSunAngRate;
            dxdt_tHalf = RHS_PBR4BP_planar(dState, dMu, dHalfSunPeriod);

            % Should differ due to Sun being on opposite side
            testCase.verifyGreaterThan(norm(dxdt_t0 - dxdt_tHalf), 1e-10, ...
                'RHS must differ at opposite Sun phases');
        end

    end
end
