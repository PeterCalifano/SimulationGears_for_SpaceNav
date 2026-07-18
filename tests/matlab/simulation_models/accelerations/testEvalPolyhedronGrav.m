classdef testEvalPolyhedronGrav < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests and benchmark for EvalPolyhedronGrav and ComputePolyhedronFaceEdgeData.
    % Uses a unit cube (analytically tractable) and a regular tetrahedron as
    % test shapes. Validates correctness properties of the gravity field and
    % provides timing benchmarks to compare loop vs vectorized implementations.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 29-03-2026    Pietro Califano    First implementation.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dGravConst = 6.67430e-11; % [m^3/(kg*s^2)]
    end

    properties (TestParameter)
        % Multiple exterior field points for parametric testing
        dFieldPointExterior = struct( ...
            'along_x',      [3.0; 0.0; 0.0], ...
            'along_z',      [0.0; 0.0; 4.0], ...
            'diagonal',     [2.0; 2.0; 2.0], ...
            'off_axis',     [1.5; 3.0; 2.5], ...
            'far_field',    [10.0; 10.0; 10.0] ...
        );
    end

    methods (Static)

        function [ui32Faces, dVerts] = BuildUnitCube()
            % Unit cube centered at origin, side length 2 (vertices at +/-1)
            % Triangulated: 6 faces x 2 triangles = 12 triangles
            dVerts = [ -1 -1 -1;
                        1 -1 -1;
                        1  1 -1;
                       -1  1 -1;
                       -1 -1  1;
                        1 -1  1;
                        1  1  1;
                       -1  1  1 ];

            % Outward-wound triangular faces are required by the
            % Werner-Scheeres solid-angle formulation.
            ui32Faces = uint32([ ...
                1 3 2; 1 4 3;   % -Z face
                5 6 7; 5 7 8;   % +Z face
                1 2 6; 1 6 5;   % -Y face
                3 8 7; 3 4 8;   % +Y face
                1 8 4; 1 5 8;   % -X face
                2 7 6; 2 3 7 ]); % +X face
        end

        function [ui32Faces, dVerts] = BuildRegularTetrahedron()
            % Regular tetrahedron centered at origin, edge length ~2.31
            dVerts = [ 1  1  1;
                       1 -1 -1;
                      -1  1 -1;
                      -1 -1  1 ];
            ui32Faces = uint32([1 2 3; 1 4 2; 1 3 4; 2 4 3]);
        end

        function [ui32Faces, dVerts] = BuildIcosphere(nSubdivisions)
            % Build an icosphere by subdividing an icosahedron.
            % Useful for higher face counts in benchmarking.
            arguments
                nSubdivisions (1,1) uint32 = uint32(2)
            end

            % Icosahedron base vertices
            dPhi = (1 + sqrt(5)) / 2;
            dVerts = [-1  dPhi 0;  1  dPhi 0; -1 -dPhi 0;  1 -dPhi 0;
                       0 -1  dPhi;  0  1  dPhi;  0 -1 -dPhi; 0  1 -dPhi;
                       dPhi 0 -1;  dPhi 0  1; -dPhi 0 -1; -dPhi 0  1];
            dVerts = dVerts ./ vecnorm(dVerts, 2, 2);

            ui32Faces = uint32([ ...
                1 12 6; 1 6 2; 1 2 8; 1 8 11; 1 11 12;
                2 6 10; 6 12 5; 12 11 3; 11 8 7; 8 2 9;
                4 10 5; 4 5 3; 4 3 7; 4 7 9; 4 9 10;
                5 10 6; 3 5 12; 7 3 11; 9 7 8; 10 9 2]);

            for idxSub = 1:nSubdivisions
                nFaces = size(ui32Faces, 1);
                ui32NewFaces = zeros(nFaces * 4, 3, 'uint32');
                edgeMap = containers.Map('KeyType', 'char', 'ValueType', 'uint32');

                for f = 1:nFaces
                    v = ui32Faces(f, :);
                    mids = zeros(1, 3, 'uint32');
                    edgePairs = [v(1) v(2); v(2) v(3); v(3) v(1)];
                    for e = 1:3
                        key = sprintf('%d_%d', min(edgePairs(e,:)), max(edgePairs(e,:)));
                        if isKey(edgeMap, key)
                            mids(e) = edgeMap(key);
                        else
                            dNewVert = (dVerts(edgePairs(e,1),:) + dVerts(edgePairs(e,2),:)) / 2;
                            dNewVert = dNewVert / norm(dNewVert);
                            dVerts = [dVerts; dNewVert]; %#ok<AGROW>
                            mids(e) = uint32(size(dVerts, 1));
                            edgeMap(key) = mids(e);
                        end
                    end
                    a = v(1); b = v(2); c = v(3);
                    ab = mids(1); bc = mids(2); ca = mids(3);
                    ui32NewFaces((f-1)*4+1, :) = [a  ab ca];
                    ui32NewFaces((f-1)*4+2, :) = [b  bc ab];
                    ui32NewFaces((f-1)*4+3, :) = [c  ca bc];
                    ui32NewFaces((f-1)*4+4, :) = [ab bc ca];
                end
                ui32Faces = ui32NewFaces;
            end
        end

    end

    %% ========================================================================
    %  CORRECTNESS TESTS
    %  ========================================================================
    methods (Test)

        function testOutputDimensions(testCase)
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            dFP = [3; 0; 0];
            dDensity = 2000;
            [dAcc, dJac, dU, dD2U] = EvalPolyhedronGrav(dFP, ui32Faces, dVerts, ...
                dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);

            testCase.verifySize(dAcc, [3, 1], 'Acceleration must be 3x1');
            testCase.verifySize(dJac, [3, 3], 'Jacobian must be 3x3');
            testCase.verifySize(dU,   [1, 1], 'Potential must be scalar');
            testCase.verifySize(dD2U, [1, 1], 'Laplacian must be scalar');
        end

        function testAccelerationPointsTowardBody(testCase, dFieldPointExterior)
            % Gravitational acceleration must point toward the body center of mass
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            dAcc = EvalPolyhedronGrav(dFieldPointExterior, ui32Faces, dVerts, ...
                2000, ui32Edges, dEe, dFf, testCase.dGravConst);

            % For a symmetric body centered at origin, acceleration should
            % point roughly toward origin (negative dot with position)
            testCase.verifyLessThan(dot(dAcc, dFieldPointExterior), 0, ...
                'Acceleration must point toward the body');
        end

        function testFarFieldConvergesToPointMass(testCase)
            % At large distances, polyhedron gravity should converge to -GM*r/|r|^3
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            dDensity = 2000;
            dVolume = 8.0; % 2x2x2 cube
            dMass = dDensity * dVolume;
            dGM = testCase.dGravConst * dMass;

            dFarPoint = [1000; 500; 700];
            dR = norm(dFarPoint);

            dAcc = EvalPolyhedronGrav(dFarPoint, ui32Faces, dVerts, ...
                dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);

            dAccPointMass = -dGM / dR^3 * dFarPoint;

            testCase.verifyEqual(dAcc, dAccPointMass, 'RelTol', 1e-4, ...
                'Far-field acceleration must converge to point-mass model');
        end

        function testJacobianIsSymmetric(testCase, dFieldPointExterior)
            % Jacobian of gravity = Hessian of potential, must be symmetric
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            [~, dJac] = EvalPolyhedronGrav(dFieldPointExterior, ui32Faces, dVerts, ...
                2000, ui32Edges, dEe, dFf, testCase.dGravConst);

            testCase.verifyEqual(dJac, dJac', 'AbsTol', 1e-20 * norm(dJac, 'fro'), ...
                'Jacobian must be symmetric (Hessian of potential)');
        end

        function testLaplacianIsZeroExterior(testCase, dFieldPointExterior)
            % Laplace equation: trace(Hessian) = 0 in exterior field
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            [~, dJac, ~, dD2U] = EvalPolyhedronGrav(dFieldPointExterior, ui32Faces, dVerts, ...
                2000, ui32Edges, dEe, dFf, testCase.dGravConst);

            dTol = 1e-18;

            % Both the explicit Laplacian output and trace of Jacobian must vanish
            testCase.verifyEqual(dD2U, 0.0, 'AbsTol', dTol, ...
                'Laplacian must be zero in exterior field');
            testCase.verifyEqual(trace(dJac), 0.0, 'AbsTol', dTol, ...
                'Trace of Jacobian must be zero in exterior field');
        end

        function testJacobianMatchesFiniteDifference(testCase, dFieldPointExterior)
            % Verify analytical Jacobian against central finite differences
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);
            dDensity = 2000;

            [~, dJacAnalytic] = EvalPolyhedronGrav(dFieldPointExterior, ui32Faces, dVerts, ...
                dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);

            dStep = 1e-7 * norm(dFieldPointExterior);
            dJacFD = zeros(3, 3);
            for k = 1:3
                dPosPlus  = dFieldPointExterior;
                dPosMinus = dFieldPointExterior;
                dPosPlus(k)  = dPosPlus(k)  + dStep;
                dPosMinus(k) = dPosMinus(k) - dStep;

                dAccPlus  = EvalPolyhedronGrav(dPosPlus, ui32Faces, dVerts, ...
                    dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);
                dAccMinus = EvalPolyhedronGrav(dPosMinus, ui32Faces, dVerts, ...
                    dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);

                dJacFD(:, k) = (dAccPlus - dAccMinus) / (2.0 * dStep);
            end

            testCase.verifyEqual(dJacAnalytic, dJacFD, 'RelTol', 1e-5, 'AbsTol', 1e-15, ...
                'Analytical Jacobian must match central finite differences');
        end

        function testPotentialPositiveExterior(testCase, dFieldPointExterior)
            % Celestial-mechanics potential U is positive and satisfies
            % dAcc = grad(U).
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            [~, ~, dU] = EvalPolyhedronGrav(dFieldPointExterior, ui32Faces, dVerts, ...
                2000, ui32Edges, dEe, dFf, testCase.dGravConst);

            testCase.verifyGreaterThan(dU, 0, ...
                'Gravitational potential must be positive in exterior field');
        end

        function testTetrahedronConsistency(testCase)
            % Cross-check: tetrahedron should also satisfy all field properties
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildRegularTetrahedron();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            dFP = [5; 3; 4];
            [dAcc, dJac, dU, dD2U] = EvalPolyhedronGrav(dFP, ui32Faces, dVerts, ...
                3000, ui32Edges, dEe, dFf, testCase.dGravConst);

            testCase.verifyLessThan(dot(dAcc, dFP), 0, 'Attraction toward body');
            testCase.verifyGreaterThan(dU, 0, 'Positive potential');
            testCase.verifyEqual(dJac, dJac', 'AbsTol', 1e-20 * norm(dJac, 'fro'), 'Symmetric Jacobian');
            testCase.verifyEqual(dD2U, 0.0, 'AbsTol', 1e-18, 'Zero Laplacian');
        end

        function testPreprocessingOutputSizes(testCase)
            % Verify ComputePolyhedronFaceEdgeData output dimensions
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            ui32NumFaces = size(ui32Faces, 1);
            ui32NumEdges = size(ui32Edges, 1);

            testCase.verifySize(ui32Edges, [ui32NumEdges, 2]);
            testCase.verifySize(dEe, [3, 3, ui32NumEdges]);
            testCase.verifySize(dFf, [3, 3, ui32NumFaces]);
        end

    end

    %% ========================================================================
    %  CROSS-VALIDATION (simulationUtils, skipped when not on path)
    %  ========================================================================
    methods (Test)

        function testCrossValidateAgainstSimulationUtils(testCase, dFieldPointExterior)
            % Compare EvalPolyhedronGrav against simulationUtils'
            % polyhedronWSparVect for numerical equivalence.
            testCase.assumeTrue(~isempty(which('polyhedronWSparVect')), ...
                'simulationUtils polyhedronWSparVect not on path, skipping');
            testCase.assumeTrue(~isempty(which('polyhedronFaceEdgeProcess')), ...
                'simulationUtils polyhedronFaceEdgeProcess not on path, skipping');

            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();
            dDensity = 2000;

            % SimGears preprocessing and evaluation
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);
            [dAccSG, dJacSG, dUSG, dD2USG] = EvalPolyhedronGrav(dFieldPointExterior, ...
                ui32Faces, dVerts, dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);

            % simulationUtils preprocessing and evaluation
            dFacesDouble = double(ui32Faces);
            [EdgSU, EeSU, FfSU] = polyhedronFaceEdgeProcess(dFacesDouble, dVerts);
            [dAccSU, dJacSU, dD2USU, dUSU] = polyhedronWSparVect( ...
                dFieldPointExterior, dFacesDouble, dVerts, dDensity, EdgSU, EeSU, FfSU, testCase.dGravConst);

            testCase.verifyEqual(dAccSG, dAccSU, 'RelTol', 1e-10, 'AbsTol', 1e-18, ...
                'Acceleration must match simulationUtils');
            testCase.verifyEqual(dJacSG, dJacSU, 'RelTol', 1e-10, 'AbsTol', 1e-18, ...
                'Jacobian must match simulationUtils');
            testCase.verifyEqual(dUSG, dUSU, 'RelTol', 1e-10, 'AbsTol', 1e-18, ...
                'Potential must match simulationUtils');
            testCase.verifyEqual(dD2USG, dD2USU, 'RelTol', 1e-10, 'AbsTol', 1e-18, ...
                'Laplacian must match simulationUtils');
        end

        function testPreprocessingRejectsInwardFaces(testCase)
            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildUnitCube();

            testCase.verifyError(@() ComputePolyhedronFaceEdgeData(ui32Faces(:, [1 3 2]), dVerts), ...
                'ComputePolyhedronFaceEdgeData:InvalidFaceOrientation');
        end

    end

    %% ========================================================================
    %  BENCHMARK
    %  ========================================================================
    methods (Test)

        function testBenchmarkEvalPolyhedronGrav(testCase)
            % Timed benchmark for performance tracking.
            % Runs on icosphere (320 faces, 162 vertices) to approximate a
            % realistic small-body shape model. Prints timing.

            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildIcosphere(uint32(2));
            dVerts = dVerts * 250; % Scale to ~250 m radius body

            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            dDensity = 2000;
            dFP = [500; 300; 400];
            nIter = 500;

            % Warm up
            EvalPolyhedronGrav(dFP, ui32Faces, dVerts, dDensity, ...
                ui32Edges, dEe, dFf, testCase.dGravConst);

            % Benchmark
            tic;
            for k = 1:nIter
                [dAcc, dJac, dU, dD2U] = EvalPolyhedronGrav(dFP, ui32Faces, dVerts, ...
                    dDensity, ui32Edges, dEe, dFf, testCase.dGravConst);
            end
            dTotalTime = toc;
            dTimePerEval = dTotalTime / nIter;

            fprintf('\n--- EvalPolyhedronGrav Benchmark ---\n');
            fprintf('Mesh: %d faces, %d edges, %d vertices\n', ...
                size(ui32Faces,1), size(ui32Edges,1), size(dVerts,1));
            fprintf('Iterations: %d\n', nIter);
            fprintf('Time per evaluation: %.4g ms\n', dTimePerEval * 1e3);
            fprintf('Total time: %.4g s\n', dTotalTime);

            % Sanity check the result is physically valid
            testCase.verifyLessThan(dot(dAcc, dFP), 0, 'Attraction toward body');
        end

        function testBenchmarkPreprocessing(testCase)
            % Benchmark for the one-time preprocessing step

            [ui32Faces, dVerts] = testEvalPolyhedronGrav.BuildIcosphere(uint32(2));
            dVerts = dVerts * 250;
            nIter = 20;

            % Warm up
            ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);

            tic;
            for k = 1:nIter
                [~, ~, ~] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);
            end
            dTotalTime = toc;

            fprintf('\n--- ComputePolyhedronFaceEdgeData Benchmark ---\n');
            fprintf('Mesh: %d faces, %d vertices\n', size(ui32Faces,1), size(dVerts,1));
            fprintf('Time per call: %.4g ms\n', (dTotalTime / nIter) * 1e3);
        end

    end

end
