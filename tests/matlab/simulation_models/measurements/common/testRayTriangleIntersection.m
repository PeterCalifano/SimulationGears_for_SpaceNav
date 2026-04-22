classdef testRayTriangleIntersection < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RayTriangleIntersection_MollerTrumbore.
    % Covers deterministic geometry, ray-vs-line semantics, optional arguments,
    % and randomized non-degenerate hit cases.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        % Simple axis-aligned triangle in the z=0 plane
        dVert0 = [0; 0; 0];
        dVert1 = [1; 0; 0];
        dVert2 = [0; 1; 0];
    end

    methods (Test)

        function testDefaultArgumentsMatchExplicitTwoSidedRay(testCase)
            % Calling without optional flags must match the documented defaults.
            % TODO(devDir): keep root-repo callers aligned with the 6-input
            % RayTriangleIntersection_MollerTrumbore signature.
            dCentroid = (testCase.dVert0 + testCase.dVert1 + testCase.dVert2) / 3;
            dRayOrigin = dCentroid + [0; 0; 5];
            dRayDir = [0; 0; -1];

            [bHitDefault, dUdefault, dVdefault, dRangeDefault, dIntPtDefault] = ...
                RayTriangleIntersection_MollerTrumbore( ...
                    dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2);

            [bHitExplicit, dUexplicit, dVexplicit, dRangeExplicit, dIntPtExplicit] = ...
                RayTriangleIntersection_MollerTrumbore( ...
                    dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2, true);

            testCase.verifyEqual(bHitDefault, bHitExplicit);
            testCase.verifyEqual(dUdefault, dUexplicit, 'AbsTol', 1e-12);
            testCase.verifyEqual(dVdefault, dVexplicit, 'AbsTol', 1e-12);
            testCase.verifyEqual(dRangeDefault, dRangeExplicit, 'AbsTol', 1e-12);
            testCase.verifyEqual(dIntPtDefault, dIntPtExplicit, 'AbsTol', 1e-12);
        end

        function testRayThroughCentroidHits(testCase)
            % Ray from above pointing down through the triangle centroid
            dCentroid = (testCase.dVert0 + testCase.dVert1 + testCase.dVert2) / 3;
            dRayOrigin = dCentroid + [0; 0; 5];
            dRayDir = [0; 0; -1];

            [bHit, dU, dV, dRange, dIntPt] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2);

            testCase.verifyTrue(bHit, 'Ray through centroid must hit');
            testCase.verifyEqual(dRange, 5, 'AbsTol', 1e-12, 'Range should be 5');

            % Barycentric coordinates at centroid: u=1/3, v=1/3
            testCase.verifyEqual(dU, 1/3, 'AbsTol', 1e-12);
            testCase.verifyEqual(dV, 1/3, 'AbsTol', 1e-12);

            % u + v should be less than 1 (third coord = 1-u-v = 1/3)
            testCase.verifyLessThanOrEqual(dU + dV, 1 + 1e-12);

            % Intersection point should be at centroid
            testCase.verifyEqual(dIntPt, dCentroid, 'AbsTol', 1e-12);
        end

        function testParallelRayMisses(testCase)
            % Ray parallel to the triangle plane should miss
            dRayOrigin = [0.3; 0.3; 1];
            dRayDir = [1; 0; 0] / norm([1; 0; 0]); % Parallel to z=0 plane

            [bHit, ~, ~, ~, ~] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2);

            testCase.verifyFalse(bHit, 'Parallel ray must miss');
        end

        function testRayMissesOutsideTriangle(testCase)
            % Ray pointing down but outside the triangle boundary
            dRayOrigin = [2; 2; 5]; % Outside the triangle
            dRayDir = [0; 0; -1];

            [bHit, ~, ~, ~, ~] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2);

            testCase.verifyFalse(bHit, 'Ray outside triangle boundary must miss');
        end

        function testRayHitsEdge(testCase)
            % Ray hits the edge between V0 and V1 (midpoint at (0.5, 0, 0))
            dMidpoint = (testCase.dVert0 + testCase.dVert1) / 2;
            dRayOrigin = dMidpoint + [0; 0; 3];
            dRayDir = [0; 0; -1];

            [bHit, dU, dV, ~, dIntPt] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2);

            testCase.verifyTrue(bHit, 'Ray through edge midpoint must hit');
            testCase.verifyEqual(dV, 0, 'AbsTol', 1e-12, 'V-coord should be 0 on V0-V1 edge');
            testCase.verifyEqual(dU, 0.5, 'AbsTol', 1e-12, 'U-coord should be 0.5 at midpoint');
            testCase.verifyEqual(dIntPt, dMidpoint, 'AbsTol', 1e-12);
        end

        function testRayBehindOriginMissesEvenForTwoSidedTest(testCase)
            % A ray intersection with negative t is behind the origin and must be rejected.
            dRayOrigin = [0.2; 0.2; 5];
            dRayDir = [0; 0; 1]; % Pointing away from the triangle

            [bHit, dU, dV, dRange, dIntPt] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2, true);

            testCase.verifyFalse(bHit, 'Intersection behind the ray origin must be rejected');
            testCase.verifyEqual(dU, 0);
            testCase.verifyEqual(dV, 0);
            testCase.verifyEqual(dRange, 0);
            testCase.verifyEqual(dIntPt, zeros(3, 1));
        end

        function testOneSidedBackfaceCulling(testCase)
            % One-sided test: ray hitting triangle from behind should be culled
            dRayOrigin = [0.2; 0.2; -5];
            dRayDir = [0; 0; 1]; % Hitting from below (backface)

            [bHitOneSided, ~, ~, ~, ~] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2, false);

            [bHitTwoSided, ~, ~, ~, ~] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2, true);

            % One-sided should cull backface; two-sided should still hit
            testCase.verifyFalse(bHitOneSided, 'One-sided test must cull backface');
            testCase.verifyTrue(bHitTwoSided, 'Two-sided test must detect backface hit');
        end

        function testOneSidedModeDetectsForwardIntersection(testCase)
            % One-sided mode must still detect forward intersections.
            dRayOrigin = [0.2; 0.2; 5];
            dRayDir = [0; 0; -1];

            [bHitOneSided, ~, ~, dRangeOneSided, ~] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2, false);

            testCase.verifyTrue(bHitOneSided, 'One-sided ray must detect the forward intersection');
            testCase.verifyGreaterThan(dRangeOneSided, 0);
        end

        function testOneSidedModeRejectsIntersectionBehindOrigin(testCase)
            % One-sided mode is still a ray test: negative t must be rejected.
            dRayOrigin = [0.2; 0.2; -5];
            dRayDir = [0; 0; -1];

            [bHitOneSided, dU, dV, dRangeOneSided, dIntPt] = RayTriangleIntersection_MollerTrumbore( ...
                dRayOrigin, dRayDir, testCase.dVert0, testCase.dVert1, testCase.dVert2, false);

            testCase.verifyFalse(bHitOneSided, 'One-sided ray must ignore intersections behind the origin');
            testCase.verifyEqual(dU, 0);
            testCase.verifyEqual(dV, 0);
            testCase.verifyEqual(dRangeOneSided, 0);
            testCase.verifyEqual(dIntPt, zeros(3, 1));
        end

        function testRandomRaysGuaranteedHit(testCase)
            % Randomized non-degenerate geometry with analytically known barycentric coordinates.
            rng('default');

            for idT = 1:30
                [dV0, dV1, dV2] = testRayTriangleIntersection.generateNonDegenerateTriangle();

                dUexpected = 0.05 + 0.40 * rand();
                dVexpected = 0.05 + (0.90 - dUexpected) * rand();
                dTargetPoint = (1 - dUexpected - dVexpected) * dV0 + ...
                               dUexpected * dV1 + ...
                               dVexpected * dV2;

                dNormal = cross(dV1 - dV0, dV2 - dV0);
                dNormal = dNormal / norm(dNormal);

                dRayOrigin = dTargetPoint + (1 + rand()) * dNormal + 0.2 * randn(3, 1);
                dRayDir = dTargetPoint - dRayOrigin;
                dRayDir = dRayDir / norm(dRayDir);

                [bHit, dU, dV, dRange, dIntPt] = RayTriangleIntersection_MollerTrumbore( ...
                    dRayOrigin, dRayDir, dV0, dV1, dV2, true);

                testCase.verifyTrue(bHit, sprintf('Guaranteed-hit case missed at trial %d', idT));
                testCase.verifyGreaterThan(dRange, 0, sprintf('Non-positive range at trial %d', idT));
                testCase.verifyEqual(dU, dUexpected, 'AbsTol', 1e-10, ...
                    sprintf('Recovered U barycentric coordinate mismatch at trial %d', idT));
                testCase.verifyEqual(dV, dVexpected, 'AbsTol', 1e-10, ...
                    sprintf('Recovered V barycentric coordinate mismatch at trial %d', idT));
                testCase.verifyEqual(dIntPt, dTargetPoint, 'AbsTol', 1e-10, ...
                    sprintf('Intersection point mismatch at trial %d', idT));
            end
        end

        function testBarycentricCoordsRecoverPoint(testCase)
            % Verify the recovered barycentric coordinates reconstruct the hit point.
            rng(99);
            for idT = 1:15
                [dV0, dV1, dV2] = testRayTriangleIntersection.generateNonDegenerateTriangle();

                dCentroid = (dV0 + dV1 + dV2) / 3;
                dNormal = cross(dV1 - dV0, dV2 - dV0);
                dNormal = dNormal / norm(dNormal);
                dRangeExpected = 1 + rand();
                dRayOrigin = dCentroid + dRangeExpected * dNormal;
                dRayDir = -dNormal;

                [bHit, dU, dV, dRange, dIntPt] = RayTriangleIntersection_MollerTrumbore( ...
                    dRayOrigin, dRayDir, dV0, dV1, dV2, true);

                dReconstructed = (1 - dU - dV) * dV0 + dU * dV1 + dV * dV2;

                testCase.verifyTrue(bHit, sprintf('Centroid hit missed at trial %d', idT));
                testCase.verifyEqual(dRange, dRangeExpected, 'AbsTol', 1e-12);
                testCase.verifyEqual(dIntPt, dCentroid, 'AbsTol', 1e-12);
                testCase.verifyEqual(dIntPt, dReconstructed, 'AbsTol', 1e-12, ...
                    'Intersection point must match barycentric reconstruction');
            end
        end

    end

    methods (Static, Access = private)
        function [dV0, dV1, dV2] = generateNonDegenerateTriangle()
            while true
                dV0 = randn(3, 1) * 4;
                dEdge1 = randn(3, 1);
                dEdge2 = randn(3, 1);

                if norm(dEdge1) > 0.25 && norm(dEdge2) > 0.25 && norm(cross(dEdge1, dEdge2)) > 0.25
                    dV1 = dV0 + dEdge1;
                    dV2 = dV0 + dEdge2;
                    return;
                end
            end
        end
    end
end
