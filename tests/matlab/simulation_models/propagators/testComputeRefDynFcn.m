classdef testComputeRefDynFcn < matlab.unittest.TestCase
    properties (Constant)
        dGM = 3.986004418e5;
        dRefRadius = 6378.0;
    end

    methods (Test)
        function testCentralGravityJacobianAndSTM(testCase)
            dxState = [7000.0; -1200.0; 900.0; 0.1; 7.4; -0.3];
            strDynParams = testCase.buildCentralDynParams();

            [dDxDt, ~, dDynMatrix] = ComputeRefDynFcn(0.0, dxState, strDynParams);
            dJacFD = testCase.finiteDifferenceJacobian(dxState, strDynParams);

            testCase.verifyEqual(dDxDt(1:3), dxState(4:6), 'AbsTol', 0.0);
            testCase.verifyEqual(dDynMatrix, dJacFD, 'RelTol', 1e-7, 'AbsTol', 1e-9);

            dSTM0 = reshape(eye(6), 36, 1);
            dDxDtSTM = ComputeRefDynFcn(0.0, [dxState; dSTM0], strDynParams);

            testCase.verifyEqual(dDxDtSTM(1:6), dDxDt, 'AbsTol', 0.0);
            testCase.verifyEqual(reshape(dDxDtSTM(7:42), 6, 6), dDynMatrix, 'AbsTol', 1e-14);
        end

        function testZeroSphericalHarmonicsPathPreservesCentralDynamics(testCase)
            dxState = [7200.0; 300.0; -200.0; 0.0; 7.2; 0.1];
            strDynParams = testCase.buildCentralDynParams();
            strDynParams.strMainData.dSHcoeff = zeros(4, 2);
            strDynParams.strMainData.ui16MaxSHdegree = uint16(2);

            [dDxDtSH, ~, dDynMatrixSH] = ComputeRefDynFcn(0.0, dxState, strDynParams);

            strDynParams.strMainData.dSHcoeff = [];
            strDynParams.strMainData.ui16MaxSHdegree = uint16(0);
            [dDxDtCentral, ~, dDynMatrixCentral] = ComputeRefDynFcn(0.0, dxState, strDynParams);

            testCase.verifyEqual(dDxDtSH, dDxDtCentral, 'AbsTol', 1e-14);
            testCase.verifyEqual(dDynMatrixSH, dDynMatrixCentral, 'AbsTol', 1e-14);
        end

        function testSphericalHarmonicsJacobianMatchesFiniteDifference(testCase)
            dxState = [7200.0; 300.0; -200.0; 0.0; 7.2; 0.1];
            strDynParams = testCase.buildCentralDynParams();
            strDynParams.strMainData.dSHcoeff = zeros(4, 2);
            strDynParams.strMainData.dSHcoeff(2, 1) = -1.0e-3;
            strDynParams.strMainData.dSHcoeff(4, 2) = 2.0e-4;
            strDynParams.strMainData.ui16MaxSHdegree = uint16(2);

            [~, ~, dDynMatrix] = ComputeRefDynFcn(0.0, dxState, strDynParams);
            dJacFD = testCase.finiteDifferenceJacobian(dxState, strDynParams);

            strDynParamsCentral = strDynParams;
            strDynParamsCentral.strMainData.dSHcoeff = [];
            strDynParamsCentral.strMainData.ui16MaxSHdegree = uint16(0);
            [~, ~, dDynMatrixCentral] = ComputeRefDynFcn(0.0, dxState, strDynParamsCentral);

            testCase.verifyGreaterThan(norm(dDynMatrix - dDynMatrixCentral), 0.0);
            testCase.verifyEqual(dDynMatrix, dJacFD, 'RelTol', 2e-4, 'AbsTol', 5e-8);
        end

        function testPolyhedronGravityRHSAndJacobianMatchExactEvaluator(testCase)
            [ui32Faces, dVerts] = testCase.buildCubeMesh();
            [ui32Edges, dEe, dFf] = ComputePolyhedronFaceEdgeData(ui32Faces, dVerts);
            [dVolume, ~] = ComputeMeshModelVolumeAndCoM(ui32Faces, dVerts);

            dDensity = 2000.0;
            dGravConst = 6.67430e-11;
            dGravParam = dGravConst * dDensity * dVolume;

            strDynParams = testCase.buildCentralDynParams();
            strDynParams.strMainData.dGM = dGravParam;
            strDynParams.strMainData.dRefRadius = 1.0;
            strDynParams.strMainData.strPolyhedronGravityData = struct( ...
                'ui32FaceVertexIds', ui32Faces, ...
                'dVerticesPos', dVerts, ...
                'dDensity', dDensity, ...
                'ui32EdgeVertexIds', ui32Edges, ...
                'dEdgeDyadics', dEe, ...
                'dFaceDyadics', dFf, ...
                'dGravConst', dGravConst, ...
                'dGravParam', dGravParam);

            dxState = [4.0; 0.3; -0.2; 0.0; 0.0; 0.0];
            [dDxDt, strAccelInfo, dDynMatrix] = ComputeRefDynFcn(0.0, dxState, strDynParams);
            [dAccPolyTotal, dJacPolyTotal] = EvalPolyhedronGrav(dxState(1:3), ...
                ui32Faces, dVerts, dDensity, ui32Edges, dEe, dFf, dGravConst);

            testCase.verifyEqual(dDxDt(4:6), dAccPolyTotal, 'RelTol', 1e-12, 'AbsTol', 1e-18);
            testCase.verifyEqual(dDynMatrix(4:6, 1:3), dJacPolyTotal, 'RelTol', 1e-10, 'AbsTol', 1e-18);
            testCase.verifyGreaterThan(norm(strAccelInfo.dAccPolyhedronPert_IN), 0.0);
        end
    end

    methods (Access = private)
        function strDynParams = buildCentralDynParams(testCase)
            strDynParams = struct();
            strDynParams.strMainData = struct();
            strDynParams.strMainData.dGM = testCase.dGM;
            strDynParams.strMainData.dRefRadius = testCase.dRefRadius;
            strDynParams.strMainData.dSHcoeff = [];
            strDynParams.strMainData.ui16MaxSHdegree = uint16(0);
        end

        function dJacFD = finiteDifferenceJacobian(~, dxState, strDynParams)
            dStep = 1e-6 * norm(dxState(1:3));
            dJacFD = zeros(6, 6);

            for idxState = 1:6
                dPerturb = zeros(6, 1);
                dPerturb(idxState) = dStep;
                dDxPlus = ComputeRefDynFcn(0.0, dxState + dPerturb, strDynParams);
                dDxMinus = ComputeRefDynFcn(0.0, dxState - dPerturb, strDynParams);
                dJacFD(:, idxState) = (dDxPlus - dDxMinus) / (2.0 * dStep);
            end
        end

        function [ui32Faces, dVerts] = buildCubeMesh(~)
            dVerts = [-1 -1 -1; ...
                       1 -1 -1; ...
                       1  1 -1; ...
                      -1  1 -1; ...
                      -1 -1  1; ...
                       1 -1  1; ...
                       1  1  1; ...
                      -1  1  1];

            ui32Faces = uint32([1 4 3; ...
                                1 3 2; ...
                                5 6 7; ...
                                5 7 8; ...
                                2 7 6; ...
                                2 3 7; ...
                                1 5 8; ...
                                1 8 4; ...
                                4 7 3; ...
                                4 8 7; ...
                                1 2 6; ...
                                1 6 5]);
        end
    end
end
