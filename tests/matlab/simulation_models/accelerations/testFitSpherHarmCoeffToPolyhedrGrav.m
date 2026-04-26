classdef testFitSpherHarmCoeffToPolyhedrGrav < matlab.unittest.TestCase
    properties (Constant)
        dGravConst = 6.67430e-11; % [m^3/(kg*s^2)]
    end

    properties (TestParameter)
        ui32ExactRecoveryDegree = struct( ...
            'deg4', uint32(4), ...
            'deg8', uint32(8));
    end

    methods (Static)
        function ui32Count = GetCoeffCount(ui32MaxDegree)
            ui32Count = uint32((double(ui32MaxDegree + 1) * double(ui32MaxDegree + 2)) / 2 - 2);
        end

        function dCSlmCoeffCols = BuildRandomCoeffCols(ui32MaxDegree, dScale, dSeed)
            rng(dSeed);

            dCSlmCoeffCols = zeros(double(testFitSpherHarmCoeffToPolyhedrGrav.GetCoeffCount(ui32MaxDegree)), 2);

            idPair = uint32(1);
            for idxDeg = uint32(1):ui32MaxDegree
                for idxOrd = uint32(0):idxDeg
                    if idxDeg == uint32(1) && idxOrd == uint32(0)
                        continue;
                    end

                    if idxDeg >= uint32(2)
                        dCSlmCoeffCols(idPair, 1) = dScale * randn();
                        if idxOrd > uint32(0)
                            dCSlmCoeffCols(idPair, 2) = dScale * randn();
                        end
                    end

                    idPair = idPair + uint32(1);
                end
            end
        end

        function [ui32Faces, dVerts] = BuildUnitCube()
            dVerts = [ -1 -1 -1;
                        1 -1 -1;
                        1  1 -1;
                       -1  1 -1;
                       -1 -1  1;
                        1 -1  1;
                        1  1  1;
                       -1  1  1 ];

            ui32Faces = uint32([ ...
                1 3 2; 1 4 3;
                5 6 7; 5 7 8;
                1 2 6; 1 6 5;
                3 8 7; 3 4 8;
                1 8 4; 1 5 8;
                2 7 6; 2 3 7 ]);
        end

        function [ui32Faces, dVerts] = BuildRegularTetrahedron()
            dVerts = [ 1  1  1;
                       1 -1 -1;
                      -1  1 -1;
                      -1 -1  1 ];
            ui32Faces = uint32([1 2 3; 1 4 2; 1 3 4; 2 4 3]);
        end

        function charObjFilePath = WriteObjFileToTempDir(ui32Faces, dVerts, charFileStem)
            charObjFilePath = string(strcat(tempname, "_", charFileStem, ".obj"));
            objFileId = fopen(charObjFilePath, 'w');
            assert(objFileId > 0, 'testFitSpherHarmCoeffToPolyhedrGrav:TempObjOpenFailed', ...
                'Unable to create temporary .obj file.');
            cleanupObj = onCleanup(@() fclose(objFileId)); %#ok<NASGU>

            for idVert = 1:size(dVerts, 1)
                fprintf(objFileId, 'v %.16g %.16g %.16g\n', dVerts(idVert, 1), dVerts(idVert, 2), dVerts(idVert, 3));
            end
            for idFace = 1:size(ui32Faces, 1)
                fprintf(objFileId, 'f %u %u %u\n', ui32Faces(idFace, 1), ui32Faces(idFace, 2), ui32Faces(idFace, 3));
            end
        end

        function CloseFigureHandles(strFigureHandles)
            cellFieldNames = fieldnames(strFigureHandles);
            for idField = 1:numel(cellFieldNames)
                objFig = strFigureHandles.(cellFieldNames{idField});
                if isgraphics(objFig)
                    close(objFig);
                end
            end
        end
    end

    methods (Test)
        function testExactRecoveryFromSyntheticField(testCase, ui32ExactRecoveryDegree)
            dGravParam = 3.986004418e5;
            dBodyRadiusRef = 6378.137;
            dCoeffTrue = testFitSpherHarmCoeffToPolyhedrGrav.BuildRandomCoeffCols( ...
                ui32ExactRecoveryDegree, 1.0e-6, double(ui32ExactRecoveryDegree) + 10.0);

            ui32NumUnknowns = ui32ExactRecoveryDegree * ui32ExactRecoveryDegree + 2 * ui32ExactRecoveryDegree - uint32(3);
            ui32PtsPerShell = repmat(uint32(max(128, 3 * double(ui32NumUnknowns))), 1, 4);
            dTrainShellRadii = dBodyRadiusRef * [1.20, 1.45, 1.90, 2.60];

            [dTrainPos_TB, ~] = GenerateShellPointSet( ...
                dTrainShellRadii, ui32PtsPerShell, 0.0);
            [dTrainPotential, dTrainAccTB] = EvalExtSphHarmSamplesInTargetFrame( ...
                dTrainPos_TB, ui32ExactRecoveryDegree, dCoeffTrue, dGravParam, dBodyRadiusRef);

            [dCoeffFit, strFitInfo] = FitGravityFieldExtSHEcoefficients( ...
                dTrainPos_TB, dTrainPotential, dTrainAccTB, ...
                ui32ExactRecoveryDegree, dGravParam, dBodyRadiusRef);

            testCase.verifyEqual(dCoeffFit, dCoeffTrue, 'RelTol', 1e-10, 'AbsTol', 1e-14);
            testCase.verifyEqual(strFitInfo.ui32Rank, ui32NumUnknowns);
            testCase.verifyFalse(strFitInfo.bUsedLeastNorm);

            [dHoldoutPos_TB, ~] = GenerateShellPointSet( ...
                dBodyRadiusRef * [1.35, 1.75, 2.35], uint32([96 96 96]), 7.0);
            [dHoldoutPotentialTrue, dHoldoutAccTrueTB] = EvalExtSphHarmSamplesInTargetFrame( ...
                dHoldoutPos_TB, ui32ExactRecoveryDegree, dCoeffTrue, dGravParam, dBodyRadiusRef);
            [dHoldoutPotentialFit, dHoldoutAccFitTB] = EvalExtSphHarmSamplesInTargetFrame( ...
                dHoldoutPos_TB, ui32ExactRecoveryDegree, dCoeffFit, dGravParam, dBodyRadiusRef);

            testCase.verifyEqual(dHoldoutPotentialFit, dHoldoutPotentialTrue, 'RelTol', 1e-10, 'AbsTol', 1e-14);
            testCase.verifyEqual(dHoldoutAccFitTB, dHoldoutAccTrueTB, 'RelTol', 1e-10, 'AbsTol', 1e-14);
        end

        function testPolyhedronFitImprovesWithDegree(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildUnitCube();
            dDensity = 2000.0;

            strSHdeg4 = FitSpherHarmCoeffToPolyhedrGrav( ...
                ui32Faces, dVerts, uint32(4), ...
                NaN, dDensity, testCase.dGravConst, NaN, uint32(4));
            strSHdeg8 = FitSpherHarmCoeffToPolyhedrGrav( ...
                ui32Faces, dVerts, uint32(8), ...
                NaN, dDensity, testCase.dGravConst, NaN, uint32(4));

            dHoldoutShellRadii = max(vecnorm(dVerts, 2, 2)) * [1.35, 1.90, 2.60];
            ui32HoldoutPtsPerShell = uint32([96 96 96]);

            strDiagnosticsDeg4 = ComputePolyhedronGravitySHfitDiagnostics( ...
                ui32Faces, dVerts, strSHdeg4, ...
                dHoldoutShellRadii=dHoldoutShellRadii, ...
                ui32HoldoutPtsPerShell=ui32HoldoutPtsPerShell);
            strDiagnosticsDeg8 = ComputePolyhedronGravitySHfitDiagnostics( ...
                ui32Faces, dVerts, strSHdeg8, ...
                dHoldoutShellRadii=dHoldoutShellRadii, ...
                ui32HoldoutPtsPerShell=ui32HoldoutPtsPerShell);

            testCase.verifyLessThan(strDiagnosticsDeg8.strMetrics.dAccRMSrel, ...
                strDiagnosticsDeg4.strMetrics.dAccRMSrel);
            testCase.verifyLessThan(strDiagnosticsDeg8.strMetrics.dPotentialRMSrel, ...
                strDiagnosticsDeg4.strMetrics.dPotentialRMSrel);
            testCase.verifyLessThan(strDiagnosticsDeg8.strMetrics.dAccRMSrel, 0.10);
            testCase.verifyLessThan(strDiagnosticsDeg8.strMetrics.dPotentialRMSrel, 0.08);
            testCase.verifyGreaterThanOrEqual(strSHdeg8.strFitStats.ui32NumIterations, uint32(1));
        end

        function testPolyhedronFitRejectsNonCenteredMesh(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildRegularTetrahedron();
            dVerts(:, 1) = dVerts(:, 1) + 0.1;

            testCase.verifyError(@() FitSpherHarmCoeffToPolyhedrGrav( ...
                ui32Faces, dVerts, uint32(4), ...
                NaN, 2500.0, testCase.dGravConst, NaN, uint32(5)), ...
                'FitSpherHarmCoeffToPolyhedrGrav:NonCenteredMesh');
        end

        function testPolyhedronFitRejectsInconsistentMassInputs(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildUnitCube();

            testCase.verifyError(@() FitSpherHarmCoeffToPolyhedrGrav( ...
                ui32Faces, dVerts, uint32(4), ...
                1.0, 2000.0, testCase.dGravConst, NaN, uint32(5)), ...
                'FitSpherHarmCoeffToPolyhedrGrav:InconsistentMassInputs');
        end

        function testCShapeModelSphericalHarmonicsPipe(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildRegularTetrahedron();
            strShape = struct( ...
                'ui32triangVertexPtr', ui32Faces.', ...
                'dVerticesPos', dVerts.');

            objShapeModel = CShapeModel("struct", strShape, "m", "m", true, 'unit_test_shape', true);
            strSHgravityData = CShapeModel.BuildSphericalHarmonicsGravityData( ...
                objShapeModel, uint32(4), ...
                dDensity=2500.0, dGravConst=testCase.dGravConst, ...
                ui32MaxFitIterations=uint32(3));

            objShapeModel = objShapeModel.setSphericalHarmonicsGravityData(strSHgravityData);
            strCachedSHgravityData = objShapeModel.getSphericalHarmonicsGravityData();

            testCase.verifyEqual(strCachedSHgravityData.dCSlmCoeffCols, strSHgravityData.dCSlmCoeffCols, ...
                'RelTol', 0.0, 'AbsTol', 0.0);
            testCase.verifyEqual(strCachedSHgravityData.ui32MaxDegree, strSHgravityData.ui32MaxDegree);
            testCase.verifyEqual(strCachedSHgravityData.dGravParam, strSHgravityData.dGravParam, 'RelTol', 1e-14);
            testCase.verifyEqual(strCachedSHgravityData.dBodyRadiusRef, strSHgravityData.dBodyRadiusRef, 'RelTol', 1e-14);

            dTestPos = [4.0; -1.5; 2.0];
            [dPotentialEval, dAccEvalTB] = EvalExtSphHarmExpInTargetFrame( ...
                dTestPos, strCachedSHgravityData.ui32MaxDegree, strCachedSHgravityData.dCSlmCoeffCols, ...
                strCachedSHgravityData.dGravParam, strCachedSHgravityData.dBodyRadiusRef);

            testCase.verifyTrue(all(isfinite(dAccEvalTB)));
            testCase.verifyTrue(isfinite(dPotentialEval));
        end

        function testExampleFitPolyhedronGravitySHfromObjSmoke(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildRegularTetrahedron();
            charObjFilePath = testFitSpherHarmCoeffToPolyhedrGrav.WriteObjFileToTempDir( ...
                ui32Faces, dVerts, 'shape_model_fit_example');
            cleanupObj = onCleanup(@() delete(charObjFilePath)); %#ok<NASGU>

            [objShapeModel, strSHgravityData] = FitSpherHarmonicsToPolyhedronGravityFromObj( ...
                charObjFilePath, uint32(4), ...
                dDensity=2500.0, ...
                dGravConst=testCase.dGravConst, ...
                ui32MaxFitIterations=uint32(3), ...
                bCacheOnShapeModel=true);

            strCachedSHgravityData = objShapeModel.getSphericalHarmonicsGravityData();

            testCase.verifyEqual(strCachedSHgravityData.dCSlmCoeffCols, ...
                strSHgravityData.dCSlmCoeffCols, 'RelTol', 0.0, 'AbsTol', 0.0);
            testCase.verifyEqual(strCachedSHgravityData.ui32MaxDegree, strSHgravityData.ui32MaxDegree);
            testCase.verifyTrue(objShapeModel.hasData());
        end

        function testRunFitSpherHarmonicsToPolyhedronGravityFromObjSmoke(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildRegularTetrahedron();
            charObjFilePath = testFitSpherHarmCoeffToPolyhedrGrav.WriteObjFileToTempDir( ...
                ui32Faces, dVerts, 'shape_model_runfit');
            cleanupObj = onCleanup(@() delete(charObjFilePath)); %#ok<NASGU>

            strRunOutputs = RunFitSpherHarmonicsToPolyhedronGravityFromObj( ...
                charObjFilePath, uint32(4), ...
                dDensity=2500.0, ...
                dGravConst=testCase.dGravConst, ...
                ui32MaxFitIterations=uint32(3), ...
                bShowMeshFigure=false, ...
                bShowConvergenceFigure=false, ...
                bShowHoldoutFigure=false, ...
                bVerbose=false);

            strCachedSHgravityData = strRunOutputs.objShapeModel.getSphericalHarmonicsGravityData();

            testCase.verifyEqual(strCachedSHgravityData.dCSlmCoeffCols, ...
                strRunOutputs.strSHgravityData.dCSlmCoeffCols, 'RelTol', 0.0, 'AbsTol', 0.0);
            testCase.verifyTrue(all(isfinite(strRunOutputs.strDiagnostics.dAccSHEpertTB), 'all'));
            testCase.verifyTrue(all(isfinite(strRunOutputs.strDiagnostics.dAccPolyPertTB), 'all'));
            testCase.verifyLessThan(strRunOutputs.strDiagnostics.strMetrics.dAccRMSrel, 0.25);
            testCase.verifyLessThan(strRunOutputs.strDiagnostics.strMetrics.dPotentialRMSrel, 0.20);
            testCase.verifyFalse(isgraphics(strRunOutputs.strFigureHandles.objMeshFig));
            testCase.verifyFalse(isgraphics(strRunOutputs.strFigureHandles.objConvergenceFig));
            testCase.verifyFalse(isgraphics(strRunOutputs.strFigureHandles.objHoldoutFig));
        end

        function testPlotPolyhedronSHfitDiagnosticsSmoke(testCase)
            [ui32Faces, dVerts] = testFitSpherHarmCoeffToPolyhedrGrav.BuildRegularTetrahedron();
            strShape = struct( ...
                'ui32triangVertexPtr', ui32Faces.', ...
                'dVerticesPos', dVerts.');

            strSHgravityData = FitSpherHarmCoeffToPolyhedrGrav( ...
                ui32Faces, dVerts, uint32(4), ...
                NaN, 2500.0, testCase.dGravConst, NaN, uint32(3));
            strDiagnostics = ComputePolyhedronGravitySHfitDiagnostics( ...
                ui32Faces, dVerts, strSHgravityData, ...
                ui32NumHoldoutShells=uint32(3), ...
                ui32HoldoutPtsPerShell=uint32(48));

            strFigureHandles = PlotPolyhedronSHfitDiagnostics( ...
                strShape, strSHgravityData, strDiagnostics, ...
                bShowMeshFigure=true, ...
                bShowConvergenceFigure=true, ...
                bShowHoldoutFigure=true);
            cleanupFigs = onCleanup(@() testFitSpherHarmCoeffToPolyhedrGrav.CloseFigureHandles(strFigureHandles)); %#ok<NASGU>

            testCase.verifyTrue(isgraphics(strFigureHandles.objMeshFig, 'figure'));
            testCase.verifyTrue(isgraphics(strFigureHandles.objConvergenceFig, 'figure'));
            testCase.verifyTrue(isgraphics(strFigureHandles.objHoldoutFig, 'figure'));
        end

        function testWorldWrapperMatchesExplicitRotation(testCase)
            ui32MaxDegree = uint32(6);
            dCoeffCols = testFitSpherHarmCoeffToPolyhedrGrav.BuildRandomCoeffCols(ui32MaxDegree, 1.0e-6, 23.0);
            dGravParam = 4.282837e4;
            dBodyRadiusRef = 3396.19;

            dAngle = 0.37;
            dDCM_WfromTB = [cos(dAngle) -sin(dAngle) 0.0; ...
                            sin(dAngle)  cos(dAngle) 0.0; ...
                            0.0          0.0         1.0];
            dPosSC_W = [4200.0; -1800.0; 900.0];
            dPosSC_TB = dDCM_WfromTB' * dPosSC_W;

            [dPotentialTB, dAccTB] = EvalExtSphHarmExpInTargetFrame( ...
                dPosSC_TB, ui32MaxDegree, dCoeffCols, dGravParam, dBodyRadiusRef);
            [dPotentialW, dAccW, dAccTBfromW, dPosTBfromW] = EvalExtSphHarmExpInWorldFrame( ...
                dPosSC_W, dDCM_WfromTB, ui32MaxDegree, dCoeffCols, dGravParam, dBodyRadiusRef);

            testCase.verifyEqual(dPotentialW, dPotentialTB, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dAccTBfromW, dAccTB, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dAccW, dDCM_WfromTB * dAccTB, 'RelTol', 1e-12, 'AbsTol', 1e-15);
            testCase.verifyEqual(dPosTBfromW, dPosSC_TB, 'RelTol', 1e-15, 'AbsTol', 1e-15);
        end
    end
end
