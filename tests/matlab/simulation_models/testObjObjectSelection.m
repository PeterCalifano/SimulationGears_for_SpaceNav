classdef testObjObjectSelection < matlab.unittest.TestCase
    %% DESCRIPTION
    % Verify exact OBJ selection through shape loading, repair and gravity preparation.
    % Use synthetic geometry to check ordering, winding, units, auxiliary indices and errors.
    % -----------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 29-09-2026  Pietro Califano, Codex gpt-6    Review parser and selected-gravity contracts.
    % -----------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CShapeModel, LoadShapeMesh, DefineShapeModel, GenerateScenarioSHGravityCoefficients,
    % CShapeModel.BuildSphericalHarmonicsGravityDataFromObj, ComputeMeshModelVolumeAndCoM
    % -----------------------------------------------------------------------------------------------------

    methods (Test)
        function TestIndentationPreservesDefaultAndSelectedGeometry(self)
            %% SIGNATURE
            % TestIndentationPreservesDefaultAndSelectedGeometry(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve face order, winding and geometry for mixed indentation with selection and repair.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Cover indentation in both loading paths.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh, ComputeMeshModelVolumeAndCoM
            % -------------------------------------------------------------------------------------------------
            % Retain every tetrahedron record in both fast-reader entry paths and repair modes.
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objFixture.Folder), "indented.obj");
            self.WriteText_(charObjPath, sprintf([ ...
                'v 0 0 0\n  v 1 0 0\n\tv 0 1 0\nv 0 0 1\n  o surface\n', ...
                'f 1 3 2\n f 1 2 4\n\tf 2 3 4\nf 1 4 3\n']));
            dExpectedVertices = [0 0 0;1 0 0;0 1 0;0 0 1];
            ui32ExpectedFaces = uint32([1 3 2;1 2 4;2 3 4;1 4 3]);
            for bRepair = [false, true]
                for bSelect = [false, true]
                    charSelection = strings(1,0);
                    if bSelect
                        charSelection = "surface";
                    end
                    strMesh = LoadShapeMesh(char(charObjPath), bRepairMesh=bRepair, ...
                        charObjObjectNames=charSelection);
                    self.verifySize(strMesh.dVerticesPos, [4,3]);
                    self.verifySize(strMesh.ui32FaceVertexIds, [4,3]);
                    if ~bRepair
                        self.verifyEqual(strMesh.dVerticesPos, dExpectedVertices);
                        self.verifyEqual(strMesh.ui32FaceVertexIds, ui32ExpectedFaces);
                    end

                    % Permit repair to renumber vertices while retaining each ordered triangle.
                    for ui32Corner = uint32(1):uint32(3)
                        self.verifyEqual( ...
                            strMesh.dVerticesPos(strMesh.ui32FaceVertexIds(:,ui32Corner),:), ...
                            dExpectedVertices(ui32ExpectedFaces(:,ui32Corner),:));
                    end
                    [dVolume, dCentroid] = ComputeMeshModelVolumeAndCoM( ...
                        strMesh.ui32FaceVertexIds, strMesh.dVerticesPos);
                    self.verifyEqual(dVolume, 1.0/6.0, AbsTol=1e-14);
                    self.verifyEqual(dCentroid, [0.25;0.25;0.25], AbsTol=1e-14);
                end
            end
        end

        function TestIndentedFallbackFacesRemainInDefaultMesh(self)
            %% SIGNATURE
            % TestIndentedFallbackFacesRemainInDefaultMesh(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Retain indented slash-index faces when the complete file requires the general reader.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Cover indented fallback records.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh
            % -------------------------------------------------------------------------------------------------
            % Include a slash-index face after ordinary faces so a partial fast result cannot pass.
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objFixture.Folder), "fallback.obj");
            self.WriteText_(charObjPath, sprintf([ ...
                'v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n\to surface\n', ...
                'f 1 3 2\nf 1 2 4\n  f 2/1 3/2 4/3\nf 1 4 3\n']));
            for bSelect = [false, true]
                charSelection = strings(1,0);
                if bSelect
                    charSelection = "surface";
                end
                strMesh = LoadShapeMesh(char(charObjPath), bRepairMesh=false, ...
                    charObjObjectNames=charSelection);
                self.verifyEqual(strMesh.ui32FaceVertexIds, uint32([1 3 2;1 2 4;2 3 4;1 4 3]));
            end
        end

        function TestRepeatedObjectSelectionIgnoresGroupChanges(self)
            %% SIGNATURE
            % TestRepeatedObjectSelectionIgnoresGroupChanges(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Combine repeated object ranges and requested unions in source face order.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel.LoadModelFromObj, LoadShapeMesh
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [charObjPath, ui32Faces, dVertices_m] = self.WriteTwoBodies_(objFixture.Folder);
            [ui32AllFaces, dAllVertices] = CShapeModel.LoadModelFromObj(charObjPath);
            self.verifySize(ui32AllFaces, [3, 8]);
            self.verifySize(dAllVertices, [3, 8]);

            [ui32SelectedFaces, dSelectedVertices] = CShapeModel.LoadModelFromObj( ...
                charObjPath, charObjObjectNames="surface");
            self.verifyEqual(ui32SelectedFaces, ui32Faces.');
            self.verifyEqual(dSelectedVertices, dVertices_m.');
            strSelected = LoadShapeMesh(char(charObjPath), ...
                bRepairMesh=false, charObjObjectNames="surface");
            self.verifyEqual(strSelected.ui32FaceVertexIds, ui32Faces);
            self.verifyEqual(strSelected.dVerticesPos, dVertices_m);
            self.verifyEqual(strSelected.charObjObjectNames, "surface");

            % Retain source face and vertex order regardless of the caller's selection order.
            [ui32UnionFaces, dUnionVertices] = CShapeModel.LoadModelFromObj(charObjPath, ...
                charObjObjectNames=["boulders_class1", "surface"]);
            self.verifyEqual(ui32UnionFaces, ui32AllFaces);
            self.verifyEqual(dUnionVertices, dAllVertices);
        end

        function TestSelectionPrecedesRepairAndMetricRadiusEvaluation(self)
            %% SIGNATURE
            % TestSelectionPrecedesRepairAndMetricRadiusEvaluation(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Exclude distant geometry before repair, volume/radius calculation and unit conversion.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel, LoadShapeMesh, ComputeMeshModelVolumeAndCoM
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [charObjPath, ui32Faces, dVertices_m] = self.WriteTwoBodies_(objFixture.Folder);
            for bRepairMesh = [false, true]
                strSelected = LoadShapeMesh(char(charObjPath), ...
                    bRepairMesh=bRepairMesh, charObjObjectNames="surface");
                [dVolume_m3, dCentroid_m] = ComputeMeshModelVolumeAndCoM( ...
                    strSelected.ui32FaceVertexIds, strSelected.dVerticesPos);
                [dExpectedVolume_m3, dExpectedCentroid_m] = ...
                    ComputeMeshModelVolumeAndCoM(ui32Faces, dVertices_m);
                self.verifyEqual(dVolume_m3, dExpectedVolume_m3, RelTol=1e-13);
                self.verifyEqual(dCentroid_m, dExpectedCentroid_m, AbsTol=1e-11);
                self.verifyEqual(strSelected.ui32NumFaces, uint32(4));
            end
            for charMethod = ["file_obj", "file_mesh"]
                objShape = CShapeModel(charMethod, charObjPath, "m", "km", true, ...
                    "selected", true, charObjObjectNames="surface");
                self.verifyEqual(objShape.ui32NumOfVertices, uint32(4));
                self.verifyEqual(objShape.dShapeRadius, mean(vecnorm(dVertices_m, 2, 2)) / 1000, ...
                    RelTol=1e-14);
                self.verifyEqual(max(vecnorm(objShape.dVerticesPos, 2, 1)), ...
                    max(vecnorm(dVertices_m, 2, 2)) / 1000, RelTol=1e-14);
                [~, dCentroid_km] = ComputeMeshModelVolumeAndCoM( ...
                    objShape.ui32triangVertexPtr.', objShape.dVerticesPos.');
                self.verifyEqual(dCentroid_km, dExpectedCentroid_m / 1000, AbsTol=1e-13);
            end
        end

        function TestSelectionRetainsIndependentTextureAndNormalIndices(self)
            %% SIGNATURE
            % TestSelectionRetainsIndependentTextureAndNormalIndices(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compact vertex indices while retaining separately indexed texture and normal corners.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel.LoadModelFromObj
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objFixture.Folder), "auxiliary.obj");
            self.WriteText_(charObjPath, sprintf([ ...
                'v 9 9 9\nv 0 0 0\nv 1 0 0\nv 0 1 0\n', ...
                'vt 0.2 0.3\nvt 0.4 0.5\nvt 0.6 0.7\n', ...
                'vn 0 0 1\nvn 0 1 0\no surface\nf 2/3/2 3/1/1 4/2/2\n']));
            [ui32Faces, dVertices, dUv, ui32Uv, dNormals, ui32Normals] = ...
                CShapeModel.LoadModelFromObj(charObjPath, false, charObjObjectNames="surface");
            self.verifyEqual(ui32Faces, uint32([1; 2; 3]));
            self.verifyEqual(dVertices, [0 1 0; 0 0 1; 0 0 0]);
            self.verifyEqual(dUv, [0.2 0.4 0.6; 0.3 0.5 0.7]);
            self.verifyEqual(ui32Uv, uint32([3; 1; 2]));
            self.verifyEqual(dNormals, [0 0; 0 1; 1 0]);
            self.verifyEqual(ui32Normals, uint32([2; 1; 2]));
        end

        function TestGeneralReaderSelectsBeforeParsingRelativePolygons(self)
            %% SIGNATURE
            % TestGeneralReaderSelectsBeforeParsingRelativePolygons(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Skip excluded face payloads and resolve selected relative polygons against source vertices.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % LoadShapeMesh
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objFixture.Folder), "polygon.obj");
            self.WriteText_(charObjPath, sprintf([ ...
                'v 0 0 0\nv 2 0 0\nv 2 2 0\nv 0 2 0\n', ...
                'o boulders_class1\nf invalid excluded face\no surface\n', ...
                '  f -4 -3 -2 -1\nv 1e12 0 0\n']));
            strSelected = LoadShapeMesh(char(charObjPath), ...
                bRepairMesh=false, charObjObjectNames="surface");
            self.verifyEqual(strSelected.ui32NumVertices, uint32(4));
            self.verifyEqual(strSelected.ui32NumFaces, uint32(2));
            self.verifyEqual(max(strSelected.dVerticesPos, [], 'all'), 2.0);
            dFirst = strSelected.dVerticesPos(strSelected.ui32FaceVertexIds(:, 1), :);
            dSecond = strSelected.dVerticesPos(strSelected.ui32FaceVertexIds(:, 2), :);
            dThird = strSelected.dVerticesPos(strSelected.ui32FaceVertexIds(:, 3), :);
            self.verifyEqual(sum(vecnorm(cross(dSecond - dFirst, dThird - dFirst, 2), 2, 2)) / 2, 4.0);
        end

        function TestUnnamedObjectsAndMissingSelectionsHaveExplicitSemantics(self)
            %% SIGNATURE
            % TestUnnamedObjectsAndMissingSelectionsHaveExplicitSemantics(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Select unnamed faces and reject missing names or incomplete requested unions.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel.LoadModelFromObj, LoadShapeMesh
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objFixture.Folder), "unnamed.obj");
            self.WriteText_(charObjPath, sprintf([ ...
                'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n', ...
                'o surface\nf 3 2 1\no\nf 1 3 2\no empty\n']));
            [ui32Faces, ~] = CShapeModel.LoadModelFromObj(charObjPath, charObjObjectNames="");
            self.verifyEqual(ui32Faces, uint32([1 1; 2 3; 3 2]));
            for charMissingName = ["Surface", "missing", "empty"]
                self.verifyError(@() CShapeModel.LoadModelFromObj(charObjPath, ...
                    charObjObjectNames=charMissingName), 'SelectObjFaceRecords:MissingObjects');
                self.verifyError(@() LoadShapeMesh(char(charObjPath), ...
                    charObjObjectNames=charMissingName), 'SelectObjFaceRecords:MissingObjects');
            end
            self.verifyError(@() CShapeModel.LoadModelFromObj(charObjPath, ...
                charObjObjectNames=["surface", "missing"]), 'SelectObjFaceRecords:MissingObjects');
        end

        function TestSelectedFaceRunsCrossBoundedParserBlocks(self)
            %% SIGNATURE
            % TestSelectedFaceRunsCrossBoundedParserBlocks(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Preserve selected face runs and ordering across bounded decoder blocks.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel.LoadModelFromObj
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objFixture.Folder), "long_selected_run.obj");
            dNumFaces = 300001;
            self.WriteText_(charObjPath, [sprintf('v 0 0 0\nv 1 0 0\nv 0 1 0\no surface\n'), ...
                repmat(sprintf('f 1 2 3\n'), 1, dNumFaces), ...
                sprintf('o excluded\nf 1 3 2\no surface\nf 3 2 1\n')]);
            [ui32Faces, dVertices] = CShapeModel.LoadModelFromObj(charObjPath, ...
                charObjObjectNames="surface");
            self.verifySize(ui32Faces, [3, dNumFaces + 1]);
            self.verifyEqual(ui32Faces(:, 1:dNumFaces), repmat(uint32([1; 2; 3]), 1, dNumFaces));
            self.verifyEqual(ui32Faces(:, end), uint32([3; 2; 1]));
            self.verifySize(dVertices, [3, 3]);
        end

        function TestObjectSelectionCannotBeSilentlyAppliedToOtherFormats(self)
            %% SIGNATURE
            % TestObjectSelectionCannotBeSilentlyAppliedToOtherFormats(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Reject object selection for STL and non-OBJ constructor sources.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel, LoadShapeMesh
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charStlPath = fullfile(string(objFixture.Folder), "empty.stl");
            self.WriteText_(charStlPath, sprintf('solid empty\nendsolid empty\n'));
            self.verifyError(@() LoadShapeMesh(char(charStlPath), charObjObjectNames="surface"), ...
                'LoadShapeMesh:ObjectSelectionRequiresObj');
            self.verifyError(@() CShapeModel("struct", struct(), "m", "m", true, "", false, ...
                charObjObjectNames="surface"), 'CShapeModel:ObjectSelectionRequiresObj');
        end

        function TestShapeBuilderForwardsSelectionAndRecordsIt(self)
            %% SIGNATURE
            % TestShapeBuilderForwardsSelectionAndRecordsIt(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Forward exact names through the custom-shape builder and record its units and selection.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % DefineShapeModel
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [charObjPath, ~, dVertices_m] = self.WriteTwoBodies_(objFixture.Folder);
            [objShape, ~, strMetadata] = DefineShapeModel("FromShape", ...
                string(objFixture.Folder), charShapeModelObjPath=charObjPath, ...
                charShapeModelInputUnits="m", charOutputLengthUnits="km", ...
                charObjObjectNames="surface", bInitSphericalHarmonicsGravityData=false);
            self.verifyEqual(objShape.dVerticesPos, dVertices_m.' / 1000);
            self.verifyEqual(strMetadata.charObjObjectNames, "surface");
            self.verifyEqual(strMetadata.charOutputLengthUnits, "km");
        end

        function TestSelectedObjGravityFitMatchesTheSelectedSolid(self)
            %% SIGNATURE
            % TestSelectedObjGravityFitMatchesTheSelectedSolid(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Compare the selected-object builder and diagnostic entry point with the same solid
            % passed directly to the fitter.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CShapeModel.BuildSphericalHarmonicsGravityDataFromObj
            % RunFitSpherHarmonicsToPolyhedronGravityFromObj, FitSpherHarmCoeffToPolyhedrGrav
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [charObjPath, ui32Faces, dVertices_m] = self.WriteTwoBodies_(objFixture.Folder, [0 0 0]);

            % Fit only the selected solid and compare with its isolated geometry.
            [objShape, strFit] = CShapeModel.BuildSphericalHarmonicsGravityDataFromObj( ...
                charObjPath, uint32(2), charObjObjectNames="surface", dDensity=2000, ...
                ui32MaxFitIterations=uint32(1), bCacheOnShapeModel=false);
            strReference = FitSpherHarmCoeffToPolyhedrGrav(ui32Faces, dVertices_m, uint32(2), ...
                NaN, 2000, 6.67430e-11, NaN, uint32(1));
            self.verifyEqual(objShape.ui32NumOfVertices, uint32(4));
            self.verifyEqual(strFit.dGravParam, strReference.dGravParam, RelTol=1e-13);
            self.verifyEqual(strFit.dBodyRadiusRef, strReference.dBodyRadiusRef, RelTol=1e-13);
            self.verifyEqual(strFit.dCSlmCoeffCols, strReference.dCSlmCoeffCols, AbsTol=1e-12);

            % Preserve selection through the diagnostic workflow and evaluate exterior holdout points.
            strRunOutputs = RunFitSpherHarmonicsToPolyhedronGravityFromObj( ...
                charObjPath, uint32(2), charObjObjectNames="surface", dDensity=2000, ...
                ui32MaxFitIterations=uint32(1), bCacheOnShapeModel=false, ...
                dHoldoutShellRadii=2 * strFit.dBodyRadiusRef, ui32HoldoutPtsPerShell=uint32(32), ...
                bShowMeshFigure=false, bShowConvergenceFigure=false, ...
                bShowHoldoutFigure=false, bVerbose=false);
            self.verifyEqual(strRunOutputs.objShapeModel.ui32NumOfVertices, uint32(4));
            self.verifyEqual(strRunOutputs.strSHgravityData.dCSlmCoeffCols, ...
                strReference.dCSlmCoeffCols, AbsTol=1e-12);
            self.verifyTrue(all(isfinite(strRunOutputs.strDiagnostics.dAccSHEpertTB), 'all'));
            self.verifyTrue(all(isfinite(strRunOutputs.strDiagnostics.dAccPolyPertTB), 'all'));
        end

        function TestScenarioGeneratorKeepsRegistryGravityAndMacroSelection(self)
            %% SIGNATURE
            % TestScenarioGeneratorKeepsRegistryGravityAndMacroSelection(self)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Retain selected geometry, centering and registered gravity normalization in generation evidence.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % self    MATLAB test case with temporary fixture support.
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % None. Report failures through the test framework.
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 29-09-2026  Pietro Califano, Codex gpt-6    Review the functional contract.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % GenerateScenarioSHGravityCoefficients, ResolveTargetAssetBundle, CScenarioRegistry
            % -------------------------------------------------------------------------------------------------
            objFixture = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charRepositoryRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            charPreviousPath = path;
            objPathCleanup = onCleanup(@() path(charPreviousPath)); %#ok<NASGU>
            addpath(fullfile(charRepositoryRoot, 'examples', 'matlab', 'shape_models'));
            charDataRoot = fullfile(charRepositoryRoot, 'data');
            % Resolve a disposable payload using the public API and materialize synthetic geometry.
            % Keep source units explicit so the fixture exercises the real conversion path.
            objBundle = ResolveTargetAssetBundle("Apophis", charDataRootPath=charDataRoot, ...
                charAssetRootPath=string(objFixture.Folder), charAppearanceProfileId="uniform", ...
                bRequireFiles=false);
            dFileLengthUnit_m = 1;
            if EnumLengthUnits.toString(objBundle.enumShapeInputUnits) == "km"
                dFileLengthUnit_m = 1000;
            end
            charObjPath = self.WriteTwoBodies_(objFixture.Folder, [10 20 30], dFileLengthUnit_m);
            mkdir(fileparts(objBundle.charShapePath));
            copyfile(charObjPath, objBundle.charShapePath);
            strGeneration = GenerateScenarioSHGravityCoefficients("Apophis", uint32(2), ...
                charDataRoot, charObjObjectNames="surface", ...
                charAssetRootPath=string(objFixture.Folder), charAppearanceProfileId="uniform", ...
                bVerbose=false, bPrintCoefficientRows=false);
            strSpec = CScenarioRegistry.GetScenarioSpec("Apophis", charLengthUnits="km");
            self.verifyEqual(strGeneration.strMesh.ui32NumFaces, uint32(4));
            self.verifyEqual(strGeneration.strMesh.ui32NumVertices, uint32(4));
            self.verifyEqual(strGeneration.strMesh.charObjObjectNames, "surface");
            self.verifyEqual(strGeneration.strMesh.charLengthUnits, "km");
            self.verifyEqual(strGeneration.strMesh.dOriginalCOM, [0.01; 0.02; 0.03], AbsTol=1e-13);
            self.verifyLessThan(norm(strGeneration.strMesh.dCenteredCOM), 1e-13);
            self.verifyEqual(strGeneration.strGeneratedModel.dGravParam, strSpec.dGravParam);
            self.verifyEqual(strGeneration.strGeneratedModel.dBodyRadiusRef, strSpec.dReferenceRadius);
        end
    end

    methods (Static, Access = private)
        function [charObjPath, ui32Faces, dVertices_m] = ...
                WriteTwoBodies_(charDirectory, dCenter_m, dFileLengthUnit_m)
            % Write two disjoint solids with repeated object declarations and misleading groups.
            arguments (Input)
                charDirectory (1,:) char
                dCenter_m (1,3) double = [10 20 30]
                dFileLengthUnit_m (1,1) double = 1
            end
            arguments (Output)
                charObjPath (1,1) string
                ui32Faces (:,3) uint32
                dVertices_m (:,3) double
            end

            charObjPath = fullfile(string(charDirectory), "two_bodies.obj");
            dVertices_m = 100 * [1 1 1; 1 -1 -1; -1 1 -1; -1 -1 1] + dCenter_m;
            ui32Faces = uint32([1 2 3; 1 4 2; 1 3 4; 2 4 3]);
            i32FileId = fopen(charObjPath, 'w');
            assert(i32FileId >= 0);
            objCleanup = onCleanup(@() fclose(i32FileId)); %#ok<NASGU>
            fprintf(i32FileId, 'v %.17g %.17g %.17g\n', ...
                [dVertices_m; dVertices_m + 1e12].' / dFileLengthUnit_m);
            fprintf(i32FileId, 'o surface # macro mesh\n');
            fprintf(i32FileId, 'f %u %u %u\n', ui32Faces(1:2, :).');
            fprintf(i32FileId, 'o boulders_class1\n');
            fprintf(i32FileId, 'f %u %u %u\n', (ui32Faces + uint32(4)).');
            fprintf(i32FileId, 'o surface\ng misleading_group\nusemtl ignored\n');
            fprintf(i32FileId, 'f %u %u %u\n', ui32Faces(3:4, :).');
        end

        function WriteText_(charPath, charText)
            % Write a complete synthetic OBJ payload with fixture-owned file cleanup.
            arguments (Input)
                charPath (1,1) string
                charText (1,:) char
            end

            i32FileId = fopen(charPath, 'w');
            assert(i32FileId >= 0);
            objCleanup = onCleanup(@() fclose(i32FileId)); %#ok<NASGU>
            fwrite(i32FileId, charText, 'char');
        end
    end
end
