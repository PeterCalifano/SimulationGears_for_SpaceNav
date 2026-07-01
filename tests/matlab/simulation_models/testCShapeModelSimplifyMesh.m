classdef testCShapeModelSimplifyMesh < matlab.unittest.TestCase
    methods (Test)
        function testConstructorAppliesLoadTimeKeepFraction(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModelFull = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true);
            objShapeModelReduced = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true, ...
                dMeshSimplifyFactor=0.65);

            testCase.verifyEqual(objShapeModelReduced.dMeshSimplifyFactor, 0.65, "AbsTol", 1e-12);
            testCase.verifyLessThan(size(objShapeModelReduced.ui32triangVertexPtr, 2), ...
                size(objShapeModelFull.ui32triangVertexPtr, 2));
            testCase.verifyLessThan(size(objShapeModelReduced.dVerticesPos, 2), ...
                size(objShapeModelFull.dVerticesPos, 2));
        end

        function testConstructorClampsKeepFractionToUnitInterval(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModelHigh = CShapeModel("file_obj", "", "m", "m", true, "placeholder", false, ...
                dMeshSimplifyFactor=1.7);
            objShapeModelLow = CShapeModel("file_obj", charObjPath, "m", "m", true, "placeholder", true, ...
                dMeshSimplifyFactor=-0.2);

            testCase.verifyEqual(objShapeModelHigh.dMeshSimplifyFactor, 1.0, "AbsTol", 1e-12);
            testCase.verifyEqual(objShapeModelLow.dMeshSimplifyFactor, 0.0, "AbsTol", 1e-12);
            testCase.verifyEqual(size(objShapeModelLow.ui32triangVertexPtr, 2), 0);
            testCase.verifyEqual(size(objShapeModelLow.dVerticesPos, 2), 0);
        end

        function testSimplifyMeshReducesGeometryLoadedFromObj(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModel = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true);

            ui32NumFacesBefore = uint32(size(objShapeModel.ui32triangVertexPtr, 2));
            ui32NumVertsBefore = uint32(size(objShapeModel.dVerticesPos, 2));

            [objShapeModel, strReductionStats] = objShapeModel.SimplifyMesh(35.0);

            ui32NumFacesAfter = uint32(size(objShapeModel.ui32triangVertexPtr, 2));
            ui32NumVertsAfter = uint32(size(objShapeModel.dVerticesPos, 2));

            testCase.verifyLessThan(ui32NumFacesAfter, ui32NumFacesBefore);
            testCase.verifyLessThan(ui32NumVertsAfter, ui32NumVertsBefore);
            testCase.verifyEqual(objShapeModel.ui32NumOfVertices, ui32NumVertsAfter);
            testCase.verifyClass(objShapeModel.ui32triangVertexPtr, "uint32");
            testCase.verifyGreaterThanOrEqual(min(objShapeModel.ui32triangVertexPtr, [], "all"), uint32(1));
            testCase.verifyLessThanOrEqual(max(objShapeModel.ui32triangVertexPtr, [], "all"), ui32NumVertsAfter);
            testCase.verifyGreaterThan(strReductionStats.dAchievedFaceReductionPercent, 20.0);
            testCase.verifyGreaterThan(strReductionStats.dAchievedVertexReductionPercent, 5.0);
        end

        function testObjLoaderParsesNormalOnlyFaceSyntaxWhenVertexFaceOnly(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "normal_only_faces.obj");

            fileId = fopen(charObjPath, "w");
            objCleanup = onCleanup(@() fclose(fileId));
            fprintf(fileId, "v 0 0 0\n");
            fprintf(fileId, "v 1 0 0\n");
            fprintf(fileId, "v 0 1 0\n");
            fprintf(fileId, "vn 0 0 1\n");
            fprintf(fileId, "f 1//1 2//1 3//1\n");
            clear objCleanup

            objShapeModel = CShapeModel("file_obj", charObjPath, "m", "m", true, "normal_only_faces", true);

            testCase.verifyEqual(objShapeModel.ui32triangVertexPtr, uint32([1; 2; 3]));
            testCase.verifyEqual(objShapeModel.ui32NumOfVertices, uint32(3));
        end

        function testSimplifyMeshInvalidatesPolyhedronGravityCache(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModel = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true);
            objShapeModel = objShapeModel.BuildPolyhedronGravityData();

            [ui32EdgeIds, dEdgeDyadics, dFaceDyadics] = objShapeModel.getPolyhedronGravityData();
            testCase.verifyNotEmpty(ui32EdgeIds);
            testCase.verifyNotEmpty(dEdgeDyadics);
            testCase.verifyNotEmpty(dFaceDyadics);

            objShapeModel = objShapeModel.SimplifyMesh(25.0);

            testCase.verifyError(@() objShapeModel.getPolyhedronGravityData(), "CShapeModel:NoGravityData");
        end

        function testDefineShapeModelPassesLoadTimeKeepFraction(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "moon_shape.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModelFull = DefineShapeModel("Moon", string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                dMeshSimplifyFactor=1.0);

            objShapeModelReduced = DefineShapeModel("Moon", string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                dMeshSimplifyFactor=0.55);

            testCase.verifyEqual(objShapeModelReduced.dMeshSimplifyFactor, 0.55, "AbsTol", 1e-12);
            testCase.verifyLessThan(size(objShapeModelReduced.ui32triangVertexPtr, 2), ...
                size(objShapeModelFull.ui32triangVertexPtr, 2));
            testCase.verifyLessThan(size(objShapeModelReduced.dVerticesPos, 2), ...
                size(objShapeModelFull.dVerticesPos, 2));
        end

        function testCspiceShapeLoadFailsClearlyWhenMiceIsUnavailable(testCase)
            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            testCShapeModelSimplifyMesh.removeMicePathEntries_();
            objEnvCleanup = testCShapeModelSimplifyMesh.preserveWorkspaceEnv_(); %#ok<NASGU>
            setenv("WS_SIMGEARS", "");
            setenv("WS_NAVSYS", "");

            testCase.verifyError(@() CShapeModel("cspice", "missing_shape.bds", "km", "m", ...
                true, "missing_shape", true), "CShapeModel:CSPICEUnavailable");
        end

        function testCspiceShapeLoadPrefersWsSimGearsMiceWhenAvailable(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charSimGearsRoot = fullfile(string(fixture.Folder), "simgears_root");
            charNavSysRoot = fullfile(string(fixture.Folder), "navsys_root");
            testCShapeModelSimplifyMesh.writeFakeMiceInstall_(charSimGearsRoot, "FakeMice:FromSimGears");
            testCShapeModelSimplifyMesh.writeFakeMiceInstall_(charNavSysRoot, "FakeMice:FromNavSys");

            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            testCShapeModelSimplifyMesh.removeMicePathEntries_();
            objEnvCleanup = testCShapeModelSimplifyMesh.preserveWorkspaceEnv_(); %#ok<NASGU>
            setenv("WS_SIMGEARS", char(charSimGearsRoot));
            setenv("WS_NAVSYS", char(charNavSysRoot));

            testCase.verifyError(@() CShapeModel("cspice", "fake_shape.bds", "km", "m", ...
                true, "fake_shape", true), "FakeMice:FromSimGears");
            testCase.verifyEqual(string(which("cspice_furnsh")), ...
                string(fullfile(charSimGearsRoot, "mice", "src", "mice", "cspice_furnsh.m")));
        end

        function testCspiceShapeLoadFallsBackToWsNavsysMice(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charNavSysRoot = fullfile(string(fixture.Folder), "navsys_root");
            testCShapeModelSimplifyMesh.writeFakeMiceInstall_(charNavSysRoot, "FakeMice:FromNavSys");

            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            testCShapeModelSimplifyMesh.removeMicePathEntries_();
            objEnvCleanup = testCShapeModelSimplifyMesh.preserveWorkspaceEnv_(); %#ok<NASGU>
            setenv("WS_SIMGEARS", "");
            setenv("WS_NAVSYS", char(charNavSysRoot));

            testCase.verifyError(@() CShapeModel("cspice", "fake_shape.bds", "km", "m", ...
                true, "fake_shape", true), "FakeMice:FromNavSys");
            testCase.verifyEqual(string(which("cspice_furnsh")), ...
                string(fullfile(charNavSysRoot, "mice", "src", "mice", "cspice_furnsh.m")));
        end
    end

    methods (Static, Access = private)
        function objCleanup = preserveWorkspaceEnv_()
            charOriginalSimGears = getenv("WS_SIMGEARS");
            charOriginalNavsys = getenv("WS_NAVSYS");
            objCleanup = onCleanup(@() testCShapeModelSimplifyMesh.restoreWorkspaceEnv_( ...
                charOriginalSimGears, charOriginalNavsys));
        end

        function restoreWorkspaceEnv_(charOriginalSimGears, charOriginalNavsys)
            setenv("WS_SIMGEARS", charOriginalSimGears);
            setenv("WS_NAVSYS", charOriginalNavsys);
        end

        function removeMicePathEntries_()
            cellPathEntries = string(strsplit(path, pathsep));
            for idxPath = 1:numel(cellPathEntries)
                charPathEntry = cellPathEntries(idxPath);
                if contains(charPathEntry, filesep + "mice" + filesep + "src" + filesep + "mice") || ...
                        contains(charPathEntry, filesep + "mice" + filesep + "lib")
                    rmpath(charPathEntry);
                end
            end
        end

        function writeFakeMiceInstall_(charWorkspaceRoot, charErrorId)
            charMiceSrcPath = fullfile(string(charWorkspaceRoot), "mice", "src", "mice");
            charMiceLibPath = fullfile(string(charWorkspaceRoot), "mice", "lib");
            mkdir(charMiceSrcPath);
            mkdir(charMiceLibPath);

            fileId = fopen(fullfile(charMiceSrcPath, "cspice_furnsh.m"), "w");
            assert(fileId ~= -1, "testCShapeModelSimplifyMesh:FileOpenFailed", ...
                "Failed to create fake cspice_furnsh.m");
            objCleanup = onCleanup(@() fclose(fileId));
            fprintf(fileId, "function cspice_furnsh(varargin)\n");
            fprintf(fileId, "error('%s', 'fake MICE reached');\n", charErrorId);
            fprintf(fileId, "end\n");
            clear objCleanup
        end

        function [ui32Faces, dVerts] = buildIcosphere_(nSubdivisions)
            arguments
                nSubdivisions (1,1) uint32 = uint32(2)
            end

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

                for idFace = 1:nFaces
                    ui32FaceVerts = ui32Faces(idFace, :);
                    ui32MidVerts = zeros(1, 3, 'uint32');
                    ui32EdgePairs = [ui32FaceVerts(1) ui32FaceVerts(2); ...
                                     ui32FaceVerts(2) ui32FaceVerts(3); ...
                                     ui32FaceVerts(3) ui32FaceVerts(1)];

                    for idEdge = 1:3
                        charKey = sprintf('%u_%u', ...
                            min(ui32EdgePairs(idEdge, :)), max(ui32EdgePairs(idEdge, :)));
                        if isKey(edgeMap, charKey)
                            ui32MidVerts(idEdge) = edgeMap(charKey);
                        else
                            dNewVert = (dVerts(ui32EdgePairs(idEdge, 1), :) + ...
                                        dVerts(ui32EdgePairs(idEdge, 2), :)) / 2;
                            dNewVert = dNewVert / norm(dNewVert);
                            dVerts = [dVerts; dNewVert]; %#ok<AGROW>
                            ui32MidVerts(idEdge) = uint32(size(dVerts, 1));
                            edgeMap(charKey) = ui32MidVerts(idEdge);
                        end
                    end

                    a = ui32FaceVerts(1);
                    b = ui32FaceVerts(2);
                    c = ui32FaceVerts(3);
                    ab = ui32MidVerts(1);
                    bc = ui32MidVerts(2);
                    ca = ui32MidVerts(3);

                    ui32NewFaces((idFace-1)*4+1, :) = [a  ab ca];
                    ui32NewFaces((idFace-1)*4+2, :) = [b  bc ab];
                    ui32NewFaces((idFace-1)*4+3, :) = [c  ca bc];
                    ui32NewFaces((idFace-1)*4+4, :) = [ab bc ca];
                end

                ui32Faces = ui32NewFaces;
            end
        end

        function writeObjFile_(charObjPath, ui32Faces, dVerts)
            fid = fopen(char(charObjPath), "w");
            assert(fid ~= -1, "testCShapeModelSimplifyMesh:FileOpenFailed", ...
                "Failed to create temporary OBJ file: %s", char(charObjPath));
            cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

            for idVert = 1:size(dVerts, 1)
                fprintf(fid, "v %.16g %.16g %.16g\n", dVerts(idVert, 1), dVerts(idVert, 2), dVerts(idVert, 3));
            end

            for idFace = 1:size(ui32Faces, 1)
                fprintf(fid, "f %u %u %u\n", ui32Faces(idFace, 1), ui32Faces(idFace, 2), ui32Faces(idFace, 3));
            end
        end
    end
end
