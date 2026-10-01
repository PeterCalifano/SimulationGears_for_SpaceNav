classdef testLoadShapeMesh < matlab.unittest.TestCase
    % testLoadShapeMesh verifies validated host-side OBJ and STL geometry loading.

    methods (Test)
        function TestTriangleObjPreservesUnrepairedGeometry(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charObjPath = fullfile(charFixtureRoot, 'triangle.obj');
            testLoadShapeMesh.WriteTextFile_(charObjPath, [ ...
                "v 0 0 0"; ...
                "v 1 0 0"; ...
                "v 0 1 0"; ...
                "f 1 2 3"]);

            strMesh = LoadShapeMesh(charObjPath, bRepairMesh=false);

            self.verifyEqual(strMesh.ui32FaceVertexIds, uint32([1, 2, 3]));
            self.verifyEqual(strMesh.dVerticesPos, [0, 0, 0; 1, 0, 0; 0, 1, 0]);
            self.verifyEqual(strMesh.ui32NumFaces, uint32(1));
            self.verifyEqual(strMesh.ui32NumVertices, uint32(3));
            self.verifyEqual(strMesh.charSourcePath, charObjPath);
        end

        function TestConcaveObjPolygonPreservesArea(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charObjPath = fullfile(charFixtureRoot, 'concave.obj');
            testLoadShapeMesh.WriteTextFile_(charObjPath, [ ...
                "v 0 0 0"; ...
                "v 2 0 0"; ...
                "v 2 2 0"; ...
                "v 1 1 0"; ...
                "v 0 2 0"; ...
                "f 1 2 3 4 5"]);

            strMesh = LoadShapeMesh(charObjPath, bRepairMesh=false);
            dTriangleArea = testLoadShapeMesh.ComputeTriangleAreas_(strMesh);

            self.verifyEqual(strMesh.ui32NumFaces, uint32(3));
            self.verifyEqual(sum(dTriangleArea), 3.0, 'AbsTol', 1.0e-14);
            self.verifyGreaterThan(min(dTriangleArea), 0.0);
        end

        function TestMixedSlashAndNegativeObjIndices(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charObjPath = fullfile(charFixtureRoot, 'slash_negative.obj');
            testLoadShapeMesh.WriteTextFile_(charObjPath, [ ...
                "v 0 0 0"; ...
                "v 1 0 0"; ...
                "v 1 1 0"; ...
                "v 0 1 0"; ...
                "f -4/1/1 -3//1 -2/3 -1"]);

            strMesh = LoadShapeMesh(charObjPath, bRepairMesh=false);

            self.verifyEqual(strMesh.ui32NumFaces, uint32(2));
            self.verifyEqual(sort(unique(strMesh.ui32FaceVertexIds)), uint32((1:4).'));
        end

        function TestRepairWeldsVerticesAndRemovesDegenerateFaces(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charObjPath = fullfile(charFixtureRoot, 'repair.obj');
            testLoadShapeMesh.WriteTextFile_(charObjPath, [ ...
                "v 0 0 0"; ...
                "v 1 0 0"; ...
                "v 0 1 0"; ...
                "v 0 0 0"; ...
                "v 2 0 0"; ...
                "f 1 2 3"; ...
                "f 4 2 5"]);

            strRawMesh = LoadShapeMesh(charObjPath, bRepairMesh=false);
            strRepairedMesh = LoadShapeMesh(charObjPath, bRepairMesh=true);

            self.verifyEqual(strRawMesh.ui32NumVertices, uint32(5));
            self.verifyEqual(strRawMesh.ui32NumFaces, uint32(2));
            self.verifyEqual(strRepairedMesh.ui32NumVertices, uint32(3));
            self.verifyEqual(strRepairedMesh.ui32NumFaces, uint32(1));
        end

        function TestInvalidObjRecordsFailClearly(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charBadIndexPath = fullfile(charFixtureRoot, 'bad_index.obj');
            testLoadShapeMesh.WriteTextFile_(charBadIndexPath, [ ...
                "v 0 0 0"; "v 1 0 0"; "v 0 1 0"; "f 1 2 99"]);

            charNonPlanarPath = fullfile(charFixtureRoot, 'non_planar.obj');
            testLoadShapeMesh.WriteTextFile_(charNonPlanarPath, [ ...
                "v 0 0 0"; "v 1 0 0"; "v 1 1 0"; "v 0 1 0.1"; "f 1 2 3 4"]);

            charBadVertexPath = fullfile(charFixtureRoot, 'bad_vertex.obj');
            testLoadShapeMesh.WriteTextFile_(charBadVertexPath, [ ...
                "v 0 0 0"; "v not_a_number 0 0"; "v 0 1 0"; "f 1 2 3"]);

            self.verifyError(@() LoadShapeMesh(charBadIndexPath, bRepairMesh=false), ...
                'LoadShapeMesh:BadFaceIndex');
            self.verifyError(@() LoadShapeMesh(charNonPlanarPath, bRepairMesh=false), ...
                'LoadShapeMesh:BadObjFace');
            self.verifyError(@() LoadShapeMesh(charBadVertexPath, bRepairMesh=false), ...
                'LoadShapeMesh:BadObjVertex');
        end

        function TestNonSimpleAndEmptyObjFailClearly(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charSelfIntersectingPath = fullfile(charFixtureRoot, 'self_intersecting.obj');
            testLoadShapeMesh.WriteTextFile_(charSelfIntersectingPath, [ ...
                "v 0 0 0"; "v 1 1 0"; "v 0 1 0"; "v 1 0 0"; "f 1 2 3 4"]);
            charEmptyPath = fullfile(charFixtureRoot, 'empty.obj');
            testLoadShapeMesh.WriteTextFile_(charEmptyPath, "# no geometry");

            self.verifyError(@() LoadShapeMesh(charSelfIntersectingPath, bRepairMesh=false), ...
                'LoadShapeMesh:BadObjFace');
            self.verifyError(@() LoadShapeMesh(charEmptyPath, bRepairMesh=false), ...
                'LoadShapeMesh:EmptyMesh');
        end

        function TestAsciiStlLoadsShortValidFile(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charStlPath = fullfile(charFixtureRoot, 'short_ascii.stl');
            testLoadShapeMesh.WriteTextFile_(charStlPath, [ ...
                "solid"; ...
                "facet normal 0 0 1"; ...
                "outer loop"; ...
                "vertex 0 0 0"; ...
                "vertex 1 0 0"; ...
                "vertex 0 1 0"; ...
                "endloop"; ...
                "endfacet"; ...
                "endsolid"]);

            strMesh = LoadShapeMesh(charStlPath, bRepairMesh=false);

            self.verifyEqual(strMesh.ui32NumFaces, uint32(1));
            self.verifyEqual(strMesh.ui32NumVertices, uint32(3));
        end

        function TestBinaryStlWithSolidHeaderLoads(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charStlPath = fullfile(charFixtureRoot, 'solid_header_binary.stl');
            testLoadShapeMesh.WriteBinaryStl_(charStlPath, 'solid binary header');

            strMesh = LoadShapeMesh(charStlPath, bRepairMesh=false);

            self.verifyEqual(strMesh.ui32NumFaces, uint32(1));
            self.verifyEqual(strMesh.dVerticesPos, [0, 0, 0; 1, 0, 0; 0, 1, 0]);
        end

        function TestMalformedStlFailsClearly(self)
            [charFixtureRoot, objCleanup] = testLoadShapeMesh.MakeFixtureRoot_(); %#ok<ASGLU>
            charTruncatedPath = fullfile(charFixtureRoot, 'truncated.stl');
            i32FileId = fopen(charTruncatedPath, 'w', 'ieee-le');
            self.assertGreaterThan(i32FileId, 0);
            fwrite(i32FileId, zeros(80, 1, 'uint8'), 'uint8');
            fwrite(i32FileId, uint32(1), 'uint32');
            fwrite(i32FileId, zeros(10, 1, 'uint8'), 'uint8');
            fclose(i32FileId);

            charMalformedAsciiPath = fullfile(charFixtureRoot, 'malformed_ascii.stl');
            testLoadShapeMesh.WriteTextFile_(charMalformedAsciiPath, [ ...
                "solid malformed"; ...
                "facet normal 0 0 1"; ...
                "outer loop"; ...
                "vertex 0 0 0"; ...
                "vertex 1 0 0"; ...
                "endloop"; ...
                "endfacet"; ...
                "endsolid malformed"]);

            self.verifyError(@() LoadShapeMesh(charTruncatedPath, bRepairMesh=false), ...
                'LoadShapeMesh:TruncatedBinaryStl');
            self.verifyError(@() LoadShapeMesh(charMalformedAsciiPath, bRepairMesh=false), ...
                'LoadShapeMesh:BadAsciiStl');
        end
    end

    methods (Static, Access = private)
        function [charFixtureRoot, objCleanup] = MakeFixtureRoot_()
            charFixtureRoot = tempname;
            mkdir(charFixtureRoot);
            objCleanup = onCleanup(@() rmdir(charFixtureRoot, 's'));
        end

        function WriteTextFile_(charFilePath, strLines)
            i32FileId = fopen(charFilePath, 'w');
            assert(i32FileId > 0, 'testLoadShapeMesh:OpenFailed', ...
                'Could not create test fixture: %s', charFilePath);
            objCleanup = onCleanup(@() fclose(i32FileId));
            fprintf(i32FileId, '%s\n', strLines);
            clear objCleanup
        end

        function WriteBinaryStl_(charFilePath, charHeaderText)
            i32FileId = fopen(charFilePath, 'w', 'ieee-le');
            assert(i32FileId > 0, 'testLoadShapeMesh:OpenFailed', ...
                'Could not create test fixture: %s', charFilePath);
            objCleanup = onCleanup(@() fclose(i32FileId));

            ui8Header = zeros(80, 1, 'uint8');
            ui8HeaderText = uint8(charHeaderText);
            ui8Header(1:numel(ui8HeaderText)) = ui8HeaderText;
            fwrite(i32FileId, ui8Header, 'uint8');
            fwrite(i32FileId, uint32(1), 'uint32');
            fwrite(i32FileId, single([0; 0; 1]), 'float32');
            fwrite(i32FileId, single([0; 0; 0; 1; 0; 0; 0; 1; 0]), 'float32');
            fwrite(i32FileId, uint16(0), 'uint16');
            clear objCleanup
        end

        function dTriangleArea = ComputeTriangleAreas_(strMesh)
            ui32Faces = strMesh.ui32FaceVertexIds;
            dVertices = strMesh.dVerticesPos;
            dEdge1 = dVertices(double(ui32Faces(:, 2)), :) - ...
                dVertices(double(ui32Faces(:, 1)), :);
            dEdge2 = dVertices(double(ui32Faces(:, 3)), :) - ...
                dVertices(double(ui32Faces(:, 1)), :);
            dTriangleArea = 0.5 .* vecnorm(cross(dEdge1, dEdge2, 2), 2, 2);
        end
    end
end
