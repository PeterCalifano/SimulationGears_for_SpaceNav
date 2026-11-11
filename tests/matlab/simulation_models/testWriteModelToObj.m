classdef testWriteModelToObj < matlab.unittest.TestCase
    %% DESCRIPTION
    % Check that WriteModelToObj writes an OBJ file that LoadShapeMesh reads back with the same vertices
    % and faces, in the folder of the given file name.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 11-11-2026  Pietro Califano, Claude Opus 5.5  Cover the round trip and the numbered name.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)
        function testRoundTripKeepsVerticesAndFaces(self)
            objTempFolder = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charRequestedPath = fullfile(objTempFolder.Folder, 'tetrahedron.obj');

            % Vertices exact in the 6 written decimals
            dVertices = [0, 0, 0; 1.25, 0, 0; 0, 2.5, 0; 0, 0, 3.125];
            ui32Faces = uint32([1, 3, 2; 1, 2, 4; 2, 3, 4; 1, 4, 3]);
            charWrittenPath = WriteModelToObj(struct('v', dVertices, 'f', struct('v', ui32Faces)), charRequestedPath);
            strMesh = LoadShapeMesh(charWrittenPath, bRepairMesh=false);

            self.verifyEqual(charWrittenPath, charRequestedPath);
            self.verifyEqual(strMesh.ui32FaceVertexIds, ui32Faces);
            self.verifyEqual(strMesh.dVerticesPos, dVertices);
        end

        function testExistingFileGetsNumberedName(self)
            objTempFolder = self.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charRequestedPath = fullfile(objTempFolder.Folder, 'triangle.obj');
            strFirstModel = struct('v', [0, 0, 0; 1, 0, 0; 0, 1, 0], 'f', struct('v', uint32([1, 2, 3])));
            strSecondModel = struct('v', [0, 0, 0; 2, 0, 0; 0, 2, 0], 'f', struct('v', uint32([1, 2, 3])));

            % Write twice without Overwrite
            charFirstPath = WriteModelToObj(strFirstModel, charRequestedPath);
            charSecondPath = WriteModelToObj(strSecondModel, charRequestedPath);

            self.verifyEqual(charFirstPath, charRequestedPath);
            self.verifyEqual(charSecondPath, fullfile(objTempFolder.Folder, 'triangle_1.obj'));
            strFirstMesh = LoadShapeMesh(charFirstPath, bRepairMesh=false);
            self.verifyEqual(strFirstMesh.dVerticesPos, strFirstModel.v);
        end
    end
end
