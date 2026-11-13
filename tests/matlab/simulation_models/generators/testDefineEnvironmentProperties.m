classdef testDefineEnvironmentProperties < matlab.unittest.TestCase
    methods (Test)
        function testSpiceRateSignAndOptionalAxisContract(testCase)
            % Explicit opt-in uses real NAIF kernels; unavailable installations report a filtered test.
            charKernelRoot = string(getenv('SIMGEARS_SPICE_KERNEL_ROOT'));
            testCase.assumeTrue(isfolder(charKernelRoot) && exist('cspice_xf2rav', 'file') == 2, ...
                'Set SIMGEARS_SPICE_KERNEL_ROOT and add the real MICE toolbox for this integration test.');

            cellKernels = {fullfile(charKernelRoot, 'naif0012.tls'), ...
                fullfile(charKernelRoot, 'pck00010.tpc'), fullfile(charKernelRoot, 'de440.bsp')};
            for ui32KernelIdx = uint32(1):uint32(numel(cellKernels))
                testCase.assertTrue(isfile(cellKernels{ui32KernelIdx}));
                cspice_furnsh(char(cellKernels{ui32KernelIdx}));
                testCase.addTeardown(@() cspice_unload(char(cellKernels{ui32KernelIdx})));
            end

            dTimestamps = 8e8 + [0, 20, 70];
            [~, strMainBodyRefData] = DefineEnvironmentProperties(dTimestamps, "Moon", "J2000");

            % A SPICE frame declares no spin axis, and its rate regime is always known.
            testCase.verifyFalse(isfield(strMainBodyRefData, 'dSpinAxis_TB'));
            testCase.verifyFalse(isfield(strMainBodyRefData, 'charSpinAxisSource'));
            testCase.verifyNotEqual(strMainBodyRefData.enumAngularVelocityRegime, ...
                EnumTargetAngularVelocityRegime.UNAVAILABLE);

            % Differentiate the SPICE attitude itself: skew(omega_IN) = dR_INfromTB/dt * R_INfromTB'.
            for ui32TimeIdx = uint32(1):uint32(numel(dTimestamps))
                dDCMBefore_INfromTB = cspice_pxform('IAU_MOON', 'J2000', dTimestamps(ui32TimeIdx) - 0.1);
                dDCMAfter_INfromTB = cspice_pxform('IAU_MOON', 'J2000', dTimestamps(ui32TimeIdx) + 0.1);
                dSkewAngVel_IN = ((dDCMAfter_INfromTB - dDCMBefore_INfromTB) / 0.2) ...
                    * strMainBodyRefData.dDCM_INfromTB(:, :, ui32TimeIdx)';
                dExpectedAngVel_IN = [dSkewAngVel_IN(3, 2); dSkewAngVel_IN(1, 3); dSkewAngVel_IN(2, 1)];
                testCase.verifyEqual(strMainBodyRefData.dAngVel_IN(:, ui32TimeIdx), dExpectedAngVel_IN, ...
                    'AbsTol', 1e-11);
            end
        end

        function testCarriesDatasetTargetAngularVelocity(testCase)
            dTimestamps = [0.0, 15.0, 40.0];
            dStateSC_IN = zeros(6, numel(dTimestamps));
            dDCM_TBfromIN = repmat(eye(3), 1, 1, numel(dTimestamps));
            dTargetPosition_IN = zeros(3, numel(dTimestamps));
            dSunPosition_IN = repmat([1.495978707e11; 0.0; 0.0], 1, numel(dTimestamps));
            dEarthPosition_IN = zeros(3, numel(dTimestamps));
            dTargetAngVel_IN = [0.8e-4, 0.9e-4, 1.1e-4; ...
                               -0.4e-4, -0.3e-4, -0.5e-4; ...
                                1.7e-4, 1.6e-4, 1.8e-4];
            objDataset = SReferenceMissionDesign(EnumFrameName.IN, dTimestamps, dStateSC_IN, ...
                dDCM_TBfromIN, dTargetPosition_IN, dSunPosition_IN, dEarthPosition_IN, ...
                'dTargetAngVel_IN', dTargetAngVel_IN);

            [~, strMainBodyRefData] = DefineEnvironmentProperties(dTimestamps, ...
                "Itokawa", "J2000", objDataset=objDataset);

            testCase.verifyEqual(strMainBodyRefData.dAngVel_IN, dTargetAngVel_IN, 'AbsTol', 0.0);
            testCase.verifyEqual( ...
                strMainBodyRefData.enumAngularVelocityRegime, ...
                EnumTargetAngularVelocityRegime.TIME_VARYING);
        end

        function testCanSkipDatasetTargetRotationalState(testCase)
            [dTimestamps, dStateSC_IN, dDCM_TBfromIN, dTargetPosition_IN, ...
                dSunPosition_IN, dEarthPosition_IN] = testCase.buildMinimalDatasetInputs();
            dTargetAngVel_IN = repmat([0.8e-4; -0.4e-4; 1.7e-4], 1, numel(dTimestamps));
            objDataset = SReferenceMissionDesign(EnumFrameName.IN, dTimestamps, dStateSC_IN, ...
                dDCM_TBfromIN, dTargetPosition_IN, dSunPosition_IN, dEarthPosition_IN, ...
                'dTargetAngVel_IN', dTargetAngVel_IN);

            [strSkippedDynParams, strMainBodyRefData, strSkipped3rdBodyRefData] = ...
                DefineEnvironmentProperties(dTimestamps, ...
                "Itokawa", "J2000", objDataset=objDataset, ...
                bLoadTargetRotationalState=false);
            [strLoadedDynParams, strLoadedMainBodyRefData, strLoaded3rdBodyRefData] = ...
                DefineEnvironmentProperties(dTimestamps, ...
                "Itokawa", "J2000", objDataset=objDataset, ...
                bLoadTargetRotationalState=true);

            testCase.verifyFalse(isfield(strMainBodyRefData, 'dDCM_INfromTB'));
            testCase.verifyFalse(isfield(strMainBodyRefData, 'dAngVel_IN'));
            testCase.verifyFalse(isfield(strMainBodyRefData, ...
                'enumAngularVelocityRegime'));
            testCase.verifyFalse(isfield(strMainBodyRefData, 'dSpinAxis_TB'));
            testCase.verifyFalse(isfield(strMainBodyRefData, 'charSpinAxisSource'));
            testCase.verifyEqual(strMainBodyRefData.dSunPosition_IN, dSunPosition_IN, 'AbsTol', 0.0);

            % The skip flag owns only target rotation; all other environment
            % and spacecraft outputs must match the normal source-loading path.
            cellRotationFields = {'dDCM_INfromTB', 'dAngVel_IN', ...
                                  'enumAngularVelocityRegime', ...
                                  'dSpinAxis_TB', 'charSpinAxisSource'};
            strExpectedMainBodyRefData = rmfield( ...
                strLoadedMainBodyRefData, cellRotationFields);
            testCase.verifyEqual(strMainBodyRefData, strExpectedMainBodyRefData);
            testCase.verifyEqual(strSkippedDynParams, strLoadedDynParams);
            testCase.verifyEqual(strSkipped3rdBodyRefData, strLoaded3rdBodyRefData);
        end

        function testDefaultsTargetSpinAxisToBodyPlusZ(testCase)
            [dTimestamps, dStateSC_IN, dDCM_TBfromIN, dTargetPosition_IN, ...
                dSunPosition_IN, dEarthPosition_IN] = testCase.buildMinimalDatasetInputs();
            objDataset = SReferenceMissionDesign(EnumFrameName.IN, dTimestamps, dStateSC_IN, ...
                dDCM_TBfromIN, dTargetPosition_IN, dSunPosition_IN, dEarthPosition_IN);

            [~, strMainBodyRefData] = DefineEnvironmentProperties(dTimestamps, ...
                "Itokawa", "J2000", objDataset=objDataset);

            testCase.verifyEqual(objDataset.dTargetSpinAxis_TB, [0.0; 0.0; 1.0], 'AbsTol', 0.0);
            testCase.verifyEqual(objDataset.charTargetSpinAxisSource, "DEFAULT_PLUS_Z");
            testCase.verifyEqual(strMainBodyRefData.dSpinAxis_TB, [0.0; 0.0; 1.0], 'AbsTol', 0.0);
            testCase.verifyEqual(strMainBodyRefData.charSpinAxisSource, "DEFAULT_PLUS_Z");
            testCase.verifyEqual( ...
                strMainBodyRefData.enumAngularVelocityRegime, ...
                EnumTargetAngularVelocityRegime.UNAVAILABLE);
        end

        function testNormalizesDeclaredTargetSpinAxis(testCase)
            [dTimestamps, dStateSC_IN, dDCM_TBfromIN, dTargetPosition_IN, ...
                dSunPosition_IN, dEarthPosition_IN] = testCase.buildMinimalDatasetInputs();
            objDataset = SReferenceMissionDesign(EnumFrameName.IN, dTimestamps, dStateSC_IN, ...
                dDCM_TBfromIN, dTargetPosition_IN, dSunPosition_IN, dEarthPosition_IN, ...
                'dTargetSpinAxis_TB', [0.0; -3.0; 0.0]);

            [~, strMainBodyRefData] = DefineEnvironmentProperties(dTimestamps, ...
                "Itokawa", "J2000", objDataset=objDataset);

            testCase.verifyEqual(strMainBodyRefData.dSpinAxis_TB, [0.0; -1.0; 0.0], 'AbsTol', 0.0);
            testCase.verifyEqual(strMainBodyRefData.charSpinAxisSource, "SCENARIO_DECLARED");
        end

        function testRejectsZeroDeclaredTargetSpinAxis(testCase)
            [dTimestamps, dStateSC_IN, dDCM_TBfromIN, dTargetPosition_IN, ...
                dSunPosition_IN, dEarthPosition_IN] = testCase.buildMinimalDatasetInputs();

            testCase.verifyError(@() SReferenceMissionDesign(EnumFrameName.IN, dTimestamps, dStateSC_IN, ...
                dDCM_TBfromIN, dTargetPosition_IN, dSunPosition_IN, dEarthPosition_IN, ...
                'dTargetSpinAxis_TB', zeros(3, 1)), ...
                'SReferenceMissionDesign:InvalidTargetSpinAxis');
        end

        function testBuildsSpacecraftPanelsAndPolyhedronGravityDataFromObj(testCase)
            objTempFolder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(objTempFolder.Folder), "cube_model.obj");
            [ui32Faces, dVerts] = testCase.buildCubeMesh();
            testCase.writeObjFile(charObjPath, ui32Faces, dVerts);

            dTimestamps = [0.0, 10.0];
            dStateSC_W = zeros(6, numel(dTimestamps));
            dDCM_TBfromW = repmat(eye(3), 1, 1, numel(dTimestamps));
            dTargetPosition_W = zeros(3, numel(dTimestamps));
            dSunPosition_W = repmat([1.495978707e11; 0; 0], 1, numel(dTimestamps));
            dEarthPosition_W = zeros(3, numel(dTimestamps));
            objDataset = SReferenceMissionDesign(EnumFrameName.IN, dTimestamps, dStateSC_W, ...
                dDCM_TBfromW, dTargetPosition_W, dSunPosition_W, dEarthPosition_W);

            strDynParamsSeed = struct();
            strDynParamsSeed.strSCdata = struct( ...
                'dReflCoeff', 1.2, ...
                'dSCmass', 10.0, ...
                'dA_SRP', 1.0);

            dDensity = 2000.0;
            dGravConst = 6.67430e-11;
            [strDynParams, ~, ~] = DefineEnvironmentProperties(dTimestamps, ...
                "Itokawa", ...
                "J2000", ...
                strDynParams=strDynParamsSeed, ...
                objDataset=objDataset, ...
                charSCpanelObjFilePath=charObjPath, ...
                dSCpanelDiffuseCoeff=0.25, ...
                dSCpanelSpecularCoeff=0.15, ...
                charPolyhedronGravityObjFilePath=charObjPath, ...
                dPolyhedronDensity=dDensity, ...
                dPolyhedronGravConst=dGravConst);

            strPanelData = strDynParams.strSCdata.strSRPpanelData;
            testCase.verifyEqual(strPanelData.ui32NumPanels, uint32(12));
            testCase.verifyEqual(sum(strPanelData.dSCquadsArea), 24.0, 'AbsTol', 1e-14);
            testCase.verifyEqual(strPanelData.dDiffSpecQuadsCoeffs, repmat([0.25, 0.15], 12, 1), 'AbsTol', 0.0);

            strPolyData = strDynParams.strMainData.strPolyhedronGravityData;
            testCase.verifyEqual(strPolyData.dVolume, 8.0, 'AbsTol', 1e-14);
            testCase.verifyEqual(strPolyData.dDensity, dDensity, 'AbsTol', 0.0);
            testCase.verifyEqual(strPolyData.dGravConst, dGravConst, 'AbsTol', 0.0);
            testCase.verifyEqual(strPolyData.dGravParam, dGravConst * dDensity * 8.0, 'RelTol', 1e-14);
            testCase.verifyEqual(size(strPolyData.ui32FaceVertexIds), [12, 3]);
            testCase.verifyGreaterThan(size(strPolyData.ui32EdgeVertexIds, 1), 0);
            testCase.verifyEqual([size(strPolyData.dEdgeDyadics, 1), size(strPolyData.dEdgeDyadics, 2)], [3, 3]);
            testCase.verifyEqual(size(strPolyData.dFaceDyadics), [3, 3, 12]);
        end
    end

    methods (Access = private)
        function [dTimestamps, dStateSC_IN, dDCM_TBfromIN, dTargetPosition_IN, ...
                dSunPosition_IN, dEarthPosition_IN] = buildMinimalDatasetInputs(~)
            dTimestamps = [0.0, 15.0, 40.0];
            dStateSC_IN = zeros(6, numel(dTimestamps));
            dDCM_TBfromIN = repmat(eye(3), 1, 1, numel(dTimestamps));
            dTargetPosition_IN = zeros(3, numel(dTimestamps));
            dSunPosition_IN = repmat([1.495978707e11; 0.0; 0.0], 1, numel(dTimestamps));
            dEarthPosition_IN = zeros(3, numel(dTimestamps));
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

        function writeObjFile(~, charObjPath, ui32Faces, dVerts)
            dFileId = fopen(char(charObjPath), "w");
            assert(dFileId ~= -1, "testDefineEnvironmentProperties:FileOpenFailed", ...
                "Failed to create temporary OBJ file: %s", char(charObjPath));
            objFileCleanup = onCleanup(@() fclose(dFileId));

            for ui32VertIdx = uint32(1):uint32(size(dVerts, 1))
                fprintf(dFileId, "v %.16g %.16g %.16g\n", dVerts(ui32VertIdx, 1), dVerts(ui32VertIdx, 2), dVerts(ui32VertIdx, 3));
            end

            for ui32FaceIdx = uint32(1):uint32(size(ui32Faces, 1))
                fprintf(dFileId, "f %u %u %u\n", ui32Faces(ui32FaceIdx, 1), ui32Faces(ui32FaceIdx, 2), ui32Faces(ui32FaceIdx, 3));
            end
        end
    end
end
