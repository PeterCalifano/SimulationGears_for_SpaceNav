close all
clear
clc


%% Landmark map table generation script
asteroidID = 1;

% ID list:
% 0) Didymos (GMV Hera model, as of 2023 baseline)
% 1) Didymos (new model, 2023)
% 2) Bennu 

% OPTIONS
savepath = "mappedLMtables";
savetime = fullfile(string(datetime("now", "Format", "dd_MMM_uuuu_HH-mm")));

seedNum = 0;
rng(seedNum);
Nlandmarks = 1000;

%% Sampling grid definition
% Construct uniform spherical sampling grid using icosahedron discretization
% [shellList, nodesArrayCell, cornersIDs] = GenIcosahedronDiscrSphere(10, 1);
% samplingGrid = nodesArrayCell{1};



%% Asteroid selection
switch asteroidID

    case 0

        %% Didymos
        asteroidName = "Didymos";
        strShapeModel = load('/home/peterc/devDir/nav-backend/customExamples/matlab/data/didymain_gmv_v1.mat').shape_data;

        strShapeModel.facetsVertexArray = [strShapeModel.pointersA; strShapeModel.pointersB; strShapeModel.pointersC];
        strShapeModel.radiusMax = 1.00000008214333;
        strShapeModel.verticesDim = 1000*strShapeModel.vertices;
        
        shapeModelPointCloud = strShapeModel.verticesDim;

    case 1
        %% Dydimos new
        asteroidName = "Didymos2023";
        % Load Didydmos model from SPICE kernels
        kernelname = char(fullfile("data", "didymos_g_09309mm_spc_0000n00000_v003.bds"));
        cspice_furnsh( kernelname );
        [filetype, sourcefile, kernelhandle] = cspice_kinfo(kernelname);
        dladsc = cspice_dlabfs(kernelhandle); % What does this function do?

        % Get number of vertices and triangles
        [nVertices, nTriangles] = cspice_dskz02(kernelhandle, dladsc);
        % Get triangles from SPICE (p: plates)
        trianglesVertices = cspice_dskp02(kernelhandle, dladsc, 1, nTriangles);
        % trianglesVertices = double(trianglesVertices);

        % Get vertices from SPICE (v: vertices)
        modelVertices = 1000*cspice_dskv02(kernelhandle, dladsc, 1, nVertices);

        strShapeModel.ui32triangVertexPtr = trianglesVertices;
        strShapeModel.dVerticesPos = modelVertices;
        
        

    case 2
        %% Bennu
        %load('data/Bennu_v20_200k.obj');


end

%% Generate and save landmarks map table

% TO REWORK: facets must be extracted from the model

[o_dLMposMap] = generateLandmarksMap(strShapeModel, Nlandmarks);

% Plot shape model and extracted LM
figure;
scatter3(strShapeModel.dVerticesPos(1,:), strShapeModel.dVerticesPos(2,:), ...
    strShapeModel.dVerticesPos(3,:), 'k', '.');
hold on;
scatter3(o_dLMposMap(2,:), o_dLMposMap(3,:), o_dLMposMap(4,:), 'r', '*');
DefaultPlotOpts;
xlabel('X [LU]');
ylabel('Y [LU]');
zlabel('Z [LU]');
axis equal
legend("Shape mesh vertices", "Sparse LMs")
% shapePatch = patch('Vertices', D1_shapeModel.verticesDim', 'Faces', D1_shapeModel.facetsVertexArray');
% set(shapePatch, 'Edgealpha', 0.11);
% set(shapePatch, 'EdgeColor', '#262525');
% set(shapePatch, 'FaceAlpha', 0.13);
% set(shapePatch, 'DisplayName', 'D1 shape');
% lighting gouraud;

filename = fullfile(savepath, strcat("LandmarksMap", asteroidName, "_Nlm", num2str(Nlandmarks), "_", savetime)); 
save(filename, 'o_dLMposMap')

