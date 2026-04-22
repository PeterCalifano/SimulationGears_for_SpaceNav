function dLMposMap = generateLandmarksMap(strShapeModel, dSamplingGrid)%#codegen
%% PROTOTYPE
% o_dLMposMap = generateLandmarksMap(strShapeModel.dVerticesPos, i_dSamplingGrid)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-04-2024        Pietro Califano         First version coded.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code
% ACHTUNG: the map must retain the same indices as of the input moddel 
% (i.e., the connectivity map to triangles)!

% i_strShapeModel.ui32triangVertexPtr = trianglesVertices;
% i_strShapeModel.dVerticesPos =  modelVertices;

assert(size(strShapeModel.dVerticesPos, 1) == 3, 'Input shape model point cloud must be [3xN].');

if not(isscalar(dSamplingGrid))
    
    % DEVNOTE: ACHTUNG, this branch never tested sufficiently. Legacy function to avoid.

    assert(size(dSamplingGrid, 2) == 4, 'Input sampling grid must be [Mx4].');

    % Get size of sampling grid
    NPoints = size(dSamplingGrid, 1);

    % Normalize shape model 3D cloud
    unitDirsTo3DPoints = strShapeModel.dVerticesPos./vernorm(strShapeModel.dVerticesPos, 2, 2);
    
    dLMposMap = coder.nullcopy(zeros(NPoints, 4));

    for idLM = 1:NPoints

        % Compute unit vector to ith sampling point
        unitDirToSamplPoint = transpose(dSamplingGrid(idLM, 2:4));
        
        % Compute dot product = cosine of "in-between" angle
        cosAngleDistMatrix = zeros(size(strShapeModel.dVerticesPos, 1), 2);

        cosAngleDistMatrix(:, 1) = dot( repmat(unitDirsTo3DPoints, 1, size(strShapeModel.dVerticesPos, 1) ), unitDirToSamplPoint);
        cosAngleDistMatrix(:, 2) = 1:size(strShapeModel.dVerticesPos, 1);

        % Find first two closest unit vectors in reduced 3D points cloud 
        sortedDistMatrix = sort( cosAngleDistMatrix( cosAngleDistMatrix(:, 1) >= 0, :), 1, "ascend");
            
        % Check range to first three nearest points. Get outermost if the
        % first the cosDist between the 1st and the 2nd is below median

        % angleError = acos(sortedDistMatrix(1:3, 1));
        medianCosDIst = quantile(sortedDistMatrix(:, 1), 0.05);
        
        if abs(sortedDistMatrix(1, 1) - sortedDistMatrix(3, 1)) <= medianCosDIst

            % Check outermost point
            idP1 = sortedDistMatrix(1, 2);
            idP3 = sortedDistMatrix(3, 2);

            dist1 = norm(strShapeModel.dVerticesPos(idP1, :));
            dist3 = norm(strShapeModel.dVerticesPos(idP3, :));

            if dist1 >= dist3
                idP = idP1;
            else
                idP = idP3;
            end

        else
            idP = 1;
        end

        % Store in LM map
        dLMposMap(idLM, 1) = idLM;
        dLMposMap(idLM, 2:4) = strShapeModel.dVerticesPos(:, sortedDistMatrix(idP, 2) );

    end

elseif isscalar(dSamplingGrid)
    % RANDOM SAMPLING

    % Generate random ID for extraction
    
%     samplingIDs = randi(size(i_strShapeModel.dVerticesPos, 2), i_dSamplingGrid, 1);
    samplingIDs = datasample(1:size(strShapeModel.dVerticesPos, 2), ...
        dSamplingGrid, 'Replace', false); % Required to avoid repetition

    % Extract points from input point cloud
    dLMposMap = [ samplingIDs; strShapeModel.dVerticesPos(:, samplingIDs)];

end

end

