function [ShapeModel] = loadShapeModel(filePath)

    % [ShapeModel] = loadShapeModel(filePath)
    % A function computing the shape model of a polyhedral shape.
    % Similar functions are read_obj and read_wobj according to the user
    % needs.
    %
    % filePath: the path of the txt file where the polyhedron is. The file
    % must have the following form: 
    %               v 0 0 0
    %               v 0 0 1
    %               .
    %               .
    %               .
    %               v 0 1 0
    %               f 1 2 5
    %               .
    %               .
    %               .
    %               f 4 8 6
    %
    % where v labels the vertex and its cordinates and f the facets and the
    % associates vertexes number (f is the connectivity table of v).
    % If lines are empty, the fucntion cannot work. Only facets and
    % vertices are accepted for the obj file.
    %
    % ShapeModel: The shape model of a polyhedral shape  
    %
    % Author: Paolo Panicucci
    
    fixOBJ(filePath, filePath)
    [ ShapeModel ] = ReadVertexShapeModel(filePath);

end
