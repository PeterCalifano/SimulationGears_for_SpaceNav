function [faces,verts] = read_obj(obj_file)
% read_obj 
%
% Usages:   read_obj(obj_file)
%
% Description:
%   This function reads an obj file and outputs the faces and vertex
%   information. It is assumed that the vertices are listed first, and
%   start on line 1, followed directly by the facets with no blank lines
%   between.
%
% Inputs:
%   obj_file - file name of obj file
%
% Outputs:
%   faces - [n x 3] matrix of vertices that form each face
%   verts - [m x 3] matrix of vertex locations in implied body frame
%
% Modification history:
%   9/19/2012 - Jay - created
%

% Read file
fid = fopen(obj_file,'r');

commentCount = 0;
x = fgetl(fid);
while strcmp(x(1),'#')
    commentCount = commentCount + 1;
    x = fgetl(fid);
end

frewind(fid);
for ii = 1:commentCount
    x = fgetl(fid);
end

A = fscanf(fid,'%s %f %f %f',[4 inf]);

fclose(fid);

face_ind = find(A(1,:)==double('f'),1);

verts = A(2:4,1:face_ind-1)';

faces = A(2:4,face_ind:end)';

end