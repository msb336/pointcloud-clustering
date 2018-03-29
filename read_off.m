function [vertex,face] = read_off(filename)

% read_off - read data from OFF file.
%
%   [vertex,face] = read_off(filename);
%
%   'vertex' is a 'nb.vert x 3' array specifying the position of the vertices.
%   'face' is a 'nb.face x 3' array specifying the connectivity of the mesh.
%
%   Copyright (c) 2003 Gabriel Peyr


fid = fopen(filename,'r');
if( fid==-1 )
    error('Can''t open the file.');
end

dimension = fscanf(fid, '%d', 1);
numpoints = fscanf(fid, '%d', 1);


[A,cnt] = fscanf(fid,'%f %f %f', 3*numpoints);

if cnt~=3*numpoints
    warning('Problem in reading vertices.');
end

A = reshape(A, 3, cnt/3);
vertex = A';



nface = fscanf(fid, '%d', 1);

[A,cnt] = fscanf(fid,'%d %d %d %d\n', 4*nface);


if cnt~=4*nface
    warning('Problem in reading faces.');
end
A = reshape(A, 4, cnt/4);
face = A'+1;


fclose(fid);
