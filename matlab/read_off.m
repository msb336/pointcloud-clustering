function [face,vertex] = read_off(filename)

fid = fopen(filename,'r');

if( fid==-1 )
    error('Can''t open the file.');
end

header = fgetl(fid);
if strcmp(header, 'OFF')
        numpoints = fscanf(fid, '%d', 1);
        nface = fscanf(fid, '%d', 1);
        fscanf(fid, '%d', 1);
else
    error('incorrect file format');
end

vertex = dlmread(filename, ' ', [2 0 1+numpoints 2]);
face = dlmread(filename, ' ', 2+numpoints, 0);
face = face(all(floor(face)==face, 2), :);
s = face(1);
face = face(:, end-s+1:end) + 1;

fclose(fid);
