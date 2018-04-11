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
    dimension = str2num(header);
    numpoints = fscanf(fid, '%d', 1);
end

vertex = dlmread(filename, ' ', [2 0 1+numpoints 2]);
face = dlmread(filename, ' ', 2+numpoints, 0);
s = size(face,2);
face = face(:, (s - face(1,1)+1):s) + 1;


fclose(fid);
