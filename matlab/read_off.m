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
        adder = 1;

elseif contains(header, 'OFF')
        [header_data] = sscanf(header, 'OFF %d %d %d');
        numpoints = header_data(1); nface = header_data(2);
        adder=0;
else
    error('incorrect file format');
end

vertex = dlmread(filename, ' ', [1+adder 0 adder+numpoints 2]);
face = dlmread(filename, ' ', 1+adder+numpoints, 0);
face = face(all(floor(face)==face, 2), :);
s = face(1);
face = face(:, end-s+1:end) + 1;

fclose(fid);
