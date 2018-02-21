function [hol] = buildshape(shape,definition, noise_level)
%BUILDSHAPE build a pointcloud of shape SHAPE, definition DEFINITION, and
%noise level NOISE_LEVEL
shape = lower(shape);
if nargin > 1
    d = -1:definition:1;
end

switch shape
    case 'l'
        %% L
        [x1,y1,z1] = meshgrid(d,d,d);
        [x2,y2,z2] = meshgrid(d,d,d+2);
        [x3,y3,z3] = meshgrid(d,d+2,d);
        
        x = [x1(:); x2(:); x3(:)];
        y = [y1(:); y2(:); y3(:)];
        z = [z1(:); z2(:); z3(:)];
        p = unique([x,y,z], 'rows');
        hol = makeHollow(p, definition*1.1);
    case 'sphere'
        %% Sphere
        [x,y,z] = sphere(ceil(10/definition));
        p = unique([x(:), y(:), z(:)],'rows');
        hol = p;
    case 'plane'
        %% oscillating Plane
        [x,y] = meshgrid(-10:definition:10, -10:definition:10);
        z = sin(x(:)) + cos(y(:));
        hol = [x(:) y(:) z(:)];
    case 'read'
        hol = csvread('bridge.csv');
        noise_level = 0;
end

%%
hol = hol + noise_level*randn(size(hol));
end

