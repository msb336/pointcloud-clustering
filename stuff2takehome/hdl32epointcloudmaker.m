clearvars;clc
laserpos = linspace(-10.67*pi/180,30.67*pi/180,32)';
location = @(x)[4*x-15 2*sin(3*x)+5 5];

objects.boundaries = unique(dlmread('simulated-beam-0noise (1).xyz'), 'rows');
rotation_rate = 20;
sample_rate=700000;

scan_per_second = sample_rate/32;
scan_per_rotation = scan_per_second/rotation_rate;

time = 0;
r = 5;
p = [];
cleanpts = [];
nextboy = 0.25;

subplot(1,2,1);
pc = pointCloud(objects.boundaries);
pcshow(objects.boundaries);
xlabel('x');ylabel('y');zlabel('z');view([125 25]);axis([-15 15 -15 15 -5 45]);
subplot(1,2,2);
im = [];
path = [];
k = 1;
gif_file='test.gif';

duration = 7.5;

for time = 0:(1/scan_per_rotation):duration
    
    %     hold off;
    plot3(0,0,0);
    loc = location(time);
    path = [path; loc];
    rot_angle = time*rotation_rate*2*pi;
    vectors = [ cos(laserpos)*cos(rot_angle) ...
        cos(laserpos)*sin(rot_angle) ...
        sin(laserpos) ];
    %     hold on;
    
    %     plot3(path(:,1), path(:,2), path(:,3), 'r-');
    %      for i = 1:32
    %          point = vectors(i,:);
    %           plot3(loc(1)+[0 point(1)], loc(2)+[0 point(2)], loc(3)+[0 point(3)], '-');
    %      end
    %     view([125 25])
    %     xlabel('x');ylabel('y');zlabel('z');
    %     pause(0.000001)
    
    [r, xyzclean] = collisiondetect(objects, vectors, loc);
    
    xyz = [ r.*cos(laserpos).*cos(rot_angle) ...
        r.*cos(laserpos).*sin(rot_angle) ...
        r.*sin(laserpos) ];
    
    p = [p; xyz(any(xyz~=0, 2),:)+loc];
    cleanpts = [cleanpts; xyzclean];
    %     plot3(cleanpts(:,1), cleanpts(:,2), cleanpts(:,3), 'b.');
    
    %     axis([-15 15 -15 15 -1 12]);
    
    
    
    %     f = getframe(gcf());
    %     im = frame2im(f);
    %
    %     [A,map] = rgb2ind(im, 256);
    %     if k == 1
    %         imwrite(A,map,gif_file,'gif','LoopCount',Inf,'DelayTime',0);
    %     else
    %         imwrite(A,map,gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 0);
    %     end
    %     k=k+1;
    
    if (time/duration) >= nextboy
        nextboy
        nextboy = nextboy+0.25;
    end
    
end


%%

lidarcloud = pointCloud(p);
close;pcshow(lidarcloud);hold on; plot3(path(:,1), path(:,2), path(:,3), 'r-');axis([-15 15 -15 15 -1 12]);pbaspect([1 1 1])
% plot3(cleanpts(:,1), cleanpts(:,2), cleanpts(:,3), 'b.')