function [acc, prec, rec, f1] = getstats(cluster,ground)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
tp =0;fp=0;tn=0;fn=0;

for i = 1:max(ground)
    points = cluster == i;
    ground(points);
    tp = tp + numpos;
end
acc = tp/length(cluster);
prec = 1;
rec = 1;
f1 = 1;