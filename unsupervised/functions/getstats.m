function [acc, prec, rec, f1] = getstats(cluster,ground)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
tp =0;fp=0;tn=0;fn=0;
for i = 1:max(ground)
    test = cluster(ground == i);
    l = length(test);
    [m,tp1] = mode(test);
    tp = tp+tp1;
    fp = fp+sum(cluster == m)-tp1;
    fn = fn+ l-tp1;
end
    tn = length(cluster) - (tp+fp+fn);
    acc = (tp+tn)/(tp+fp+fn+tn);
    rec = tp/(tp+fn);
    prec = tp/(tp+fp);
    f1 = 2*rec*prec/(rec+prec);
end
