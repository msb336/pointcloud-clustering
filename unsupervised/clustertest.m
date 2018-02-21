%%
clearvars;close all;clc
addpath functions data
%%
 j = 1;
 groups = [];
 cloud = [];
for i = 1:10
h1 = buildshape('l', 0.3, 0) + [(rand-0.5)*20 (rand-0.5)*20 (rand-0.5)*20];
h2 = buildshape('sphere', 0.5, 0)+[(rand-0.5)*20 (rand-0.5)*20 (rand-0.5)*20];
groups = [groups; j*ones(length(h1),1); (j+1)*ones(length(h2),1)];
j = j + 2;
cloud = [cloud; h1;h2];
end
%%
num = 20;
res = table;
%% Agglomerative Cluster
tic
Z = linkage(cloud);
c = cluster(Z, 'maxclust', num);
t = toc;
[agga, aggp, aggr, aggf1] = getstats(c, groups);
res.agglomerative = [agga, aggp, aggr, aggf1 t]';
tit = sprintf('Agglomerative Cluster. Time: %d', t);
figure;
scatter3(cloud(:,1),cloud(:,2),cloud(:,3),10,c); title(tit);

%% Fuzzy Clustering
tic
[~, U] = fcm(cloud, num);
t = toc;

tit = sprintf('Fuzzy C-means. Time: %d', t);
maxU = max(U);
figure;
p = [];
for i = 1:num
    index = U(i,:) == maxU;
    p = [p; i*ones(length(index(index)),1)];
    plot3dvectors(cloud(index,:), 'o');
end
title(tit);
[fcma, fcmp, fcmr, fcmf1] = getstats(p, groups);
res.fcm = [fcma, fcmp, fcmr, fcmf1 t]';

%% K-means Clustering
tic
[centers, U] = kmeans(cloud, num);

t = toc;
tit = sprintf('K-means. Time: %d', t);

maxU = max(U);
figure;
p = [];
for i = 1:num
    index = centers == i;
    p = [p; i*ones(length(index(index)),1)];
    plot3dvectors(cloud(index,:), 'o');
end
title(tit);
[kma, kmp, kmr, kmf1] = getstats(p, groups);
res.kmeans = [kma, kmp, kmr, kmf1 t]';

%%
res


