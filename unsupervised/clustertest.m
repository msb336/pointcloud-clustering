%%
clearvars;close all;clc
addpath functions
%%
 j = 1;
 groups = [];
 cloud = [];
 noise = 0.1;
 figure;
 title('Ground Truth')
for i = 1:10
h1 = buildshape('randomizedblock', 0.3, noise) + [(rand-0.5)*20 (rand-0.5)*20 (rand)*5];
plot3dvectors(h1, '.');
h2 = buildshape('sphere', 0.5, noise)+[(rand-0.5)*20 (rand-0.5)*20 (rand)*5];
plot3dvectors(h2, '.');
groups = [groups; j*ones(length(h1),1); (j+1)*ones(length(h2),1)];
j = j + 2;
cloud = [cloud; h1;h2];
end
plane = buildshape('plane', 0.3, noise);
plot3dvectors(plane);
groups = [groups; j*ones(length(plane),1)];
cloud = [cloud;plane];
%%
num = j;
res = table;
%% Agglomerative Cluster
tic
Z = linkage(cloud);
c = cluster(Z, 'maxclust', num);
t = toc;
tit = sprintf('Agglomerative Cluster. Time: %d', t);
figure;
subplot(2,2,1);
scatter3(cloud(:,1),cloud(:,2),cloud(:,3),10,c, '.'); title(tit);
% [agga, aggp, aggr, aggf1] = getstats(c, groups);
% res.agglomerative = [agga, aggp, aggr, aggf1 t]';


%% Fuzzy Clustering
tic
[~, U] = fcm(cloud, num);
t = toc;

tit = sprintf('Fuzzy C-means. Time: %d', t);
maxU = max(U);
subplot(2,2,2);
p = [];
for i = 1:num
    index = U(i,:) == maxU;
    p = [p; i*ones(length(index(index)),1)];
    plot3dvectors(cloud(index,:), '.');
end
title(tit);
% [fcma, fcmp, fcmr, fcmf1] = getstats(p, groups);
% res.fcm = [fcma, fcmp, fcmr, fcmf1 t]';

%% K-means Clustering
tic
[centers, U] = kmeans(cloud, num);

t = toc;
tit = sprintf('K-means. Time: %d', t);

maxU = max(U);
subplot(2,2,3);
p = [];
for i = 1:num
    index = centers == i;
    p = [p; i*ones(length(index(index)),1)];
    plot3dvectors(cloud(index,:), '.');
end
title(tit);
% [kma, kmp, kmr, kmf1] = getstats(p, groups);
% res.kmeans = [kma, kmp, kmr, kmf1 t]';

%% DBScan
epsilon=0.5;
MinPts=10;
tic
[IDX, isnoise]=DBSCAN(cloud, epsilon, MinPts);
t = toc;
tit = sprintf('DBScan. Time: %d', t);
subplot(2,2,4);
for i = 1:max(IDX)
    plot3dvectors(cloud(IDX == i,:), '.');
end
% plot3dvectors(cloud(isnoise,:), '.');
title(tit);
%%
% res


