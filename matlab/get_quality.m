function [ quality_matrix ] = get_quality( tri_obj )
%GET_QUALITY retrieve quality metrics from triangulation object
%   returns QUALITY_MATRIX with output columns:
%   1 -> l_min/R
%   2 -> r/l_min
%   3 -> r/R
%   4 -> l_min/l_max
%   5 -> V/l_max^3


[l_tri, con_type] = size(tri_obj.ConnectivityList);

if con_type == 3
    edges = [ 2 1; 3 1; 3 2];
elseif con_type == 4
    edges = [ 2 1; 3 2; 4 3; 1 4];
end

[~,R] = circumcenter(tri_obj);
[~,r] = incenter(tri_obj);
quality_matrix = zeros(l_tri, 5);


for i = 1:l_tri
    con = tri_obj.ConnectivityList(i,:);
    vertices = tri_obj.Points(con(:), :);
    edge_vectors = vertices(edges(:,1),:) - vertices(edges(:,2),:);
    edge_norms = (edge_vectors(:,1).^2+edge_vectors(:,2).^2+edge_vectors(:,3).^2).^0.5;
    l_min = min(edge_norms);
    l_max = max(edge_norms);
    quality_matrix(i,:) = [ l_min/R(i), r(i)/l_min, r(i)/R(i), ...
                            l_min/l_max, 0];
end
    




end

