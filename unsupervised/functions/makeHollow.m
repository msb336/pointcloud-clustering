function [ newpoints ] = makeHollow( pts, thresh )
%MAKEHOLLOW Summary of this function goes here
%  Pseudocode:
%  for point in points
%       check if point is local max x,y, or z. if it is, keep it.


bool = arrayfun(@(x)do(pts, x, thresh), 1:length(pts));
newpoints = pts(bool,:);

% newpoints = [];
% for i = 1:length(pts)
%     center = pts(i,:);
%     temp = pts - center;
%     inside = pts(sqrt(sum(temp.^2, 2)) <= thresh, :);
%     
%     posedges = [max(inside(:,1)) max(inside(:,2)) max(inside(:,3))];
%     negedges = [min(inside(:,1)) min(inside(:,2)) min(inside(:,3))];
%     
%     if any(center == posedges) || any(center == negedges)
%         newpoints(end+1,:) = center;
%     end
%     
% end

end
function [bool] = do(pts, index, thresh)

center = pts(index,:);
temp = pts - center;
inside = pts(sqrt(sum(temp.^2, 2)) <= thresh, :);
posedges = [max(inside(:,1)) max(inside(:,2)) max(inside(:,3))];
negedges = [min(inside(:,1)) min(inside(:,2)) min(inside(:,3))];

if any(center == posedges) || any(center == negedges)
    bool = true;
else
    bool = false;
end

end

