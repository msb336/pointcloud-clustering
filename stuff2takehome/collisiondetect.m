function [r, xyzclean] = collisiondetect(objects,vectors, location)
r = zeros(32,1);
xyzclean = [];
for i = 1:length(objects)
    testpts = objects(i).boundaries-location;
    unit_test = testpts ./ (testpts(:,1).^2 + testpts(:,2).^2 + testpts(:,3).^2).^0.5;
    
    
    for j = 1:length(vectors)
        subtr = unit_test - vectors(j,:);
        nn = (subtr(:,1).^2 + subtr(:,2).^2 + subtr(:,3).^2).^0.5;
        if min(nn) < 0.05
            pt = testpts(nn==min(nn), :);
            normpt = pt(:,1).^2 + pt(:,2).^2+pt(:,3).^2;
            pt = pt(normpt==min(normpt),:);
            plot3(location(1)+[0 pt(1)], location(2)+[0 pt(2)], location(3)+[0 pt(3)], 'r-');
            r(j) = norm(pt);
            xyzclean = [xyzclean; pt+location];
        end
    end
end
    
r = r(:,1);
end


