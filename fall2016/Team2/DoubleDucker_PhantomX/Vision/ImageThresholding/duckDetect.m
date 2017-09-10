function [circle_cent] = duckDetect(centroids, BoundingBox)
c = centroids;
bb = BoundingBox;

% compute area to detectect largest object
area = 0;
area_i=0;
circle_cent = centroids(1,:);
for j =1:length(c(:,1))
    bb_i=bb(j,:);
    area_i = abs((bb_i(1)-bb_i(2))) * abs((bb_i(3)-bb_i(4)));
    if area_i > area;
        area = area_i;
        area_i=0;
        circle_cent = c(j,:);
    end
end