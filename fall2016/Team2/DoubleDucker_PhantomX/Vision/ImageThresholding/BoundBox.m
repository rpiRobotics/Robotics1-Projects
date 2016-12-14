function [ centroids, boundingboxes ] = BoundBox( BW )
%BOUNDBOX 

test = find(BW==1, 1);

if isempty(test) == 1
    centroids = 0;
    boundingboxes = 0;
    return

else
    bb  = regionprops(BW, {'BoundingBox','Centroid'});
    centroids = cat(1, bb.Centroid);
    boundingboxes = cat(1, bb.BoundingBox);
    hold on
    plot(centroids(:,1), centroids(:,2), 'r+')
    for k = 1:size(boundingboxes,1)
        rectangle('position',boundingboxes(k,:),'Edgecolor','g')
    end
hold off

end

end

