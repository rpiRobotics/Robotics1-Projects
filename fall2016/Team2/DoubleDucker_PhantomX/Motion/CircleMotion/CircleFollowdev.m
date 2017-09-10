function [  ] = CircleFollowdev( phantomX,radius,normvec,center )
%Command phantomX unit to follow a circle in 3-space defined by a radius, a
%vector normal to the circle plane, and the circle center. 
path=Circle3(radius,normvec,center(1),center(2),center(3));
smoothMotion(phantomX,path(:,1));
for i=1:length(path)
    i
    if i ~=1
        smoothMotion1(phantomX,path(:,i),path(:,i-1));
    else
        smoothMotion1(phantomX,path(:,i),[.1;0;.1]);
    end
end

end
