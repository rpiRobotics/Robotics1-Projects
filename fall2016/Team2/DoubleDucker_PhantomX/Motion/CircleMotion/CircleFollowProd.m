function [  ] = CircleFollowProd( phantomX,radius,normvec,center )
%Command phantomX unit to follow a circle in 3-space defined by a radius, a
%vector normal to the circle plane, and the circle center. 
path=Circle3(radius,normvec,center(1),center(2),center(3));
smoothMotionprod(phantomX,path(:,1));
for i=1:length(path)
        delay(0.35);
        smoothMotionprod(phantomX,path(:,i));
        i
end

end
