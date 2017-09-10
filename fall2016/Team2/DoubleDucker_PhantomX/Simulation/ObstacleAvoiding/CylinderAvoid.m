function [  ] = CylinderAvoid( C, r,p0T, phantomX )
%Command phantomX unit to move to target while avoiding an infinite
%cylinder in the workplane
%   Input(s):   C......... 2x1 column vector of x and y coordinates of
%                          cylinder center
%               r......... Radius of the cylinder
%               p0T....... Desired end position
%               phantomX.. Target arm
%           
%   Output(s):  []
%--------------------------------------------------------------------------
%p1=phantomX_ForwardKinematics(phantomX.getJointPositions()); %Obtain current position
p1=[.15;.15;.15] %test
z=p1(3);
d=.02; %Step distance
C1=[C;z]-[r+d;0;0]; %Intermediate point below circular object in x by d and same z.
while LineCircleIntersect(p1(1:2),C1(1:2),C,r) || LineCircleIntersect(C1(1:2),p0T(1:2),C,r) %While p1 to C1 or C1 to p0T intersects cylinder:
    C1=C1-[d;0;0]; %Bring intermediate point farther from cylinder
end
Smoothmotion(phantomX, C1); %Move to intermediate point
Smoothmotion(phantomX,p0T); %Move to goal


end

