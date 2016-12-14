function [ out ] = LineCircleIntersect( P1,P2,C,r )
%Determines if a line between points P1 and P2 will intersect a circle 
%   Input(s):   P1,P2,C... 2x1 column vectors of x and y coordinates of
%                          points 1 and 2, and the center of the circle,
%                          respectively
%               r......... Radius of circle              
%           
%   Output(s):  out....... Boolean denoting if an intersection exists
%--------------------------------------------------------------------------
x=linspace(P1(1),P2(1)); %Vector of x values between p1 and p2
y=linspace(P1(2),P2(2)); %Vector of y values between p1 and p2
out=false; %Initialize output
for i=1:100 %For all points on line between p1 and p2
    vc=[x(i);y(i)]-C; %Vector from ith coordinate to center C
    if norm(vc) <=r %Check if distance from center to ith coordinate less than radius
        out=true;
    end
end

end

