function [ circ ] = Circle3( r, e,xc,yc,zc )
%Create a collection of points describing a circle in 3-space
%   Input(s):   r........ Circle radius
%               e........ Normal vector to the circle plane
%               xc....... X-coordinate of center
%               yc....... Y-coordinate of center
%               zc....... Z-coordinate of center
%           
%   Output(s):  circ..... Collection of 3x1 column vectors describing each
%                         point in the 3D circle
%--------------------------------------------------------------------------
t=linspace(0,2*pi,100); 
x=r*cos(t); %Generate x,y,z of planar circle orthogonal to vertical vector n
y=r*sin(t);
z=0*t;
n=[0;0;1];
e1=e/norm(e); %Take unit vector colinear to e
v=cross(n,e1); %Crossproduct of n and e1
s=norm(v); %Sine of angle between n and e1
if s~=0 %If e1 not parallel to n
    c=dot(n,e1); %Cosine of angle between n and e1
    R=eye(3)+CrossMat(v)+CrossMat(v)^2*(1-c)/s^2; %Rotation matrix from n to e1
    circ=R*[x;y;z]; %Rotate planar circle points to plane orthogonal to e1
else
    circ=[x;y;z];
end
circ(1,:)=circ(1,:)+xc; %Move circle center to desired coordinates
circ(2,:)=circ(2,:)+yc;
circ(3,:)=circ(3,:)+zc;

end

