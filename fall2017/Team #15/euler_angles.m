function [ a,b, g ] = euler_angles( R )
%euler_angles: R
%Function: Euler_Angles
a = atan2(R(2,1), R(1,1));
b = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2));
g = atan2(R(3,2), R(3,3));
end