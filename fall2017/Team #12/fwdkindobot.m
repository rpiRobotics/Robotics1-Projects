function [ p, R ] = fwdkindobot( q1, q2, q3 , d)
%fwdkindobot Outputs rotation and distance from origin
%   from Dobot's joint angles
ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];

%all lengths are in mm
l1 = 103;
l2 = 135;
l3 = 160;
Lg = 56;  %length from point T to q4 rotation
%d = 115;  %pen length

R = rot(ez, q1)*rot(ex, q2)*rot(ex, q3);
p = l1*ez + rot(ez,q1)*rot(ey,q2)*l2*ez + rot(ez,q1)*rot(ey,q2)*rot(ey,q3)*l3*ex+[Lg*cos(q1);Lg*sin(q1);-d];
end