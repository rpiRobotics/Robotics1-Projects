function [ p, R ] = fwdkinPEN( q1, q2, q3 , d)
%fwdkindobot Outputs rotation and distance from origin
% d is pen length
ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];

% all lengths are in mm
l1 = 103;
l2 = 135;
l3 = 160;
Lg = 56;  %length from point T to q4 rotation

R = rotk(ez, q1)*rotk(ex, q2)*rotk(ex, q3);
p = l1*ez + rotk(ez,q1)*rotk(ey,q2)*l2*ez + rotk(ez,q1)*rotk(ey,q3)*l3*ex+[Lg*cos(q1);Lg*sin(q1);-d];
end