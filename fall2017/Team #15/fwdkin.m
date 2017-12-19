function [ R, p ] = fwdkin( q, type, H, P, n )
%% fwdkin: Solve the FK for any serial robot
%% Author: Garrison Johnston
%% INPUTS:
% q: vector of joint variables, q = [q1; q2;...; qn]
% type: vector of joint types, 0 = revolute, 1 = prismatic. 
%    EX: RPR arm, type = [0;1;0];
% H: 3xn matrix where a column i is the rotation axis of joint i if joint i is
%   a revolute joint or the displacement axis of joint i is a prismatic joint 
% P: 3xn matrix where colum i is the position vector from joint i to i+1 
% n: number of joints
%% OUTPUTS:
% R: Rotation Matrix R0T
% p: Position Vector p0T
%% Main
p = zeros(3,1);
R = eye(3);
for i = 1:1:n
    if type(i) == 0
        Ri = rot(H(:,i), q(i));
    else 
        Ri = eye(3);
    end
    R = R*Ri;
    p = p + R*P(:,i);
end
p = p + R*P(:,n+1);
end
