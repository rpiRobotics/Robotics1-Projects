function [ qv1 , qv2 ] = invkinPEN( p , d )
% invkindobot finds required joint angles, in radians
% d is pen length
% returns 2 solutions in vector form
% the first solution should be always within the joint angles
ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];
l1 = 103;
l2 = 135;
l3 = 160;
Lg = 56;  %length from point T to q4 rotation

q1i=atan(p(2)/p(1));
p=p-l1*ez-[Lg*cos(q1i);Lg*sin(q1i);-d]; % remove const vectors
q3s=subprob3(ey,-l3*ex,l2*ez,norm(p));
q21=subprob1(ey,l2*ez+rotk(ey,q3s(1))*(l3*ex),rotk(-ez,q1i)*(p));
q22=subprob1(ey,l2*ez+rotk(ey,q3s(2))*(l3*ex),rotk(-ez,q1i)*(p));
qv2=[q1i,q21,q3s(1)+q21]';
qv1=[q1i,q22,q3s(2)+q22]';
end

% run this code to test:
% q1=rand*1.5*pi-.75*pi;q2=rand*pi/2;q3=rand*pi/2;[p,R]=fwdkinPEN(q1,q2,q3,115);[qv1,qv2]=invkinPEN(p,115);[[q1 q2 q3]' qv1 qv2]
% first column is prescribed q, second and third column are solutions.
% first and second column match