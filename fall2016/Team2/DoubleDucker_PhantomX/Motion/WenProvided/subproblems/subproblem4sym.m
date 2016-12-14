%
% [theta]=subproblem4sym(p,q,k,d)
%
% solve for theta from
%
% d=p'*rot(k,theta)*q
%
% input: p,q as R^3 vectors, k is a unit vector, d: scalar
% output: theta (up to 2 solutions)
%

function [theta]=subproblem4sym(p,q,k,d)

c=d-(p.'*q+p.'*hat(k)*hat(k)*q);
a=p.'*hat(k)*q;
b=-p.'*hat(k)*hat(k)*q;

phi=atan2(b,a);

%if abs(c/sqrt(a^2+b^2))>1;theta=[NaN;NaN];return;end

%theta=zeros(2,1);
psi=asin(c/sqrt(a^2+b^2));

theta(1)=-phi+psi;
theta(2)=-phi-psi+pi;
