%
% [theta]=subproblem0(p,q,k)
%
% solve for theta subtended between p and q
%
% input: p,q as R^3 column vectors
% output: theta (scalar)
%

function [theta]=subproblem0(p,q,k)

if ((k'*p)>sqrt(eps)|(k'*q)>sqrt(eps))
  error('k must be perpendicular to p and q');
end

ep=p/norm(p);
eq=q/norm(q);

theta=2*atan2(norm(ep-eq),norm(ep+eq));

if k'*(cross(p,q))<0
  theta=-theta;
end
