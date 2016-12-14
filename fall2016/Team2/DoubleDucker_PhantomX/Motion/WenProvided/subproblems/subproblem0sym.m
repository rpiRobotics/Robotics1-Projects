%
% [theta]=subproblem0(p,q)
%
% solve for theta subtended between p and q
%
% input: p,q as R^3 column vectors
% output: theta (scalar)
%

function [theta]=subproblem0sym(p,q,k)

ep=p/normsym(p);
eq=q/normsym(q);

theta=simplify(2*atan(normsym(ep-eq)/normsym(ep+eq)));

