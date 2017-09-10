%
% [theta]=subproblem1(k,p,q)
%
% solve for theta from
%
% exp(k x theta) p = q
%
% input: k,p,q as R^3 column vectors
% output: theta (scalar)
%

function [theta]=subproblem1sym(k,p,q)

pp=p-(p'*k)*k;
qp=q-(q'*k)*k;

epp=pp/normsym(pp);
eqp=qp/normsym(qp);

theta=subproblem0sym(epp,eqp,k);

