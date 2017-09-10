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

function [theta]=subproblem1(k,p,q)

if norm(p-q)<sqrt(eps);theta=0;return;end
  
k=k/norm(k);
pp=p-(p'*k)*k;
qp=q-(q'*k)*k;

epp=pp/norm(pp);
eqp=qp/norm(qp);

theta=subproblem0(epp,eqp,k);
%theta=atan2(k'*(cross(epp,eqp)),epp'*eqp);

tol=1e-2;
if (abs(norm(p)-norm(q))>tol);
  disp('*** Warning *** ||p|| and ||q|| must be the same!!!');
end
