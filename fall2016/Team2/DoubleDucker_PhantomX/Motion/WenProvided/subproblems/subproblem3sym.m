%
% [theta]=subproblem3sym(k,p,q,d)
%
% solve for theta from
%
% norm(q-exp(k x theta) p) = d
%
% input: k,p,q as R^3 column vectors, delta: scalar
% output: theta (2x1 vector, 2 solutions)
%

function [theta]=subproblem3sym(k,p,q,d)

pp=p-k'*p*k;
qp=q-k'*q*k;
dpsq=d^2-(k'*(p-q))^2;

%if dpsq<0;theta=[];return;end

%if dpsq==0;theta=subproblem1(k,pp/norm(pp),qp/norm(qp));return;end
  
bb=(pp.'*pp+qp.'*qp-dpsq)/(2*sqrt(pp.'*pp)*sqrt(qp.'*qp));
%if abs(bb)>1; theta=[];return;end

phi=acos(bb);

theta0=subproblem1sym(k,pp/sqrt(pp.'*pp),qp/sqrt(qp.'*qp));
%theta=zeros(2,1);
theta(1)=theta0+phi;
theta(2)=theta0-phi;
