%
% q=subprob3(k,p1,p2,d)
%
% solve for theta from
%
% norm(p2-exp(k x q) p1) = d
%
% input: k,p1,p2 as R^3 column vectors, delta: scalar
% output: q (2x1 vector, 2 solutions)
%

function q=subprob3(k,p1,p2,d)

pp1=p1-k'*p1*k;
pp2=p2-k'*p2*k;
dpsq=d^2-(k'*(p1-p2))^2;

if dpsq<0;theta=[NaN;NaN];return;end

if dpsq==0;theta=subprob1(k,pp1/norm(pp1),pp2/norm(pp2));return;end
  
bb=(norm(pp1)^2+norm(pp2)^2-dpsq)/(2*norm(pp1)*norm(pp2));
if abs(bb)>1; theta=[NaN;NaN];return;end

phi=acos(bb);

q0=subprob1(k,pp1/norm(pp1),pp2/norm(pp2));
q=zeros(2,1);

q(1)=q0+phi;
q(2)=q0-phi;
