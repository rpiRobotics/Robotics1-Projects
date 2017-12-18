%
% q=subprob1(k,p1,p2)
%
% solve for q from
%
% exp(k x q) p1 = p2
%
% input: k,p1,p2 as R^3 column vectors
% output: q (scalar)
%

function q=subprob1(k,p1,p2)

p2=p2/norm(p2)*norm(p1);

if norm(p1-p2)<sqrt(eps);q=0;return;end
  
k=k/norm(k);
pp1=p1-(p1'*k)*k;
pp2=p2-(p2'*k)*k;

epp1=pp1/norm(pp1);
epp2=pp2/norm(pp2);

q=subprob0(k,epp1,epp2);
%q=atan2(k'*(cross(epp1,epp2)),epp1'*epp2);
