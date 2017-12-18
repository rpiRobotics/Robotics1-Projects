%
% q=subprob0(k,p1,p2)
%
% solve for q subtended between p1 and p2
%    k determines the sign of q
%
% input: k,p1,p2 as R^3 column vectors
% output: q (scalar)
%

function q=subprob0(k,p1,p2)

if ((k'*p1)>sqrt(eps)|(k'*p2)>sqrt(eps))
  error('k must be perpendicular to p and q');
end

p1=p1/norm(p1);
p2=p2/norm(p2);

q=2*atan2(norm(p1-p2),norm(p1+p2));

if k'*(cross(p1,p2))<0
  q=-q;
end
