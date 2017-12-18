%
% q=subprob4(k,h,p,d)
%
% solve for theta from
%
% d=h'*rot(k,q)*p
%
% input: k,h,p as R^3 vectors, k,h are unit vectors, d: scalar
% output: q (up to 2 solutions)
%

function q=subprob4(k,h,p,d)

d=d/norm(h);
h=h/norm(h);

c=d-(h.'*p+h.'*hat(k)*hat(k)*p);
a=h.'*hat(k)*p;
b=-h.'*hat(k)*hat(k)*p;

phi=atan2(b,a);

if abs(c/sqrt(a^2+b^2))>1;q=[NaN;NaN];return;end

q=zeros(2,1);
psi=asin(c/sqrt(a^2+b^2));

q(1)=-phi+psi;
q(2)=-phi-psi+pi;
