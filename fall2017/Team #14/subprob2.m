%
% [q1,q2]=subprob2(k1,k2,p1,p2)
%
% solve for theta1 and theta2 from
%
% exp(k1 x q1) p1 = exp(k2 x q2) p2 
%
% input: k1,k2,p1,p2 as R^3 column vectors
%
% output: q1 and q2 as 2x1 columns corresponding to the two solutions
%

function [q1,q2]=subprob2(k1,k2,p1,p2)

p2=p2/norm(p2)*norm(p1);
k12=k1'*k2;
pk1=p1'*k1;
pk2=p2'*k2;

% check if solution exists

if abs(k12^2-1)<eps;theta1=[];theta2=[];
    q1=[NaN;NaN];q2=[NaN;NaN];
    disp('no solution (k1 and k2 are collinear)');
    return;
end

a=[1 -k12; -k12 1]*[pk1;pk2]/(1-k12^2);

% 
% check if solution exists
%
cond=(norm(p1)^2-norm(a)^2-2*a(1)*a(2)*k12);

% special case: 1 solution
if abs(cond)<eps;
  v=[k1 k2]*a;
  q1a=subprob1(k1,p1,v);
  q2a=subprob1(k2,p2,v);
  q1=[q1a;q1a];
  q2=[q2a;q2a];
end

% special case: no solution
if cond<0
    q1=[NaN NaN];q2=[NaN NaN];
    disp('no solution (two cones do not intersect)');
    return;
end

gamma=sqrt(cond)/norm(cross(k1,k2));

% general case: 2 solutions

q1=zeros(2,1);
q2=zeros(2,1);

v1=[k1 k2 cross(k1,k2)]*[a;gamma];
v2=[k1 k2 cross(k1,k2)]*[a;-gamma];
q1(1)=subprob1(k1,p1,v1);
q1(2)=subprob1(k1,p1,v2);

q2(1)=subprob1(k2,p2,v1);
q2(2)=subprob1(k2,p2,v2);
