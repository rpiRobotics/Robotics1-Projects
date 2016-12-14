%
% [theta1,theta2]=subproblem2sym(k1,k2,p,q)
%
% solve for theta1 and theta2 from
%
% exp(k1 x theta1) * exp(k2 x theta2) p = q
%
% input: k1,k2,p,q as R^3 column vectors
% output: theta1 and theta2 as 2x1 columns corresponding to the two solutions
%

function [theta1,theta2]=subproblem2sym(k1,k2,p,q)

k12=k1'*k2;
pk=p'*k2;
qk=q'*k1;

% check if solution exists

%if abs(k12^2-1)<eps;theta1=[];theta2=[];
%disp('no solution (***1***)');
%return;end

a=[k12 -1;-1 k12]*[pk;qk]/(k12^2-1);

bb=(p.'*p-a.'*a-2*a(1)*a(2)*k12);
%if abs(bb)<eps;bb=0;end
%if bb<0;theta1=[NaN NaN];theta2=[NaN NaN];
%disp('no solution (***2***)');
%return;end

% check if there is only 1 solution
gamma=sqrt(bb)/(cross(k1,k2).'*cross(k1,k2));
% if abs(gamma)<eps;
%   c1=[k1 k2 cross(k1,k2)]*[a;gamma];
%   theta2=[subproblem1(k2,p,c1) NaN];
%   theta1=[-subproblem1(k1,q,c1) NaN];  
% end  

% general case: 2 solutions

%theta1=zeros(2,1);
%theta2=zeros(2,1);

c1=[k1 k2 cross(k1,k2)]*[a;gamma];
c2=[k1 k2 cross(k1,k2)]*[a;-gamma];
theta2(1)=subproblem1sym(k2,p,c1);
theta2(2)=subproblem1sym(k2,p,c2);

theta1(1)=-subproblem1sym(k1,q,c1);
theta1(2)=-subproblem1sym(k1,q,c2);

