function [thetaf]=trossen_inverse(R04,P0T)
% R0T is given
% it is a 3x3 matrix determined by rotz(q1)rotx(q2+q3+q4)
%P0T is given
r04=R04;
p0T=P0T;

%trossen parameters
ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];

%length vectors in inches
l0=5;l1x=-1.5;l1z=6;l2=6.25; l3=4.5;

%position vectors
p01=[0;0;l0];p12=[0;0;0];p23=[l1x;0;l1z];p34=[0;0;l2];p4T=[0;0;l3];

% Step 1: Use subproblem 1 to get q1
kq1=ez;pq1=ey;qq1=r04*ey;

thetaq1=subproblem1(kq1,pq1,qq1);

% Step 2:Use subproblem 3 to get q3
dq3=norm(p0T-p01-(r04*p4T));
pq3=r23*p34; qq3=p23;

[thetaq3]=subproblem3(kq3,pq3,qq3,dq3); % note there will be multiple solutions

% Step 3:Use subproblem 1 to get q2
R01=rotz(thetaq1); 
r23=roty(-thetaq3); 

kq2=ey;
pq2=p23+(r23*p34); 
qq2=transpose(R01)*((p0T-p01-(r04*p4T)));

thetaq2=subproblem1(kq2,pq2,qq2);

% Step 4: Use subproblem 1 to get q4
r03=rotz(thetaq1)*rotx(thetaq2+thetaq3(1));
pq4=r0T*transpose(r03)*ey;
qq4=ey;
kq4=ey;
thetaq4=subproblem1(kq4,pq4,qq4);

%define final theta vector
thetaf(1,1)=thetaq1;
thetaf(2,1)=thetaq2;
thetaf(3,1)=thetaq3;
thetaf(4,1)=thetaq4;