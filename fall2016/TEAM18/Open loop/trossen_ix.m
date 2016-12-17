function [thetaV]=trossen_ix(r0T,p0T)

load('Parameters.mat');
%trossen parameters
ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];

%position vectors [X; 0 ;z] where  is the horizontal distance 
p01=Pcal(:,1);
p12=Pcal(:,2);
p23=Pcal(:,3);
p34=Pcal(:,4);
p4T=Pcal(:,5);
p0T=p0T;

thetaq1=atan2(p0T(2),p0T(1));

r2T=rotz(phi)^-1 * rotz(thetaq1)^-1 * r0T;
Q=atan2(r2T(2,1),r2T(1,1));
p1T=p0T - p01;
p2T=p1T-p12;
X=sqrt(p2T(1)^2 + p2T(2)^2);
p2T=[X;0;p2T(3)];
p24=p2T-(r2T*p4T);
L=norm(p24);
a=norm(p23);
b=norm(p34);
%thetaq3=pi-acos(((a^2)+(b^2)-(L^2))/(2*a*b))+acos(p23(3)/a)-acos(p34(3)/b);
thetaq3=pi-acos(((a^2)+(b^2)-(L^2))/(2*a*b))-acos(p23(3)/a)-acos(p34(3)/b);
arm=p23+roty(thetaq3)*p34;
armoffset=atan2(arm(3),arm(1));
desired=atan2(p24(3),p24(1));
thetaq2=armoffset-desired;
thetaq4=Q-thetaq2-thetaq3;
thetaq1=thetaq1-phi;
thetaf=[thetaq1;thetaq2;-thetaq3;thetaq4];
thetaV=round((thetaf*1023/5.236)+ Acal);










