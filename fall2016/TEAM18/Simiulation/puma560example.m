%
% Inverse Kinematics for PUMA 560
%

clear all;close all;

ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];zv=[0;0;0];
d1=.6604;d3=.1495;d4=.4320;l2=.432;l3=.0203;d6=.0565;
h1=ez;h2=-ey;h3=-ey;h4=ez;h5=-ey;h6=ez;
p01=d1*h1;p12=zv;p23=-d3*ey+l2*ex;p34=-l3*ex+d4*ez;
p45=zv;p56=zv;p6T=d6*ez;
 
H=[h1 h2 h3 h4 h5 h6];
P=[p01 p12 p23 p34 p45 p56 p6T];
jointtype=[0 0 0 0 0 0]; % 6R robot
n=6;

%N=1000;
N=1;
for j=1:N
%q=(rand(6,1)-.5)*2*pi/2;
%q=[pi/4;pi/4;pi/4;0;pi/4;0];
%q=[0;0;0;0;pi/2;0];
q=zeros(6,1);

[p,R]=fwdkinrecursion(1,eye(3,3),q,jointtype,H,P,n);
[R1,p1]=fwdkin(q,jointtype,H,P,n);

% solve for q3

p16=p-R*p6T-p01;
q3vec=subproblem3(h3,-p34,p23,norm(p16));
q3a=q3vec(1);q3b=q3vec(2);

% solve for (q1,q2)
[q1avec,q2avec]=subproblem2(h1,h2,p23+expm(hat(h3)*q3a)*p34,p16);
[q1bvec,q2bvec]=subproblem2(h1,h2,p23+expm(hat(h3)*q3b)*p34,p16);

q1a1=q1avec(1);q1a2=q1avec(2);q2a1=q2avec(1);q2a2=q2avec(2);
q1b1=q1bvec(1);q1b2=q1bvec(2);q2b1=q2bvec(1);q2b2=q2bvec(2);

qsol=zeros(6,9);
qsol(:,9)=q;
qsol(1:3,1)=[q1a1;q2a1;q3a];
qsol(1:3,2)=[q1a2;q2a2;q3a];
qsol(1:3,3)=[q1b1;q2b1;q3b];
qsol(1:3,4)=[q1b2;q2b2;q3b];

% wrist angles

for i=1:4
  qsol(1:3,i+4)=qsol(1:3,i);
  R03=rot(h1,qsol(1,i))*...
      rot(h2,qsol(2,i))*...
      rot(h3,qsol(3,i));
  [q4vec,q5vec]=subproblem2(h4,h5,h6,R03'*R*h6);
  q4a=q4vec(1);q4b=q4vec(2);
  q5a=q5vec(1);q5b=q5vec(2);
  qsol(4:5,i)=[q4a;q5a];
  qsol(4:5,i+4)=[q4b;q5b];

  R05a=R03*rot(h4,q4a)*rot(h5,q5a);
  R05b=R03*rot(h4,q4b)*rot(h5,q5b);

  R56=R05a'*R;
  qsol(6,i)=atan2(.5*h6'*vee(R56-R56'),...
      .5*(trace(R56)-1));
  R56=R05b'*R;
  qsol(6,i+4)=atan2(.5*h6'*vee(R56-R56'),...
      .5*(trace(R56)-1));
end



% check forward kinematics

 for i=1:8
   qq=qsol(:,i);
   ii=num2str(i);
   eval(['[RR',ii,',pp',ii,']=fwdkin(qq,jointtype,H,P,n);']);
   eval(['errp(j,i)=norm(pp',ii,'-p);']);
   eval(['errR(j,i)=norm(R''*RR',ii,'-eye(3,3),''fro'');']);
 end

 % visualization
 
 scale=.5;

[my_robot,my_robot_structure]=defineRobot(jointtype,H,P,n,scale);

maxP = sqrt(max(sum(P.*P,1)));
fixaxis=[-1 1 -1 1 0 2]*maxP;
%viewpoint=[-32 26];
viewpoint=[-4 36];

figure(1); clf; 
h_my_robot = createCombinedRobot(my_robot,my_robot_structure);
axis equal;axis(fixaxis);view(viewpoint);grid   
title('true solution');

h_my_robot=showRobot(q,h_my_robot,fixaxis,viewpoint);

for l=1:8

    figure(l+1); clf; 
    h_my_robot = createCombinedRobot(my_robot,my_robot_structure);
    axis equal;axis(fixaxis);view(viewpoint);grid
    h_my_robot=showRobot(qsol(:,l),h_my_robot,fixaxis,viewpoint);

    switch l
        case 1
            title('l = 1, shoulder right, elbow up, wrist up');
        case 2
            title('l = 2, shoulder left, elbow down, wrist up');
        case 3
            title('l = 3, shoulder right, elbow down, wrist up');
        case 4
            title('l = 4, shoulder left, elbow up, wrist up');
        case 5
            title('l = 5, shoulder right, elbow up, wrist down');
        case 6
            title('l = 6, shoulder left, elbow down, wrist down');
        case 7
            title('l = 7, shoulder right, elbow down, wrist down');
        case 8
            title('l = 8, shoulder left, elbow up, wrist down');
    end
%     disp('press any key to contiue ');
%     pause;
%     disp(l);
%     h_my_robot=showRobot(qsol(:,l),h_my_robot,fixaxis,viewpoint);
%     axis equal;axis(fixaxis);view(viewpoint)
%     if l==8;l=1;end
end

end

% for i=1:8
%   figure(i);plot((1:N),errp(:,1),'x');
%   figure(10+i);plot((1:N),errR(:,1),'x');
%   disp(sprintf('sol %d p0T error: %0.5g',i,norm(errp(:,i))));
%   disp(sprintf('sol %d R0T error: %0.5g',i,norm(errR(:,i))));
% end
