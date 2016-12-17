clear all; close all;

ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];zv=[0;0;0];

%arm=1;
arm=input(['enter 1 for PUMA 560, 2 for Elbow, 3 for Dobot, 4 for ' ...
           'Tilted Elbow, 5 for prismatic base joint: ']);

switch(arm) 

    case 1
    % PUMA arm 
        d1=.6604;d3=.1495;d4=.4320;l2=.432;l3=.0203;d6=.0565;
        h1=ez;h2=-ey;h3=-ey;h4=ez;h5=-ey;h6=ez;
        p01=d1*h1;p12=zv;p23=-d3*ey+l2*ex;p34=-l3*ex+d4*ez;
        p45=zv;p56=zv;p6T=d6*ez;
        jointtype=[0 0 0 0 0 0];
        P=[p01 p12 p23 p34 p45 p56 p6T];
        H=[h1 h2 h3 h4 h5 h6];
        n=6;scale=.5;
    case 2 
    % Trossen arm 
        l0=1/2.0;l1=5.78/2.0;l2=5.5/2.0;l3=2.8/2.0;lt=1.65/2.0;l3t=l3+lt;
        H=[ez ex ex ex];
        P=[l0*ez zv l1*ez l2*ez l3t*ez];
        jointtype=[0;0;0;0];
        n=4;scale=.05;
    case 3
    %dotbot
        l0=.103;l1=0.135;l2=0.160;l3=0.059;
        H=[ez ey ey];P=[l0*ez zv l1*ez l2*ex];
        jointtype=[0;0;0]; 
        n=3;scale=1;
    case 4
    % hw arm
        l0=1;l1=1;l2=1;h3=[0;-1;-1];h3=h3/norm(h3);
        H=[ez ey h3];P=[l0*ez zv l1*ez l2*ex];
        jointtype=[0;0;0]; 
        n=3;scale =.3;
    case 5
    % rhino
        l0=1;l1=1;l2=1;
        H=[ey ez -ex];P=[l0*ey l1*ez zv l2*ey];
        jointtype=[1;0;0];
        n=3;scale=.3;
    otherwise
        break;
end 

%

[my_robot,my_robot_structure]=defineRobot(jointtype,H,P,n,scale);

maxP = sqrt(max(sum(P.*P,1)));
if sum(jointtype)>0
  fixaxis=[-1 1 -1 1 0 2]*maxP*3;
else
  fixaxis=[-1 1 -1 1 0 2]*maxP*2;
end
viewpoint=[-32 26];

figure(1); clf; 
h_my_robot = createCombinedRobot(my_robot,my_robot_structure);
axis equal;axis(fixaxis);view(viewpoint);grid   

q0=(0.25)*pi;
dq=ones(n,1)*pi/4;

t=(0:.1:100);
q = []
%% construct a joint trajectory
% SEGMENT 1
for i=1:1:150
   q = [q [0;0.2+0.8*sin(.02*i);0.2+0.8*sin(.02*i);0.2+0.8*sin(.02*i)]] 
end


% SEGMENT 2
for i=1:1:90
   q = [q [-1.8*sin(.02*i);0.2+0.8*sin(.02*150);0.2+0.8*sin(.02*150);0.2+0.8*sin(.02*150)]] 
end


% SEGMENT 3
for i=1:1:20
   q = [q [-1.8*sin(.02*90);0.2+0.8*sin(.02*150)+1.2*sin(.035*i);0.2+0.8*sin(.02*150+1.5*sin(.18*i));0.2+0.8*sin(.02*150)+1.5*sin(.025*i)]] 
end

% SEGMENT 4
for i=1:1:20
   q = [q [-1.8*sin(.02*90);0.2+0.8*sin(.02*150)+1.2*sin(.035*20);0.2+0.8*sin(.02*150+1.5*sin(.18*20));0.2+0.8*sin(.02*150)+1.5*sin(.025*20)]] 
end

% SEGMENT 5
for i=1:1:11
   q = [q [-1.8*sin(.02*90);0.2+0.8*sin(.02*150)+1.2*sin(.035*20)-1.2*sin(.05*i);0.2+0.8*sin(.02*150+1.5*sin(.18*20))-1.5*sin(.25*i);0.2+0.8*sin(.02*150)+1.5*sin(.025*20)-1.5*sin(.035*i)]] 
end

% SEGMENT 6
for i=1:1:90
   q = [q [-1.8*sin(.02*90)+1.8*sin(.02*i);0.2+0.8*sin(.02*150)+1.2*sin(.035*20)-1.2*sin(.05*11);0.2+0.8*sin(.02*150+1.5*sin(.18*20))-1.5*sin(.25*11);0.2+0.8*sin(.02*150)+1.5*sin(.025*20)-1.5*sin(.035*11)]] 
end


for i=1:1:size(q,2)
    h_my_robot = showRobot(q(:,i),h_my_robot,fixaxis,viewpoint);
end

% for i=1:(length(t))
%     q = q0+dq*sin(.5*t(i));
%     %q(1) = 0;
%     %q(3:4) = 0;
%     %q(1) = dq*sin(.5*(t(i)+length(t)/4));
%     %q(3:4) = dq*sin(.5*(t(i)+length(t)/4));
%     %h_my_robot = showRobot(q,h_my_robot,fixaxis,viewpoint);
%     
% end


