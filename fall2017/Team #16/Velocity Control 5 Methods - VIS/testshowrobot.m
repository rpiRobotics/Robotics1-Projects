clear all; close all;

ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];zv=[0;0;0];


scale = 0.5;
h1 = ez; h2 = ey; h3 = ex; h4 = ey; h5 = ex; h6 = ey; h7 = ex;
P = [0,0,0.0800;0.0810,0.0500,0.2370;0.1400,0.1425,0;0.2600,-0.0420,0;0.125,-0.1265,0;0.2750,0.0310,0;0.1100,0.1053,0;0.112,0,0]';
q = [0 0 0 0 0 0 0]';
H = [h1 h2 h3 h4 h5 h6 h7];
type = [0 0 0 0 0 0 0];
n = size(H,2);
%

[my_robot,my_robot_structure]=defineRobot(type,H,P,n,scale);

maxP = sqrt(max(sum(P.*P,1)));

long = max(sum(P,2));
ax_scle = 1.25;
axis_lim = [-long/ax_scle ax_scle*long -long/ax_scle ax_scle*long -long/ax_scle ax_scle*long];


viewport=[-32 26];

figure(1); clf; 
h_my_robot = createCombinedRobot(my_robot,my_robot_structure);
axis equal;axis(axis_lim);view(viewport);grid   

q0=(rand(n,1)-.5)*pi;
dq=ones(n,1)*pi/4;

t=(0:.1:100);
for i=1:length(t)
  
    q = q0+dq*sin(.5*t(i));
    h_my_robot = showRobot(q,h_my_robot,axis_lim,viewport);
    
end
