function p0g = fkdobot( q1,q2,q3 )
%fkdobot computes forward kinematics for dobot 
%   takes desired q1,q2,q3 and returns gripper x,y,z
%% constants needed
l1 = 103;
l2 = 135;
l3 = 160;
Lg = 56;  %length from point T to q4 rotation
d = 115;  %length from q4 to gripper end

q1 = q1*pi/180;
q2 = q2*pi/180;
q3 = q3*pi/180;

ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1]; 
p01 = l1*ez; p12 = 0*ez; p23 = l2*ez; p34=l3*ex; p4t=0*ez;
h1=ez; h2=1*ey; h3=ey; h4=ez;
r01=rot(h1,q1); r12=rot(h2,q2); r23=rot(h3,q3); r34=rot(h4,q4);
p0t = p01+rotz(q1)*(p12+roty(q2)*p23+roty(q3)*p34);


xg = Lg*cos(q1);
yg = Lg*sin(q1);
p0g = p0t+[xg;yg;-d];
end

