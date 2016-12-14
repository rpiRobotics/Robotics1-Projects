function [ q ] = phantomX_InverseKinematics_JS(phantomX)
%phantomX_InverseKinematics: computes the inverese kinematics
%   Will solve the inverse kinematics of the phantomX 4-DOF robot arm.
%   The solution uses values set by the phantomX structure created in the
%   phantomX_Inital Script.
%
%   Input(s): desired R0T and p0T entered in phantomX_ArmPose script. 
%               Dependent on the phantomX structure and inital values.
%   Output(s)= joint angles

% Initialize Joint Angles [rads], to be solved for
q0 = []; q1 = []; q2 = []; q3 = []; q4 = [];

% Set up Initial Parameters
I3=eye(3,3); ex=I3(:,1); ey=I3(:,2); ez=I3(:,3); zv=[0;0;0];

% Import PhantomX parameters
% Joint Rotation Vectors / Axis of Rotation
H = phantomX.H;
h1=H(:,1); h2=H(:,2); h3=H(:,3); h4=H(:,4);

R0T = phantomX.R0T;
p0T = phantomX.p0T;
PVect = phantomX.PVect;
p01=PVect(:,1);
p12=PVect(:,2);
p23=PVect(:,3);
p34=PVect(:,4);
p4T=PVect(:,5);

q1 = subproblem1(ez,ey,R0T * ey);
LHS = p0T - R0T * p4T - p01;
q31 = subproblem3(ey,-p34, p23,norm(LHS));
q3size = size(q31);

if q3size(1) >1
    [q21] = subproblem1(ey,p23 + roty(q31(1))* p34, rotz(-q1) * LHS);
    [q22] = subproblem1(ey,p23 + roty(q31(2))* p34, rotz(-q1) * LHS);
    q3 = [q31(1) q31(2)];
    q2 = [q21 q22];
elseif q3size(1) == 0
    q1 = 0;
    q2 = 0;
    q3 = 0;
    q4 = 0;
    q0 = 0;
    return;
else
     [q2] = subproblem1(ey,p23 + roty(q31)* p34, rotz(-q1) * LHS);
    q3 = q31;
end

st = size(q2);
for i = 1:st(2)
    ch1 = roty(-q3(i))*roty(-q2(i))*rotz(-q1) * R0T * ex;
    q41 = subproblem1(ey,ex,ch1);
    q4 =[ q4 q41 ];
end
tempid = [];
for i = 1:st(2)
   temp = rotz(q1) * roty(q2(i)) * roty(q3(i)) * roty(q4(i));
   v11 = temp(1,1) - R0T(1,1);
   v12 = temp(1,2) - R0T(1,2);
   v13 = temp(1,3) - R0T(1,3);
   v21 = temp(2,1) - R0T(2,1);
   v22 = temp(2,2) - R0T(2,2);
   v23 = temp(2,3) - R0T(2,3);
   v31 = temp(3,1) - R0T(3,1);
   v32 = temp(3,2) - R0T(3,2);
   v33 = temp(3,3) - R0T(3,3);
   if (v11 < .0001 && v12 < .0001 && v13 < .0001 && v21 < .0001 && ...
           v22 < .0001 && v23 < .0001 && v31 < .0001 && v32 < .0001 && ...
           v33 < .0001)
      tempid = [tempid i]; 
   end
end
t = size(tempid);
if t(2) > 1
    if (q2(1) > pi/2) || (q3(1) > pi/2) || (q4(4) >  pi/2)
        q0 = [q1;q2(2);q3(2);q4(2)];
    else
        q0 = [q1;q2(1);q3(1);q4(1)];
    end
elseif t(2) == 1
    q0 = [q1(tempid);q2(tempid);q3(tempid);q4(tempid)];
end
q = q0;
end
