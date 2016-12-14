function [ q, q_all, q_possible ] = phantomX_InverseKinematics(phantomX)
%phantomX_InverseKinematics: computes the inverese kinematics
%   Will solve the inverse kinematics of the phantomX 4-DOF robot arm.
%   The solution uses values set by the phantomX structure created in the
%   phantomX_Inital Script.
%
%   Input(s): desired R0T and p0T entered in phantomX_ArmPose script. 
%               Dependent on the phantomX structure and inital values.
%   Output(s)= filter joint configuration, all solutions, possible
%               solutions after first filter (joint limits).

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

q1 = subproblem1(ez,ey, R0T*ey);

LHS = p0T - p01 - rotz(q1)*p12 - R0T * p4T; %********
q3_solns = subproblem3(ey,-p34, p23,norm(LHS));
q1_solns = zeros(2,length(q3_solns));
q2_solns = zeros(2,length(q3_solns));

k = 1;
q_test = zeros(3,4);
for i = 1:length(q3_solns)
    [q1_solns(i,:),q2_solns(i,:)] = subproblem2...
        (h1,h2,(p23+rot(h3,q3_solns(i))*p34),LHS);
    for j = 1:length(q1_solns(i,:))
        q_test(:,k) = [q1_solns(i,j),q2_solns(i,j),q3_solns(i)]';
        k = k+1;
    end
end

q_possible = zeros(4,4);
for i = 1:length(q_test(i,:))
    temp =rot(h3,q_test(3,i))'*rot(h2,q_test(2,i))'*rot(h1,q_test(1,i))'...
        * (p0T-p01-rot(h1,q_test(1,i))*p12...
        - rot(h1,q_test(1,i))*rot(h2,q_test(2,i))*p23...
        - rot(h1,q_test(1,i))*rot(h2,q_test(2,i))*rot(h3,q_test(3,i))*p34);
    q4_soln = subproblem1(ey,p4T,temp);
    q_possible(:,i) = [q_test(:,i);q4_soln];
end

% Rads -pi to pi Check
for i = 1:length(q_possible(:,i))
    for j = 1:length(q_possible(j,:))
        if pi < q_possible(j,i)
            q_possible(j,i)=q_possible(j,i)-pi;
        elseif -pi > q_possible(j,i)
            q_possible(j,i)=q_possible(j,i)+pi;
        end
    end
end

% Servo Limit Check and conversion
q_all = q_possible;
for i = 1:length(q_all(:,i))
    for j = 1:length(q_possible(j,:))
        if phantomX.qLimits(1,j) > q_possible(j,i) ||...
                phantomX.qLimits(2,j) < q_possible(j,i)
            q_possible(:,i) = NaN;
        end
    end
end

% Solution Filter
q = [];
j = 1;
for i = 1:length(q_possible(:,i))
    if isnan(q_possible(1,i))
        j = j+1;
    else
        q = q_possible(:,i);
        
        % Check Filtered Solution with Forward Kinematics 
        R_test=eye(3,3);p_test=zeros(3,1);Jtr_test=[];
        for m=1:phantomX.n+1
            [R_test,p_test,Jtr_test]=fwdkinstep(R_test,p_test,...
                Jtr_test,[q_possible(:,i);0],phantomX.H,...
                phantomX.jointtype,phantomX.PVect,m);
        end
        if abs(p_test - p0T) < 0.001
            q = q_possible(:,i);
        end
    end
end
end
