function [ q ] = smoothMotion( phantomX, p0T)
% smoothMotion: Smoothly move arm to desired p0T. Position Based
%   Converges the current joint configuration to a desired joint
%   configuration. Moves iteratively towards desired joint configuration.
%   Path following generated from Trapezoidal Trajectory
%   Tunning for can be used for Arm Calibration.
%   
%   Input(s): Current joint configuration of phantomX and desired joint
%               configuration from inverse kinematics.
%   Output(s): q, new joint configuration which was moved to
%--------------------------------------------------------------------------
v0=0; vf=0; vm=.005; am=.005/2; dm=am; n=50;
path = zeros(3,n+1);
posepath=zeros(4,n+1);
phantomX.p0T = p0T;
phantomX.R0T = rotz(atan(p0T(2)/p0T(1)));
%Testing
pos_curr=[0.15;0.15;0.15];
pos_curr=[0.105;0;0.15];
%\Testing

% pose_curr = phantomX.robot.getJointPositions();
% pose_curr = phantomX.robot.getJointPositions();
% pose_curr = [348;684;512;400;300]; %test
%pose_curr = [348;684;512;400;300]; %test
% phantomX.q=step2rad(pose_curr);
% [l1,pos_curr,l2]=phantomX_ForwardKinematics(phantomX);
%pose_des = rad2step(phantomX_InverseKinematics_JS(phantomX)); % JS
%pose_des = rad2step(phantomX_InverseKinematics_MP(phantomX)); % MP
pos_des=p0T;
for i=1:3
    path(i,:) = trapezoidal_generation_v2(double(pos_curr(i)),...
        double(pos_des(i)),v0,vf,vm,am,dm,n);
end

%path(5,:) = pose_curr(5);
%path=path(:,end);  %For direct motion... janky
p=phantomX;
for i=1:length(path)
    p.p0T=path(:,i);
    posepath(:,i)=phantomX_InverseKinematics_JS(p);
end
%Need to change below:
for k = 1:size(path,2)
    %delay(.045);
    phantomX.robot.setJointPositions(int16(posepath(:,k)));
end
% 
% pose_final = phantomX.robot.getJointPositions();
% pose_final = phantomX.robot.getJointPositions();
% q = step2rad(pose_final(1:4)); % convert ending joint config to rads

end

