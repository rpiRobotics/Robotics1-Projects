function [ q ] = smoothMotionLP( phantomX, p0T)
% smoothMotion: Smoothly move arm to desired p0T, uses low pass filter
%   Converges the current joint configuration to a desired joint
%   configuration. Moves iteratively towards desired joint configuration.
%   Path following generated from Trapezoidal Trajectory
%   Tunning for can be used for Arm Calibration.
%   
%   Input(s): Current joint configuration of phantomX and desired joint
%               configuration from inverse kinematics.
%   Output(s): q, new joint configuration which was moved to

v0=0; vf=0; vm=5; am=1; dm=1; n=400;
path = zeros(4,n+1);
filtpath=path;
phantomX.p0T = p0T;
phantomX.R0T = rotz(atan(p0T(2)/p0T(1)));

pose_curr = phantomX.robot.getJointPositions();
pose_curr = phantomX.robot.getJointPositions();
% pose_curr = [348;684;512;400;300]; %test
% pose_curr = [348;684;512;400;300]; %test
pose_des = rad2step(phantomX_InverseKinematics_JS(phantomX)); % JS
%pose_des = rad2step(phantomX_InverseKinematics_MP(phantomX)); % MP

for i=1:4
    path(i,:) = trapezoidal_generation_v2(double(pose_curr(i)),...
        double(pose_des(i)),v0,vf,vm,am,dm,n);
    filtpath(i,:)=Lowpass(path(i,:),0.08);
end

path(5,:) = pose_curr(5);
filtpath(5,:)=pose_curr(5);
for k = 1:length(filtpath)
    delay(.045);
    phantomX.robot.setJointPositions(int16(filtpath(:,k)));
end
% 
% pose_final = phantomX.robot.getJointPositions();
% pose_final = phantomX.robot.getJointPositions();
% q = step2rad(pose_final(1:4)); % convert ending joint config to rads

end

