function [Rt, Pt]=trossen_forwardx(joint_readings_3)
% theta is a 4x1 vector with joint angles for the four revolute joints of
% the trossen arm
% Ex.theta=[pi/4;pi/4;pi/4;pi/4];
% take in joint readings and convert to theta
% encoder: current readings in counts
% offset: joint readings in counts at zero position
% Encoder to degrees conversion: joint readings x 1/1023 x 300

%Convert readings into theta
load('Parameters.mat')
theta=double((double(joint_readings_3)-Acal)*(1/1023)*5.236); %convert joint readingsto radians
theta=theta+[phi;0;0;0];
n=4;
type=[0 0 0 0];

P=Pcal;

% Define rotation axes
h1=[0;0;1];
h2=[0;1;0];
h3=[0;-1;0];
h4=[0;1;0];

H=[h1 h2 h3 h4];

[Rt, Pt]=fwdkin(theta,type,H,P,n);


