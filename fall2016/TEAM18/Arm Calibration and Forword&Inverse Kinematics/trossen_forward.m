function [Rt pt]=trossen_forward(joint_readings_3)
% theta is a 4x1 vector with joint angles for the four revolute joints of
% the trossen arm
% Ex.theta=[pi/4;pi/4;pi/4;pi/4];
% take in joint readings and convert to theta
% encoder: current readings in counts
% offset: joint readings in counts at zero position
% Encoder to degrees conversion: joint readings x 1/1023 x 300

%Convert readings into theta
offset=double([525;506;823;516]); % joint readings in counts in zero position
theta=double((double(joint_readings_3)-offset)*(1/1023)*5.236); %convert joint readings to radians

n=4;
type=[0 0 0 0];

%length vectors in inches
l0=5;
l1x=-1.5;
l1z=6;
l2=6.25;
l3=4.5;

%position vectors
p01=[0;0;l0];
p12=[0;0;0];
p23=[l1x;0;l1z];
p34=[0;0;l2];
p4T=[0;0;l3];

P=[p01 p12 p23 p34 p4T];

% Define rotation axes
h1=[0;0;1];
h2=[0;-1;0];
h3=[0;1;0];
h4=[0;-1;0];

H=[h1 h2 h3 h4];

[Rt pt]=fwdkin(theta,type,H,P,n);

