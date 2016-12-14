% PHANTOMX Initial Parameters

% Run this File and Save phantomX struct to be loaded into workspace
% Note: Clears Workspace at the end except PhantomX struct

% Set up Initial Parameters
I3=eye(3,3); ex=I3(:,1); ey=I3(:,2); ez=I3(:,3); zv=[0;0;0];

phantomX.n = 4; % Degrees of Freedom (4-DOF)
phantomX.jointtype = [0 0 0 0 1]; % 4R Robot (RRRR)

% Joint Contraints
% Lower Limits, in degs
q1_L=(-90); q2_L=(-90); q3_L=(-110); q4_L=(-90);
% Upper Limits, in degs
q1_H=(90); q2_H=(90); q3_H=(90); q4_H=(90);
%Converted to rads
phantomX.qLimits = (pi/180)*[q1_L, q2_L, q3_L, q4_L;
    q1_H, q2_H, q3_H, q4_H];

% Joint Rotation Vectors / Axis of Rotation
h1=ez; h2=ey; h3=ey; h4=ey;
phantomX.H=[h1 h2 h3 h4 h4];

% Joint Lenghts [m]
%l0=0.09; l1=0.025; l2=0.145; l3=0.038; l4=0.145; l5=0.09; % mp
l0 = .085; l1 = .03; l2 = .145; l3 = .04; l4 = .145; l5 = .09; % js
phantomX.JL = [l0 l1 l2 l3 l4 l5];

% Position Vectors
%p01=l0*ez; p12=l1*ez; p23 =(l2*ez)+(l3*ex); p34=l4*ex; p4T=l5*ex; % mp
p01 =ez*(l0+l1); p12=zv; p23=(ez*l2)+(ex*l3); p34 =ex*l4; p4T=-ez*l5; %js vert
%p01 =ez*(l0+l1); p12=zv; p23=(ez*l2)+(ex*l3); p34 =ex*l4; p4T=ex*l5; %js horz

phantomX.PVect = [p01, p12, p23, p34, p4T];

% Initalize Joint Angles
q1=0; q2=0; q3=0; q4=0;
phantomX.q = [q1;q2;q3;q4]; % in rads. Used for current joint configuration
phantomX.q_des = phantomX.q; % initialize desired joint configuration

% Initialize Rotation Matrices
phantomX.R01 = rot(h1,q1); 
phantomX.R12 = rot(h2,q2); 
phantomX.R23 = rot(h3,q3); 
phantomX.R34 = rot(h4,q4);

% Initalize R0T, p0T, J0T
phantomX.R0T = eye(3,3);
phantomX.p0T = zeros(3,1);
phantomX.J0T = zeros(4,6);

% Connect to PhantomX and RR Server
phantomX.robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');

clearvars -except phantomX

pause(3)

phantomX = Home(phantomX);
duckieDrop( phantomX )

