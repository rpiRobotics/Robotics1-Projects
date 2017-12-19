%% Phantom Omni Baxter Project
% Authors: Jeffrey Chen, Garrison Johnston, Stephen Sutton
% This script initializes all Robot Raconteur connections and conducts the
% functions that are necessary to control the Baxter Robot from Phantom
% Omni
disp('PhantomOmniStartUp');

%Initializes both Baxter and Omni communications
baxter = RobotRaconteur.Connect('tcp://192.168.0.41:11333/BaxterWorkspaceServer/Baxter');
omni = RobotRaconteur.Connect('tcp://127.0.0.1:5150/PhantomOmniSimulinkHost/PhantomOmni');

%Initalize constants
ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1]; zz = [0;0;0];

%% Jacobian of Phantom Omni
h1 = -ez; h2 = -ey; h3 = -ey;
p01 = zz; p12 = zz; p23 = .15*ex; p3T = .17*ex + .02*-ez;

type = zeros(3,1);
P = [p01 p12 p23 p3T];
H = [h1 h2 h3];
n = 3;

disp('Phantom Omni Initialized, Forward Kinematics Calculated')

%% Baxter Control
baxter.setControlMode(uint8(0));


% Set Baxter to initial desired position
q = [0;-pi/4;0;pi/4;0;pi/2;0];
baxter.setJointCommand('right',q);
alpha = 1.5;
disp('Baxter initialized and Set to Initial Position')
pause(2);

while(1)

    % Initial Forward Kinematics of Phantom Omni
    q = callAngles(omni);
    q = q(1:3);
    [R0T, P0T] =fwdkin(q, type,H,P,n);

    pause(.1)
    % Second Forward Kinematics of Phantom Omni
    q = callAngles(omni);
    q = q(1:3);
    [R0T_d, P0T_d] = fwdkin(q,type,H,P,n);
    delt_P0T = alpha*(P0T_d-P0T);
    R_quat = [.49848584, 0.86663104, 0.02108932,-0.00421412];
    
    %Sending control to Baxter Robot
    baxter.setWorkspaceCommand('right',double(delt_P0T),double(R_quat'));

    wrenches = baxter.endeffector_wrenches;
    J = callJacobian(q);
    Torques = J'*wrenches(1:3,1);
    setTorques(omni,Torques);
end