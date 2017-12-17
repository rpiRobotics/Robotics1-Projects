clear
%% connect to robotraconteur
load('connInfo.mat');
robot = RobotRaconteur.Connect(RobotAddr);
mode = uint8(0);
robot.setControlMode(mode);
%% set to zero configuration
%  q0R = [0   0    0    0   0   0   0]';
%  robot.setJointCommand('right',q0R);

%% set to neutral
% q0R = [0,-1.18,0,2.18,0,0.57,3.3161]';
% robot.setJointCommand('right',q0R);
% robot.setNeutral();
% robot.setJointCommand('right',[pi/2,-0.3,0,0.58,0,-0.31,0.15]');
% robot.setJointCommand('right',[1.6924765625;-0.487662109375;-0.2751298828125;1.02378125;-0.008703125;-0.545306640625;0.3953486328125]);
% robot.setJointCommand('right',[0,0,0,0,0,0,0]');
robot.setJointCommand('right',[1.9054716796875,-0.4835458984375,-0.286734375,1.127244140625,0.3376572265625,-0.5959150390625,0.115078125]');
%% set to arbitary angle
% q0R = [-0.3062,-0.3999,0.7959,1.0667,0.1004,-0.5731,-0.8040]';
% robot.setJointCommand('right',q0R);


%% updated info
joint_positions = robot.joint_positions;
joint_velocities = robot.joint_velocities;
joint_torques = robot.joint_torques;
endeffector_positions = robot.endeffector_positions;
endeffector_orientations = robot.endeffector_orientations;
endeffector_twists = robot.endeffector_twists;
endeffector_wrenches = robot.endeffector_wrenches;


% robot.joint_positions

% robot.setNeutral();