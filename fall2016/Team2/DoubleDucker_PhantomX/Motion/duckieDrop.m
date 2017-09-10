function [  ] = duckieDrop( phantomX )
% duckieDrop: Drops the already grasped duck 
%   Opens the claw of the phantomX robot
%   
%   Input(s): current joint configuration of phantomX
%   Output(s): null. Closes the last joint (NOT USED FOR KINEMATICS) 

pose = phantomX.robot.getJointPositions();
pose = phantomX.robot.getJointPositions();
pose(5) = 550;
phantomX.robot.setJointPositions(int16( pose ));

end

