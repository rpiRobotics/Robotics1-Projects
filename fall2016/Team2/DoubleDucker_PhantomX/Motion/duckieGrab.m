function [  ] = duckieGrab( phantomX )
% duckieGrab: Grabs the duck 
%   Closes the claw of the phantomX robot
%   
%   Input(s): current joint configuration of phantomX
%   Output(s): null. Opens the last joint (NOT USED FOR KINEMATICS) 

pose = phantomX.robot.getJointPositions();
pose = phantomX.robot.getJointPositions();
pose(5) = 250;
phantomX.robot.setJointPositions(int16( pose ));

end

