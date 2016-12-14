clc; clear; close;
Q = zeros(1,3,16);
[Q(:,:,1),~] = ikdobot(180,0,10); %start position
[Q(:,:,2),~] = ikdobot(180,0,75); %move above duck
[Q(:,:,3),~] = ikdobot(250,0,75); %move to duck position
[Q(:,:,4),~] = ikdobot(250,0,40); %lower onto duck
Q(:,:,5) = Q(:,:,4); %close gripper
[Q(:,:,6),~] = ikdobot(250,0,75); %pick up duck at its location
[Q(:,:,7),~] = ikdobot(145,-170,75); %move toward goal
[Q(:,:,8),~] = ikdobot(145,-170,110); %move above obstacle
[Q(:,:,9),~] = ikdobot(90,-225,115); %move toward goal
[Q(:,:,10),~] = ikdobot(90,-225,55); %lower into goal
Q(:,:,11) = Q(:,:,10); %open gripper
Q(:,:,12) = Q(:,:,9); %move out of goal
Q(:,:,13) = Q(:,:,8); %move away from goal
[Q(:,:,14),~] = ikdobot(145,-170,50); %lower outside of goal
[Q(:,:,15),~] = ikdobot(250,0,50); %move toward start
Q(:,:,16) = Q(:,:,1); %return to start
rot = 0; 
grip = [25 25 25 25 53 53 53 53 53 53 25 25 25 25 25 25];
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(rot),int16(50))
pause(0.1)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(rot),int16(50))
uiwait(msgbox({'Ready to go?'}));

for ii = 2:16
    robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(rot),int16(grip(ii)))
    pause(2)
end
