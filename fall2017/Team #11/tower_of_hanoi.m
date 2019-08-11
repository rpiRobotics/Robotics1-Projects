clear all

robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

% [joint1 joint2 joint3 rot suction]
% if suction == 1, suction goes on
% if suction == 0, suction goes off
start_angles = int16( [0 0 0 0 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4), start_angles(5));
pause(2);

[Q] = ikdobot(220, 0, 10);
new_angles = int16(Q+start_angles);
robot.setJointPositions(new_angles(1),new_angles(2),new_angles(3),new_angles(4), start_angles(5));
robot.getJointPositions();
pause(5);

start_angles = int16( [0 0 0 0 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4), start_angles(5));
pause(2);

pole_detection; 

pole1_y = dx_12-14; 
pole2_y = 0; 
pole3_y = dx_23+30; 

% Waiting for locate the disks
[Q] = ikdobot(220, pole1_y, 50);
new_angles = int16(Q+start_angles);
robot.setJointPositions(new_angles(1),new_angles(2),new_angles(3),new_angles(4), start_angles(5));
robot.getJointPositions();
pause(8);
disp 'Please move the stack of disks under the suction cup'


% Playing the game
desired_location = location_desired(pole1_y, pole2_y, pole3_y);
disp 'Tower of Hanoi'

for i=1:1:length(desired_location)
    
    [Q] = ikdobot(desired_location(i, 1), desired_location(i, 2), desired_location(i, 3));
    new_angles = Q+start_angles;
    robot.setJointPositions(new_angles(1),new_angles(2),new_angles(3),new_angles(4), int16(desired_location(i, 4)));
    robot.getJointPositions();
    pause(1);

end

% Shake head
for i = 1:1:6
    if (i == 1) || (i == 3) || (i == 5)
        [Q] = ikdobot(220, 20, 50);
        new_angles = int16(Q+start_angles);
        robot.setJointPositions(new_angles(1),new_angles(2),new_angles(3),new_angles(4), int16(0));
        robot.getJointPositions();
        pause(0.1);
    elseif (i == 2) || (i == 4)
        [Q] = ikdobot(220, -20, 50);
        new_angles = int16(Q+start_angles);
        robot.setJointPositions(new_angles(1),new_angles(2),new_angles(3),new_angles(4), int16(0));
        robot.getJointPositions();
        pause(0.1);
    else
        [Q] = ikdobot(220, 0, 50);
        new_angles = int16(Q+start_angles);
        robot.setJointPositions(new_angles(1),new_angles(2),new_angles(3),new_angles(4), int16(0));
        robot.getJointPositions();
    end
end
start_angles = int16( [0 0 0 0 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4), start_angles(5));
