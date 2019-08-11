clear all
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
start_angles = int16( [0 0 0 0 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4), start_angles(5));
pause(2);
