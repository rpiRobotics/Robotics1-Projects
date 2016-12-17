robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');
while(1)
    P=input('[x;y;z]=');
    R=rotz(atan2(P(2,1),P(1,1))*rotx(pi));
    theta=trossen_ix(R,P);
    robot.setJointPositions[(int16([theta;500]));
end
