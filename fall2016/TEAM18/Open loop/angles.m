
robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');
%global robot
t0=clock;
while (etime(clock,t0)<120)
    thetax=double(robot.getJointPositions());
    %disp(thetax)
    theta(:,1)=thetax(1:4,1);
    [Rt, Pt]=trossen_forwardx(theta);
    disp(Pt)
    pause(0.5)
end

    
   