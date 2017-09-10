robot = RobotRaconteur.Connect('tcp://localhost:10001/phantomXRR/phantomXController');

while 1
    theta_reading=double(robot.getJointPositions())
    offset=double([525;506;823;516;506]); % joint readings in counts in zero position
    theta=(theta_reading-offset)*(1/1023)*(5.24) %convert joint readings to radians
    pause(0.2);
end
