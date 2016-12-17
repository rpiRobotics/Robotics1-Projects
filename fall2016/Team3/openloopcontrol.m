clc
clear

duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');

duck.sendCmd(0.4,0);%forward
pause(2);
duck.sendCmd(0,-8.3);%turn right
pause(0.3);
duck.sendCmd(0.5,0);%forward
pause(2.8);
duck.sendCmd(0,-8);%turn right
pause(0.3);
duck.sendCmd(0.4,0);%forward
pause(2.5);
duck.sendCmd(0,8.8);%turn left
pause(0.3);
duck.sendCmd(0.4,0);%forward
pause(2);
duck.sendCmd(0.0,0.0);%stop