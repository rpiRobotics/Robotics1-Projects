if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://192.168.2.4:1234/DuckiebotServer.dtpd/Duckiebot');
    camera_on=0;
else
    if camera_on==0;duck.openCamera();camera_on=1;end
    duckkeydrive(duck);
end

