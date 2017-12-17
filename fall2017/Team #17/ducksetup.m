if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.13.215.114:1234/DuckiebotServer.jrcjpmbot/Duckiebot');
    camera_on=0;
else
    if camera_on==0;duck.openCamera();camera_on=1;end
    duckkeydrive(duck);
end

