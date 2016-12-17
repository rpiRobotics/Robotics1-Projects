while true    
    ap=duck.april_tags;
    distance=ap{1,1}.pos(1);
    disp(distance);
    if distance > 0.2
        time=distance/0.11;
        disp(time);
        duck.sendCmd(0.1,0);
        pause(time-0.1);
        duck.sendCmd(0,0);
        disp('near car');
    else
        disp(' near car')
    end
    disp('ready to swerve');
    duck.sendCmd(0,5);
    pause(0.5);
    duck.sendCmd(0.4,0);
    pause(0.8);
    duck.sendCmd(0,-5);
    pause(0.5);
    duck.sendCmd(0.1,0)
    pause(2);
    duck.sendCmd(0,0);

    
    
end