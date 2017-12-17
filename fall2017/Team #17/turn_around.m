function turn_around( duck )
    duck.sendCmd(0.3, 4);
    pause(2.225);
    duck.sendCmd(0.3,-2);
    pause(0.2);
    duck.sendCmd(0,0);
end

