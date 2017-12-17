function turn_right( duck )
    duck.sendCmd(0.5, -4);
    pause(1.5);
    duck.sendCmd(0.5,1);
    pause(0.2);
    duck.sendCmd(0,0);
end

