function turn_left( duck )
    duck.sendCmd(0.5, 2);
    pause(2.25)
    duck.sendCmd(0.7,-2);
    pause(0.2);
    duck.sendCmd(0,0);
end

