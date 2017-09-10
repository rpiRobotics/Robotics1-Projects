while true
    ap=duck.april_tags;
    if ap{1,1}.pos(1)<0.3
        duck.sendCmd(-0.1,0);
    else
        duck.sendCmd(0.1,0);
    end
    pause(1);
    duck.sendCmd(0,0);
    pause(3);
    disp(ap{1,1}.pos(1));    
end