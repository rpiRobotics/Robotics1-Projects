%% To P1
a=duck.april_tags;
dd=a{1,1}.pos(1)
aa=a{1,1}.pos(2)
dist=(dd^2+(aa+0.06)^2)^0.5
angle=90-abs(atand(dd/(aa+0.06)))

duck.sendCmd(0.1,-1.45)
pause(angle/43)
duck.sendCmd(0.2,0.24)
pause(dist*10.5)
duck.sendCmd(0.1,2)
pause(angle/18)
duck.sendCmd(0,0)

% to P2
duck.sendCmd(-0.23,2.2)
pause(1.2)
duck.sendCmd(-0.1,-0.5)
pause(0.5)
a=duck.lane_pose; 
while abs(a.phi)<1.4
    a=duck.lane_pose; 
    duck.sendCmd(-0.23,-2.2);
    pause(0.1);
end
duck.sendCmd(0,0)
