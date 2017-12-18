w = 0;
i = 0;
while i <= 100000
    old_pose = duck.lane_pose;
    duck.sendCmd(0.4,w);
    pause(0.5);
    new_pose = duck.lane_pose;
    dphi = new_pose.phi - old_pose.phi;
    if abs(dphi) < 0.05
        disp w;
        disp(w);
        break;
    end
    i = i + 1;
    w = w + 0.05;
end