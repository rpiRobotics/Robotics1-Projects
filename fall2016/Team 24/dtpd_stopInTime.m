ap=duck.april_tags;
ap=duck.april_tags;
ap=duck.april_tags;
distance=ap{1,1}.pos(1);
disp(distance);
if distance > 0.2
    time=distance/0.11;
    disp(time);
    duck.sendCmd(0.1,0);
else
    disp('alread near scene')
end
pause(time);
duck.sendCmd(0,0);
disp('arrive at scene');