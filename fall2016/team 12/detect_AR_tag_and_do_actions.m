i = 0;
stop_sign = 0;
size = 0;
while (i~=200)
    y = duck.april_tags;
    x = duck.lane_pose;
    if(isempty(y)~=1)		%check whether we detect tag
        pos = y{1}.pos;
        if (pos(1)< 0.45)	%check if the distance is less than 0.45
            disp(pos);
            if(stop_sign~=1)	%if see stop sign
                duck.sendCmd(0,0);
                pause(2);
                stop_sign = 1;
            else
                if(y{1}.id==7)	%if see turn left sign
                    duck.sendCmd(0,4.8);
                    pause(0.5);
                    duck.sendCmd(0,0);
                    pause(4);
                    stop_sign = 0;
                elseif (y{1}.id==6)		%if see turn right sign
                    duck.sendCmd(0,-4.8);
                    pause(0.5);
                    duck.sendCmd(0,0);
                    pause(4);
                    stop_sign = 0;
                end
            end
        else
            duck.sendCmd(0.1,0);
            disp('detect tag');
        end
    else
        duck.sendCmd(0.1,0);
        disp('do not detect tag');
    end
    pause(0.1);
    i= i+1;
    duck.sendCmd(0,0);
    pause(0.05);
    clear y pos;
end
duck.sendCmd(0,0);
