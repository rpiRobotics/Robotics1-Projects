i = 0;
stop_sign = 0;
size = 0;
lane_change = 0;
stop_stop=0;
while (i~=350)
    y = duck.april_tags;
    x = duck.lane_pose;
    if(isempty(y)~=1)		%check whether we detect tag
        pos = y{1}.pos;
        if (pos(1)< 0.45)	%check distance to tag
            disp(pos);
            if(stop_sign~=1)
                duck.sendCmd(0,0);
                pause(2);
                stop_sign = 1;
            else
                if(x.d>0.15)	%if d>0.15, we are in the left lane
                    lane_change = 1;
                    duck.sendCmd(0,-2);
                    pause(0.5);
                    duck.sendCmd(0,0);
                    pause(4);
                    stop_sign = 0;
                    disp('lane 1 to 2');
                elseif(x.d>0 && x.d<0.15)	%in the right lane
                    lane_change = 2;
                    duck.sendCmd(0,2);
                    pause(0.5);
                    duck.sendCmd(0,0);
                    pause(4);
                    stop_sign = 0;
                    disp('lane 2 to 1');
                end
            end
        else
            if (lane_change==1)	%change the lane
                if(x.d>0)
                    duck.sendCmd(0.1,0);
                    disp('cross lane 1!!!');
                else			%return the orientation back
                    duck.sendCmd(0,2);
                    pause(0.5);
                    lane_change = 0;
                    disp('finish cross lane 1');
                end
            elseif (lane_change==2)		%change the lane
                if(x.d<0.27)&&(stop_stop==0)
                    duck.sendCmd(0.1,0);
                elseif(x.d>=0.27)
                    duck.sendCmd(0.1,0);
                    pause(0.3);
                    stop_stop=1;
                else
                    duck.sendCmd(0,-2);
                    pause(0.5);
                    lane_change = 0;
                end
            else
                duck.sendCmd(0.1,0);
                disp('detect tag');
            end
        end
    elseif (lane_change==1)		%same case when tag is not detected
        if(x.d>0)
            duck.sendCmd(0.1,0);
            disp('cross lane 1!!!');
        else
            duck.sendCmd(0,2);
            pause(0.5);
            lane_change = 0;
            disp('finish cross lane 1');
        end
    elseif (lane_change==2)
        if(x.d<0.27)&&(stop_stop==0)
            duck.sendCmd(0.1,0);
        elseif(x.d>=0.27)
            duck.sendCmd(0.1,0);
            pause(0.3);
            stop_stop=1;
        else
            duck.sendCmd(0,-2);
            pause(0.5);
            lane_change = 0;
        end
    else
        duck.sendCmd(0.1,0);
        disp('do not detect tag');
    end
    pause(0.1);
    i= i+1;
    duck.sendCmd(0,0);
    pause(0.1);
    clear y pos;
end
duck.sendCmd(0,0);
