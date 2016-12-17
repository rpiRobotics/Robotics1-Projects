speed_gain = 0.2;	%our forward velocity gain
i = 0;			%loop number
while (i~=500)
    x = duck.lane_pose;	%lane pose information
    d_error = x.d;
    head_error = x.phi;
    v = speed_gain;
    if (d_error>0.2)		%for different d, different gains
        w = 1.2*d_error-4*head_error;
    else
        w = 1.2*d_error-2.5*head_error;
    end
    duck.sendCmd(v,w);
    pause(0.1);
    i= i+1;
end
duck.sendCmd(0,0);	%stop command
