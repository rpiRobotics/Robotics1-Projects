%% drive_st.m

% Author:   Devon Baughen 
% Date:     11/21/16
%
% Description:
% This function sends commands to the robot to drive it in a straight 
% line for a given amount of time, t, and at a velocity, v. The inputs to
% the function are the robot name, the length of time to drive the bot, and
% the velocity at which to drive the bot.

function drive_st(duck,t,v)
    % drive the robot straight with velocity v
    duck.sendCmd(v,0);
    % wait for time t
    pause(t);
    % stop the robot
    duck.sendCmd(0,0);
end
