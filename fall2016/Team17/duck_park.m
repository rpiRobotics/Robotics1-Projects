%% duck_park.m

% Authors: Devon Baughen
%          Nicola Mesiti
% Date:    11/26/16
% Description:
%   This code drives the robot around a track while keeping it in 
%   the lane using closed loop control based on duck_move2.m, while 
%   searching for an AR tag. When the bot reaches a certain 
%   distance from the AR tag it then performs a parallel park.

% connect to the duckiebot
if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.13.215.147:1234/DuckiebotServer.faduck/Duckiebot');
    camera_on=0;
end

% constants:
k1 = 6;
k2 = 8;
k3 = 1;
vmax = .3;
ps=0;

%initial values:
w = 0;
%v = .05;
xy_vec = [];
d_vec = [];
phi_vec = [];
ar_d = 5;
ar_vec = [];

for i=1:27
    % get ar tag information
    ar=duck.april_tags;
    
    % if an AR tag is detected...
    if length(ar)>0
        % record distance from the tag
        ar_d = ar{1,1}.pos(1);
        ps=1;
        
        % if within .3 units from the tag parallel park the bot
        if ar_d<0.3
            pause(2);
            parallel_park(duck);
            break
        end
    end

    % record distances measured from AR Tag
    ar_vec=[ar_vec ar_d];
        
    % get 'phi' and 'd' using averaging function
    position = pos(duck);
    dist = position(1);
    phi = position(2);
    
    % get xy position and add to vector of positions
    xy_cur = xypos(duck);
    xy_vec = [xy_vec xy_cur];
    
    % record d and phi
    d_vec = [d_vec dist];
    phi_vec = [phi_vec phi];
    
    % set angular velocity
    w = 0 - (k1 * phi) - (k2 * dist);
    % set forward velocity
    %v = vmax - (k3 * phi);
    
    % if tag detected send new 'v' and 'w' to the bot
    if ps==1
        v=.1;
        duck.sendCmd(v,w);
        pause(.3);
    
        % stop the bot
        duck.sendCmd(0,0);
        pause(.5);
    % if no AR tag detected send following velocity to the bot
    else
        v=.3;
        duck.sendCmd(v,w);
        pause(.4);
    
        % stop the bot
        duck.sendCmd(0,0);
        pause(.5);    
    end
end

%stop the bot
duck.sendCmd(0,0);

