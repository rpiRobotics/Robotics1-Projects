%% duck_move2.m

% Authors:  Devon Baughen
%           Nicola Mesiti
% Date:     11/23/16
% Descrition:
%   This code drives the duckiebot arount a track using proportional
%   control feed back based on phi and d in order to keep the robot
%   within the lane.

% connect to the duckiebot
if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.13.215.147:1234/DuckiebotServer.faduck/Duckiebot');
    camera_on=0;
end

% open the camera
if camera_on==0
    duck.openCamera();
    camera_on=1;
end

% constants:
k1 = 5;     % phi gain
k2 = 2;     % d gain
k3 = 1;     % phi gain for velocity control 
vmax = .3;  % maximum velocity

%initial values:
w = 0;
v = .25;
xy_vec = [];
d_vec = [];
phi_vec = [];

for i=1:20
    % get 'phi' and 'd' using averaging function
    position = pos(duck);
    dist = position(1);
    phi = position(2);
    
    % get xy position and add to vector of positions
    %xy_cur = xypos(duck);
    %xy_vec = [xy_vec xy_cur];
    
    % record d and phi
    d_vec = [d_vec dist];
    phi_vec = [phi_vec phi];
    
    % set angular velocity
    w = 0 - (k1 * phi) - (k2 * dist);
    
    % set forward velocity
    %v = vmax - (k3 * phi);
    
    % send new 'v' and 'w' to the bot
    duck.sendCmd(v,w);
    pause(.1);
    
    % stop the bot
    %duck.sendCmd(0,0);
    %pause(1);

end

%stop the bot
duck.sendCmd(0,0);

