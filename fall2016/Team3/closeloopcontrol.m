clc
clear

duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');

i = 0;
k_d = -0.8;
k_theta = -2.1;
tic
while (i < 500)
    lanepos = duck.lane_pose;
    
    w = k_d*lanepos.d + k_theta*lanepos.phi;
    duck.sendCmd(0.2,w);
    ar=duck.april_tags;          
        
    end
    i = i+1; 
end
duck.sendCmd(0.0,0.0);


  % d = 0 at yellow lane, 
    %   
    % phi increase when heading to left, 
    %     decrease when heading to right.
    
     %w > 0, turn left
    %w < 0, turn right 
     