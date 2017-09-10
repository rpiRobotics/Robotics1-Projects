% Team 24
% Project: DTPD
% Team members: Dingyuan Zhang, Zhengjie Zhang
% This is a script contains a lane following code with chasing and
% collision avoidance functions

% Control flags
suspect = 1; % set to 0 if no suspect to chase, otherwise set to target AR tag ID
camp = 1;   % set to 0 for patrol mode (perform lane following until identifies suspect)
            % set to anything else for camping mode (wait until identifies suspect)

% Start RobotRaconteur
duck=RobotRaconteur.Connect('tcp://192.168.2.4:1234/DuckiebotServer.dtpd/Duckiebot');
camera_on=0;

% Start feedback control loop
i = 0;
while (i<400)
    % Get lane pose information form Duckiebot
    lp = duck.lane_pose;
    d = lp.d;
    phi = lp.phi;
    
    % set base linear speed gain
    if camp == 0
        v = 0.1;
        d_gain = -3;
        phi_gain = -2.5;
    else
        v = 0;
        d_gain = 0;
        phi_gain = 0;
    end
        
    % identify all AR tags
    ARs = duck.april_tags;
    AR_size = size(ARs);
    i_AR = 1;
    too_close = 0;
    
    while i_AR <= AR_size(1)
        AR = ARs{i_AR,1};
        %Start chasing if spotted suspect in camping mode
        if AR.id == suspect && camp ~=0
            v = 0.1;
            d_gain = -3;
            phi_gain = -2.5;
        end

        %if too close to AR tags
        if AR.pos(1)<0.4
            too_close = too_close + 1;
            %stop if closing to suspect
            if AR.id == suspect
                v = 0;
                d_gain = 0;
                phi_gain = 0;
                too_close = 0;
                break;
            end
        end

        i_AR = i_AR + 1;
    end
    
    %Perform collision Avoidance if civilian cars ahead
    if too_close~=0
        duck.sendCmd(0,5);
        pause(0.5);
        duck.sendCmd(0.4,0);
        pause(0.8);
        duck.sendCmd(0,-4.5);
        pause(0.5);
        duck.sendCmd(0.1,0)
        pause(2);
        duck.sendCmd(0,0);
    end
    
    
    % adjust gains using lane position 
    if d<=0
        v = 0.6*v;
    elseif d>0.15
        v = 0.8*v;
    else
        d_gain = d_gain * 2 / 3;
    end
    
    if abs(phi)>0.5
        v = 0.6*v;
    else
        phi_gain = phi_gain * 0.8;
    end
    
    w = d_gain*d + phi_gain*phi;
    
    % Send command to Duckiebot
    duck.sendCmd(v,w);
    i= i+1;
end

% Stop Duckiebot
duck.sendCmd(0,0);