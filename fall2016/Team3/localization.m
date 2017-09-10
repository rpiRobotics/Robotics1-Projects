clc
clear
%duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');

hold on;
scatter(40,30,'b');
%map();
%sensor();

ar=duck.april_tags;

oc = [22.5*3/4;4];
i=0;
k_d = -0.8;
k_theta = -2.1;
tic
while (i < 1200)
    lanepos = duck.lane_pose;
    
    %d increase when it's close to yellow lane, 
    %  decrease when it's close to white lane.
    %phi increase when heading to left, 
    %    decrease when heading to right.
   
    w = k_d*lanepos.d + k_theta*lanepos.phi;
    
    duck.sendCmd(0.04,w); 
    % v = 0.1 is 4inch/s
    % w = 1 is 32.7 d/s           
    % w > 0, turn left
    % w < 0, turn right 
                     
    % see if there is a ar tag
    ar=duck.april_tags;
    if isempty(ar) == 0
        % which ar tag
        if ar{1,1}.id==7
            oa = [22.5/4*3; 22.5*2];
            ac_a = ar{1,1}.pos(1:2,:)*39.37; %meter to inch for x y
            %ac_a = [temp(2,:);temp(1,:)];
            Roa = [cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)]; %rotation matrix between ar7 and origin
            ac_o = Roa * ac_a; 
            oc = oa + ac_o;
            scatter(oc(1),oc(2),'b');
            disp(ar{1,1}.pos)
        
        elseif ar{1,1}.id==8
            oa = [22.5*2; 22.5*2+2];
            ac_a = ar{1,1}.pos(1:2,:)*39.37;
            %ac_a = [temp(2,:);temp(1,:)];
            Roa = [cos(pi),-sin(pi);sin(pi),cos(pi)];
            ac_o = Roa * ac_a;
            oc = oa + ac_o;
            scatter(oc(1),oc(2),'b');
        elseif ar{1,1}.id==9
            oa = [22.5/2*3; 22.5*4];
            ac_a = ar{1,1}.pos(1:2,:)*39.37;
            %ac_a = [temp(2,:);temp(1,:)];
            Roa = [cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)];
            ac_o = Roa * ac_a;
            oc = oa + ac_o;         
            scatter(oc(1),oc(2),'b');
        end
        temp_t = toc;            
    else
       % measure time
       diff_t = toc - pre_time;
       
       %cacluate angle based on phi & w
       lanepos = duck.lane_pose;     
       angle = 60*lanepos.phi+90 + w*32.7*diff_t;
       rad = pi*angle/180;
       
       %calculate estimate movement
       x = cos(rad)*0.04*40*diff_t;
       y = sin(rad)*0.04*40*diff_t;
       oc = oc +[x;y];
       
       %plot next position
       scatter(oc(1),oc(2),'gx');
       
       pre_time = toc;   
    end
    
 
    i=i+1;
end

%duck.sendCmd(0.0,0.0);