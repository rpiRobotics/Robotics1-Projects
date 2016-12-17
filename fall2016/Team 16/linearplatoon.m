%platooning is currently implemented for straight lines 
%only due to issues with odometry. This platooning
%alogrithm can be thought of as a PID controller
%with respect to velocity. Only PI is used in this code. 

if ~exist('duck1')
    disp('connecting duck 1');
    duck1=RobotRaconteur.Connect('tcp://10.13.215.101:1234/DuckiebotServer.motherduck/Duckiebot');
    camera_on=0;
end

if ~exist('duck2')
    disp('connecting duck 2');
    duck2=RobotRaconteur.Connect('tcp://10.13.215.113:1234/DuckiebotServer.duckiebot1/Duckiebot');
    camera_on=0;
end

%duck1 executes basic path, duck2 will follow

K1 = .5; %position/integral gain
K2 = .5; %velocity/proportional gain
d = 0; %desired seperation distance
%note, if no external odometry is used, set d to 0 and manually 
%place ducks in desired distance positions


tpos = [0;0]; % record positions for graph output
tvel = [0;0]; % record velocities for graph output

xcor1 = duck1.x; % x correction for starting positions
xcor2 = duck2.x;


for i = 1:30
    p1 = duck1.x - xcor1;
    p2 = duck2.x - xcor2;
    %add more duck positions add needed

    %duck1 basic path - leading duck
    v1 = 0;
    if (i <= 20)
        v1 = .5;
        duck1.sendCmd(v1,0);
    end

    v2 = K1*(p1-p2 - d)+K2*v1; %Platooning velocity control
    
    duck2.sendCmd(1.3*v2,0);

    tpos = [tpos, [p1;p2]];
    tvel = [tvel, [v1;v2]];
    
    
    pause(.1) %Currently updating at 10 Hz
end

duck1.sendStop();
duck2.sendStop();

t = 0:.1:3; % plot output data
figure(1);
plot(t,tpos(1,:), t, tpos(2,:));
figure(2);
plot(t, tvel(1,:), t, tvel(2,:));

