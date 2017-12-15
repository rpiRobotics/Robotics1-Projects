%% Code for running dobot with PRM toolbox and custom PRM algorithm (dobprm(),cprm() resp.)
clc
clear
%% Using the PRM toolbox, opens figure for user to manually click start/end points 
cond=0;
while cond==0
    
    coords=dobprm('t1.jpg');
    figure
    for i=1:5:length(coords)
        plot(coords(i,1),coords(i,2),'*')
        hold on
        pause(0.1)
    end
    x=input('Input [y] to continue, anything else to retry  ','s')
    if x=='y'
        cond=1;
    else
        cond=0;
    end
end

%% Using custom PRM code, just calling cprm() to generate coordinates
coords=cprm('t1.jpg')

%% Enable connection with Dobot
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

%% Run calibration to ensure Dobot can hit corners of task space
calib=[180 280 280 180 220; 100 100 -100 -100 0];
length(calib);
pause(1);
for i=1:1:length(calib)
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(calib(1,i),calib(2,i),20)
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(1)
end
disp('Calibration completed')
pause(2)

%% Begin drawing image
disp('Begining drawing')
p=0.3;
for n=1:4:length(coords)
   if n==1
       Q=zeros(1,3,16);
       [Q(:,:,1),~]=ikdobot(coords(n,1),coords(n,2),20)
       robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
       robot.getJointPositions()
       pause(p)
   end
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(coords(n,1),coords(n,2),0)
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p)
end

disp('Enjoy your scribble')
pause(1)
Q=zeros(1,3,16);
[Q(:,:,1),~]=ikdobot(220,0,20)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
robot.getJointPositions()
disp('At start')
