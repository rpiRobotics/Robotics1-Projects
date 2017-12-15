clc
clear

% Call gradmaker() to generate points to draw
gradpts=gradmaker('snoopy.jpg',9);

figure
x=gradpts(:,1);
y=gradpts(:,2);
% Scale gradient points to fit in Dobot task space
% Show plots of final picture to be drawn
for i=1:size(gradpts,1)
    cd=[gradpts(i,1) gradpts(i,2)];
    gradpts(i,1)=round(((cd(1)-min(x))*(270-190))/(max(x)-min(x))+190);
    gradpts(i,2)=round(((cd(2)-min(y))*(40-(-40)))/(max(y)-min(y))+(-40));
end
for i=1:size(gradpts,1)
    plot(gradpts(i,1), gradpts(i,2), '*')
    hold on
end

% Connect Dobot and begin drawing points
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
p=0.5;
p2=0.2;
for n=1:size(gradpts,1)
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(gradpts(n,1)),double(gradpts(n,2)),20);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p)
   
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(gradpts(n,1)),double(gradpts(n,2)),10);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p2)
   
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(gradpts(n,1)),double(gradpts(n,2)),20);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p2)
   
   [Q(:,:,1),~] = ikdobot(230,0,20); %start position
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   pause(p);
end
disp('Enjoy your scribble')
Q=zeros(1,3,16);
[Q(:,:,1),~]=ikdobot(270,100,30)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(10)
Q=zeros(1,3,16);
[Q(:,:,1),~]=ikdobot(230,0,30)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
robot.getJointPositions()
disp('At start')
















% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% x=[LC(:,1);LC(:,3);gradpts(:,1)];
% y=[LC(:,2);LC(:,4);gradpts(:,2)];
% 
% for i=1:size(LC,1)
%     cd1=[LC(i,1) LC(i,2)];
%     cd2=[LC(i,3) LC(i,4)];
%     LC(i,1)=round(((cd1(1)-min(x))*(270-190))/(max(x)-min(x))+190);
%     LC(i,2)=round(((cd2(2)-min(y))*(40-(-40)))/(max(y)-min(y))+(-40));
%     LC(i,3)=round(((cd1(1)-min(x))*(270-190))/(max(x)-min(x))+190);
%     LC(i,4)=round(((cd2(2)-min(y))*(40-(-40)))/(max(y)-min(y))+(-40));
% end
% 
% for i=1:size(gradpts,1)
%     cd=[gradpts(i,1) gradpts(i,2)];
%     gradpts(i,1)=round(((cd(1)-min(x))*(270-190))/(max(x)-min(x))+190);
%     gradpts(i,2)=round(((cd(2)-min(y))*(40-(-40)))/(max(y)-min(y))+(-40));
% end
% 
% figure
% for i=1:1:size(LC,1)
%     plot([LC(i,1) LC(i,3)],[LC(i,2) LC(i,4)],'LineWidth',3,'Color','blue');
%     hold on 
% end
% 
% for i=1:size(gradpts,1)
%     plot(gradpts(i,1), gradpts(i,2), '*')
%     hold on
% end






