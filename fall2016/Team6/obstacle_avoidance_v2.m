clc; clear; close all;

%% define walls for plotting purposes

wallleftx = 0:1:210; walllefty = 160*ones(1, 211);
walltopy = 0:1:160; walltopx = 210*ones(1, 161);

%% potential field parameters
N=1000;

alpha=.02; % time step
q0=[230;0]; % starting position
z = 25;
%robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
[J,~] = ikdobot(q0(1),q0(2),z);
%robot.setJointPositions(int16(J(1)),int16(J(2)),int16(J(3)),int16(0),int16(60))
uiwait(msgbox({'Ready to go?'}));
%robot.setJointPositions(int16(J(1)),int16(J(2)),int16(J(3)),int16(0),int16(60))
%pause(1)

k=1; % attractive force gain
eta=1500; % repulsive force gain
qf=[0;180]; % end position

qq=zeros(2,N+1); 
qq(:,1)=q0;
wall_buffer = 10; % buffer to exert force on robot

for i=1:N
    q_current_x = qq(1,i);
    q_current_y = qq(2,i);
    
    % compute whether or not the robot is within a wall's zone of exerting
    % force. These values will be 1 or 0
    inWallTop = (0<q_current_y) &&  (q_current_y<165) && (230-wall_buffer>q_current_x) && (q_current_x>210);
      
    dist_wallTop = abs(210-q_current_x);
      
    qq(:,i+1) =qq(:,i)+ alpha*( -k * (qq(:,i)-qf) - ...
    (eta*(((1/dist_wallTop) - (1/wall_buffer))*[-1;0]))*(inWallTop));
     
end


%% plot everything
figure; hold on;
plot(wallleftx,walllefty,'k');plot(walltopx,walltopy,'k');
plot(wallleftx,walllefty+wall_buffer,'-m');plot(walltopx+wall_buffer,walltopy,'-m');
plot(qq(1,:),qq(2,:),'x-')
% show a few of the robot movements
viscircles(qq(:,1)',1); viscircles(qq(:,5)',1);
viscircles(qq(:,6)',1); viscircles(qq(:,7)',1);
daspect([1 1 1])

QQ = zeros(2,201);
QQ(:,1) = q0;
for ii = 1:200
    QQ(:,1+ii) = qq(:,5*ii);
end
plot(QQ(1,:),QQ(2,:),'rx')

qkeep = QQ(:,1:50);

for ii = 1:50
    [J,~] = ikdobot(QQ(1,ii),QQ(2,ii),z);
   % robot.setJointPositions(int16(J(1)),int16(J(2)),int16(J(3)),int16(0),int16(60))
   % pause(0.5)
end
