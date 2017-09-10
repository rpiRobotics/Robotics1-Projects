clear; close; clc;
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
   
%% move to start
z1 = 5; %writing height

[Q,e] = ikdobot(190, -105, z1);
if strcmp(e,'None')
    robot.setJointPositions(int16(Q(1)),int16(Q(2)),int16(Q(3)),int16(0),int16(60))
    pause(0.1)
end

%% path 1 D round
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain
r=25; %(mm) radius of the circle
xc = 215; %x center of circle
yc = -105; %y center of circle

dt=.1;
t=(0:dt:T);
N=length(t);
q=zeros(2,N);
u=zeros(2,N-1);
L=zeros(1,N);

q(:,1)=[190;-105];
maxunorm=100;
u_planned = zeros(2,N-1);

for k=1:N-1
    qstarl=[r*cos((L(k)-1)*pi)+xc;-r*sin((L(k)-1)*pi)+yc];
    u_planned(:,k)=-k*(q(:,k)-qstarl)*ldot;
    if norm(u_planned(:,k))>maxunorm
        u(:,k)=u_planned(:,k)/norm(u_planned(:,k))*maxunorm;
    else
        u(:,k)=u_planned(:,k);
    end
    normu(k)=norm(u(:,k));
    q(:,k+1)=q(:,k)+ dt * u(:,k);
    L(k+1)=L(k)+ldot*dt;
end

qstar=[r*cos((L-1)*pi)+xc;-r*sin((L-1)*pi)+yc];
q(:,N) = qstar(:,length(qstar));
%figure(1);plot(t,q)
figure(2);plot(q(1,:),q(2,:),'x',qstar(1,:),qstar(2,:),'o');
legend('q','c(\lambda)')
hold on

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    
    for ii = 1:length(Q)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.06)
    end
end
clear Q e;        

%% path 2 D straight
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 240; %x start
ys = -105; %y start
xe = 190; %x end
ye = -105; %y end

plot([xs xe],[ys ye])
q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end
clear Q e;         

%% path 3 to U
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 190; %x start
ys = -105; %y start
xe = 190; %x end
ye = -60; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end
clear Q e; 

%% path 4 U straight left
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 190; %x start
ys = -60; %y start
xe = 215; %x end
ye = -60; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end 
clear Q e; 

%% path 5 U round
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain
r=25; %(mm) radius of the circle
xc = 215; %x center of circle
yc = -35; %y center of circle

dt=.1;
t=(0:dt:T);
N=length(t);
q=zeros(2,N);
u=zeros(2,N-1);
L=zeros(1,N);

q(:,1)=[215;-60];
maxunorm=100;
u_planned = zeros(2,N-1);

for k=1:N-1
    qstarl=[-r*sin((L(k)-1)*pi)+xc;r*cos((L(k)-1)*pi)+yc];
    u_planned(:,k)=-k*(q(:,k)-qstarl)*ldot;
    if norm(u_planned(:,k))>maxunorm
        u(:,k)=u_planned(:,k)/norm(u_planned(:,k))*maxunorm;
    else
        u(:,k)=u_planned(:,k);
    end
    normu(k)=norm(u(:,k));
    q(:,k+1)=q(:,k)+ dt * u(:,k);
    L(k+1)=L(k)+ldot*dt;
end

qstar=[-r*sin((L-1)*pi)+xc;r*cos((L-1)*pi)+yc];
q(:,N) = qstar(:,length(qstar));
%figure(1);plot(t,q)
figure(2);plot(q(1,:),q(2,:),'x',qstar(1,:),qstar(2,:),'o');
legend('q','c(\lambda)')

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
   
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.06)
    end
end
      clear Q e;   

%% path 6 U straight right
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 215; %x start
ys = -10; %y start
xe = 190; %x end
ye = -10; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end
clear Q e; 

%% path 7 to C
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 190; %x start
ys = -10; %y start
xe = 190; %x end
ye = 35; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.06)
    end
end    
clear Q e; 

%% path 8 C
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain
r=25; %(mm) radius of the circle
xc = 215; %x center of circle
yc = 35; %y center of circle

dt=.1;
t=(0:dt:T);
N=length(t);
q=zeros(2,N);
u=zeros(2,N-1);
L=zeros(1,N);

q(:,1)=[190;35];
maxunorm=100;
u_planned = zeros(2,N-1);

for k=1:N-1
    qstarl=[r*cos((L(k)-1)*pi)+xc;r*sin((L(k)-1)*pi)+yc];
    u_planned(:,k)=-k*(q(:,k)-qstarl)*ldot;
    if norm(u_planned(:,k))>maxunorm
        u(:,k)=u_planned(:,k)/norm(u_planned(:,k))*maxunorm;
    else
        u(:,k)=u_planned(:,k);
    end
    normu(k)=norm(u(:,k));
    q(:,k+1)=q(:,k)+ dt * u(:,k);
    L(k+1)=L(k)+ldot*dt;
end

qstar=[r*cos((L-1)*pi)+xc;r*sin((L-1)*pi)+yc];
q(:,N) = qstar(:,length(qstar));
%figure(1);plot(t,q)
figure(2);plot(q(1,:),q(2,:),'x',qstar(1,:),qstar(2,:),'o');
legend('q','c(\lambda)')

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
  
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end
    clear Q e;     

%% path 9 to K
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 240; %x start
ys = 35; %y start
xe = 240; %x end
ye = 80; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end   
clear Q e; 

%% path 10 K left
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 240; %x start
ys = 80; %y start
xe = 190; %x end
ye = 80; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end   
clear Q e; 

%% path 11 to next K
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 190; %x start
ys = 80; %y start
xe = 190; %x end
ye = 110; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end        
clear Q e; 

%% path 12 K top
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 190; %x start
ys = 110; %y start
xe = 215; %x end
ye = 80; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end     
clear Q e; 

%% path 13 K bottom
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain

xs = 215; %x start
ys = 80; %y start
xe = 240; %x end
ye = 110; %y end

plot([xs xe],[ys ye])

q = [xs xe;ys ye];

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),z1);
    if ~strcmp(e,'None')
        er = 1;
        num = [num ii];
    end
end
if er == 1
    disp('Error in')
    disp(num)
    break
else
    for ii = 1:size(Q,3)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.5)
    end
end     


daspect([1 1 1])