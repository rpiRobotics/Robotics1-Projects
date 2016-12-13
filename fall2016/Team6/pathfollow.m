clear; close; clc;
ldot=.2; % lambda dot
T=1/ldot; % T=length of time to travel the path
k=100; % feedback gain
r=50; %(mm) radius of the circle
xc = 230; %x center of circle
yc = 0; %y center of circle

dt=.1;
t=(0:dt:T);
N=length(t);
q=zeros(2,N);
u=zeros(2,N-1);
L=zeros(1,N);

q(:,1)=[200;0];
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
figure(1);plot(t,q)
figure(2);plot(q(1,:),q(2,:),'x',qstar(1,:),qstar(2,:),'o');
legend('q','c(\lambda)')

Q = zeros(1,3,length(q));
num = [];
er = 0;
for ii=1:length(q)
   [Q(:,:,ii),e] = ikdobot(q(1,ii),q(2,ii),30);
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
    robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
    robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(60))
    pause(0.1)
    robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(60))
    uiwait(msgbox({'Ready to go?'}));
    for ii = 2:length(Q)
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(0),int16(60))
        pause(0.06)
    end
end
        