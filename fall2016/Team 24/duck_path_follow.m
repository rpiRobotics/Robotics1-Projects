ldot=.1; % lambda dot
T=1/ldot; % T=length of time to travel the path
dt=.1; %time increment
t=(0:dt:T); %all time
N=length(t); %number of time steps
q=zeros(2,N); %configuration history
u=zeros(2,N-1); %control history
l=zeros(1,N); %lambda values
full_path = construct_path([2;0], [2;10], [4;12], [18;12], dt, ldot);
%full_path2 = construct_path([2;0], [2;40], [4;32], [22;32], dt, ldot);
%full_path2 = construct_path([18;12], [30;12], [32;10], [32;0], dt, ldot);
full_path2 = full_path;
full_path2(1,:)=36-full_path(1,:);
% full_path=[full_path full_path2];
% full_path=full_path(:,1:500);
x=full_path(1,1);
y=full_path(2,1);
theta=0;
ww=[];
tt=[];
for i=2:1:length(full_path)
    old_x=x;
    old_y=y;
    old_theta=theta;
    x=full_path(1,i);
    y=full_path(2,i);
    dx=x-old_x;
    dy=y-old_y;
    if dx==0 || dy==0
        theta=0;
    else
        theta=atan2(dy,dx);
        disp(theta);
    end
    dtheta=theta-old_theta;
    w=15/2/pi*10*dtheta*9.5;
    v=0.6;
    duck.sendCmd(v,-1.1*w+0.2);
    disp('w');
    disp(w);
    ww=[ww w];
    tt=[tt theta];
end

full_path=full_path2;
x=full_path(1,1);
y=full_path(2,1);
theta=0;
ww=[];
tt=[];
for i=2:1:length(full_path)
    old_x=x;
    old_y=y;
    old_theta=theta;
    x=full_path(1,i);
    y=full_path(2,i);
    dx=x-old_x;
    dy=y-old_y;
    if dx==0 || dy==0
        theta=0;
    else
        theta=atan2(dy,dx);
        disp(theta);
    end
    dtheta=theta-old_theta;
    w=15/2/pi*10*dtheta*9.5;
    v=0.6;
    duck.sendCmd(0.8*v,0.95*w);
    disp('w');
    disp(w);
    ww=[ww w];
    tt=[tt theta];
end
duck.sendCmd(0,0);