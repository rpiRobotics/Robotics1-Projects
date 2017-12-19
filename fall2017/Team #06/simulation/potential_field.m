% Author: Ruixuan Yan
% Simulate Potential Field

clear;
X0=[0;0];

% step gain
%k=1.20098;
k = 1.20098;

% gain to avoid obstacle
%p=3.259876;
p = 3.259;

% boundary when repulsive force kicks in
p0=2;

% radius of obstacle
r=0.8;
n=1;
J=200;

% car velocity
v = 1.2;
t = 1;
w = zeros(J,1);

end_margin = 0.3;

%Xsum=[5.245,9.876;2,2.675];
% Describes target location and obstacle
Xsum=[5.245,9.876;2,2.675];
Xj=X0;

for j=1:J
    Goal(j,1)=Xj(1);
    Goal(j,2)=Xj(2);
    
    % calcualate attractive force
    Fatt_x=k*(Xsum(1,1)-Xj(1));
    Fatt_y=k*(Xsum(1,2)-Xj(2));
    
    % calcualte repulsive distance
    Pq=sqrt(abs((Xj(1)-Xsum(2,1))^2+(Xj(2)-Xsum(2,2))^2-r^2));
    
    if Pq>p0
        Fre_x=0;
        Fre_y=0;
    else
        Fre_x=p*(1/Pq-1/p0)*(1/Pq^2)*(Xj(1)-Xsum(2,1))/Pq;
        Fre_y=p*(1/Pq-1/p0)*(1/Pq^2)*(Xj(2)-Xsum(2,2))/Pq;
    end
    
    % calculate net force
    Fsum_x=Fatt_x+Fre_x;
    Fsum_y=Fatt_y+Fre_y;
    
    Position_angle(j)=atan(Fsum_y/Fsum_x);
    Xnext(1)=Xj(1)+ v*cos(Position_angle(j));
    Xnext(2)=Xj(2)+ v*sin(Position_angle(j));
    
    Xj=Xnext;
    
    % if we're at the target
    if (abs(Xj(1)-Xsum(1,1))<end_margin)&&(abs(Xj(2)-Xsum(1,2))<end_margin)
        Kfin=j;
        Goal(j,1)=Xj(1);
        Goal(j,2)=Xj(2);
        break;
    end
    
    % else reassign x
    Goal(j,1)=Xj(1);
    Goal(j,2)=Xj(2);
end

figure;
hold on;
theta=-2*pi:pi/10:2*pi;
cir_x=Xsum(2,1)+r*cos(theta);
cir_y=Xsum(2,2)+r*sin(theta);
plot(cir_x,cir_y, 'DisplayName', 'Obstacle');
plot(Xsum(1,1),Xsum(1,2),'*', 'DisplayName', 'Goal');
axis([0 12 0 12]);
size_Goal=size(Goal);
sizer_goal=size_Goal(1);

F(sizer_goal*10) = struct('cdata',[],'colormap',[]);

v = VideoWriter('potential_field.avi');
open(v);
xlabel('x'); ylabel('y'); legend('show')

for jk=1:sizer_goal
h(jk)=plot(Goal(jk,1),Goal(jk,2),'o-');
pause(0.2);
drawnow
F(jk) = getframe;
frame = getframe(gcf);
writeVideo(v,frame);
end

close(v);
