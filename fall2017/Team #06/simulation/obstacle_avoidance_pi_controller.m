% Author: Ruixuan Yan
% Simulation for PI controller

clear

z=0.5;
for z = linspace(0.1,2,10)

p=0;
d0=0;
theta0=0;
theta(1) = theta0;
wt=10;

s=0.1;
l=0.3;

theta = [];
m=2;
t_s = 0.05; % 0.3;
k_1 = -.7; % -0.5576;
k_2 = -3; % -1.3274;
 
theta_obj=atan((s+l)/z);
del_theta0=0;
del_theta1=0;
 while abs((theta0-theta_obj))>0.01
    del_theta1=-(theta_obj-theta0);
    wt=k_1*(del_theta1-del_theta0)+ k_2*del_theta1;
    theta0=theta0+wt*t_s;
    theta(m)=theta0;
    dt=-0.5/wt*cos(theta0)+d0+0.5/wt*cos(0);
    d(m)=dt;
    m=m+1;
    del_theta0=del_theta1;
 end

disp_str = ['Distance = ', num2str(z), 'm'];
plot((0:m-2)/10,theta,'DisplayName', disp_str); hold on;
end
%title('Convergence Time for Varying Obstacle Distances');
xlabel('time (seconds)');
ylabel('theta');
hold on;

axis([0, 8, 0 1.4]);

legend('show', 'Location', 'NorthEast');


