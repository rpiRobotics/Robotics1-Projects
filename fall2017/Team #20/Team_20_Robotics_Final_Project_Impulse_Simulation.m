%% Robotics Final Project
% Team 20 - Impulse Simulation
% Joseph Grella & Thomas DeMartino

clear all
clc

Impulse_Time = 0.01; % Impulse Time (s)
K = .2;              % Spring Constant (N/m)
E = -18;
B = 13;
C = 10;              % Damping Constant
M = 1;               % System Mass

F = E*.127*B*Impulse_Time; % Force Value (N)

zeta = C/(2*sqrt(K*M));    % Damping Coefficient

Wn = sqrt(K/M);            % Natural Frequency
Wd = Wn*sqrt(1-zeta^2);    % Damped Natural Frequency

i = 1;

for t = 0:0.01:30          % Plotting Loop
    time(i) = t;    
    x(i) = .25-(((F*exp(-zeta*Wn*t)/(M*Wd))*sin(Wd*t + pi))); % Main Function
    i = i+1;
end 

figure(1)                   % Plots of x1(t)
plot(time,x,'k')

title(' Simulation Displacement of X1 - Simulation'); % Plotting Features
xlabel('Time (seconds)');
ylabel('Displacement vs. Time (meters)');
legend('x1(t)');


