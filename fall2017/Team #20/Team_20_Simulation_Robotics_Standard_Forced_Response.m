%% Robotics Final Project
% Team 20
% Joseph Grella & Thomas DeMartino
%%
% This Code Simulates Motion of a Spring Mass Damper

clear all
clc

global K F C M W

K = 20;                % Spring Constant (N/m)
F = 18*.127;           % Force Value (N)
C = 1;                 % Damping Constant
M = 1;                 % System Mass
W = 0.0001; 

tspan = [0:0.01:10];                           % Solver Time Span
y0 = [0.25 0];                                 % Initial Conditions
[Z,X] = ode45(@Robotics_Simulation,tspan,y0);  % Solve ODE

figure(1)                                      % Plot of x1
plot(Z,X(:,1),'k')

G = X(:,1)+.25;                                % Phase Shift
Tim = tspan-.32;

plot(Tim(1,(33:end)),G((33:end),1));           % Plot

title('ODE SOLVER - Displacement of X1');      % Plotting Features
xlabel('Time (seconds)');
ylabel('Displacement vs. Time (meters)');
legend('x1(t)');


