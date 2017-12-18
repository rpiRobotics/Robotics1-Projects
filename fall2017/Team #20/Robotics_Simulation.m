%% Robotics Final Project
% Team 20
% Joseph Grella & Thomas DeMartino
% This is the ODE Solver to compute the standard response

function [xdot] = Robotics_Simulation(t,x)

global K F C M W

% State Declaration

x1 = x(1); 
x2 = x(2);

% Solver Function

xdot(1,1) = x2;
xdot(2,1) = ((F/M)*sin(W*t)) - ((C/M*(x2))) - ((K/M)*x1);

end 
