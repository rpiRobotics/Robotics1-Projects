

function drawing(P)
%UNTITLED Summary of this function goes here
%   makes the arm move to position P
global robot;
load('Parameters.mat')
    %start=double(robot.getJointPositions())- double([Acal; 0]);
    R=rotz(phi)*rotz(atan2(P(2),P(1)))*roty(pi);
    finish=trossen_ix(R, P);
    finish=[round(finish);1000];
    robot.setJointPositions(int16(finish));
end

