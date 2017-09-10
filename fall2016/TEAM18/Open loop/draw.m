

function draw( path, precision, robot)
%UNTITLED Summary of this function goes here
%   this makes the arm go between set points in path
load('Parameters.mat')
s=length(path);
for s=1:1:s
    start=double(robot.getJointPositions())- double([Acal; 0]);
    R=rotz(phi)*rotz(atan2(path(2,s),path(1,s)))*roty(pi);
    P=path(:,s);
    finish=trossen_ix(R, P);
    finish=[round(finish);1000];
    robot.setJointPositions(int16(finish));
    pause(0.5);
    %step=(finish-start)/precision;
   %for t=0:1:precision
        %start=start+step;
        %robot.setJointPositions(int16([round(start);1000]));
        %pause(0.1);
    end
end

