function [] = movetoorigin( robot, dist_x, dist_y , d )
%movetoorigin Moves Dobot end effector
%to draw origin
%dist_x - x distance of origin to end effector
%dist_y - y distance of origin to end effector
    [q]=robot.getJointPositions();
    q1=double(q(1));
    q2=double(q(2));
    q3=double(q(3));
    p=fwdkindobot(q1,q2,q3,0)
    p=p-[dist_x;dist_y;0];
    [Q,~]=invkinPEN(p,d);
    gotoAngles(robot,[Q(1),Q(2),Q(3),0]);
end

