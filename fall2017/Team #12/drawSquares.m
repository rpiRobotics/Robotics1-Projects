function drawSquares(robot,th,op,M,x,y,d)
% robot, control class
% th, z-rotation of paper
% op, origin point (column vector)
% M, square side lengths
% x, width of paper
% y, length of paper
% d, pen length
  %  movePen(robot,0,0,25,d); % can remove this
    [points,t] = ratio(M,[0 x],[0 y]);
    for i=1:length(points)
        points(1,i)=points(1,i)+20;
        points(2,i)=points(2,i)+5;
    end

    [q]=robot.getJointPositions();
    q1=double(q(1));
    q2=double(q(2));
    q3=double(q(3));
    p=fwdkindobot(q1,q2,q3,0);
    
    op=[p(1)+op(2)+65; p(2)-op(1)-6];
    th=pi+th;
    points=[[cos(th) -sin(th);sin(th) cos(th)]*points]';
    figure
    hold on; axis equal;
%     for i=1:length(points)
%         points(i,2)=-points(i,2);
%     end
    points=points+repmat(op',size(points,1),1);
    
    for(i=1:length(points))
        plot(points(i,1),points(i,2),'r*');
    end
    for i=1:size(M,2)*t
        movePen(robot,points(4*(i-1)+1,1),points(4*(i-1)+1,2),25,d); % pen up at first point
        for j=[0 1 3 2 0]
            movePen(robot,points(4*(i-1)+1+j,1),points(4*(i-1)+1+j,2),0,d); % draw
        end
        movePen(robot,points(4*(i-1)+1,1),points(4*(i-1)+1,2),25,d); % pen up
    end
end