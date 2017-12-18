%{
This program sets the steering angle of the car, simulating
a modern car. A graph is generated to give the user
feedback.

Authors: Sam Ansaldo, Sean Thammakhoune, Yinsu Zhang
%}


function duckkeycar(duck) %Graph the steering angle
    h=figure;
    global curang;
    curang = 0;
    set(h,'KeyPressFcn',@duckcar);    
end

function duckcar(hobj,callbackdata,duck)
    v=1;w=2;T=.1;
    global curang;
    
    %set current angle corresponding to key strokes
    %Angle is constrained to [-pi,pi]
    switch callbackdata.Key
        case{'uparrow'}
            duck.sendCmd(v,(curang/3)*w-1);
            pause(T)
            duck.sendCmd(0,0);            
        case{'downarrow'}
            duck.sendCmd(-v, -(curang/3)*w+1);
            pause(T)
            duck.sendCmd(0,0); 
        case{'leftarrow'};
            if curang < 20
                curang = curang + 2
                pause(T*2)
            end
            
        case{'rightarrow'};
            if curang > -20 
                curang = curang - 2
                pause(T*2)
            end
        otherwise
            disp(callbackdata.Key)
    end
    %duck.sendCmd(0.2, curang);
    figure(1);
    cla reset;
    x = 1;                          % X coordinate of arrow start
    y = 2;                          % Y coordinate of arrow start
    theta = pi*curang/40;                   % Angle of arrow, from x-axis
    L = 2;                          % Length of arrow
    xEnd = x+L*cos(theta);          % X coordinate of arrow end
    yEnd = y+L*sin(theta);          % Y coordinate of arrow end
    points = linspace(0, theta);    % 100 points from 0 to theta
    xCurve = x+(L/2).*cos(points);  % X coordinates of curve
    yCurve = y+(L/2).*sin(points);  % Y coordinates of curve
    plot(x+[-L L], [y y], '--k');   % Plot dashed line
    hold on;                        % Add subsequent plots to the current axes
    axis([x+[-L L] y+[-L L]]);      % Set axis limits
    %axis equal;                     % Make tick increments of each axis equal
    arrow([x y], [xEnd yEnd]);      % Plot arrow
    p1 = plot(xCurve, yCurve, '-r','DisplayName','Steering Angle');     % Plot curve
    plot(x, y, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w');  % Plot point
    view([-90 90]);
    xlabel('y');
    ylabel('x ');
    legend([p1])
    %camroll(90)
    %B= imread('C:\Wheel.jpg'); 
    %figure(1);
    %imshow(B);
    %camroll(90*(curang/20))
    
    end
