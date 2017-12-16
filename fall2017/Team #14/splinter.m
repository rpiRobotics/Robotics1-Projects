%% Function for creating splines when needed
function sp=splinter(n1,n2,I)
    sp=[];
    m=(n2(2)-n1(2))/(n2(1)-n1(1));
    mpx=(n2(1)+n1(1))/2;
    % Check position of the initial point relative to the target
    % Create a new x coordinate for splinter 
    if n1(1)<=n2(1)
        nx=round(n1(1)+abs(mpx-n1(1)));
    elseif n1(1)>n2(1)
        nx=round(n1(1)-abs(mpx-n1(1)));
    end
    % Generate the splinter point y coordinate
    inc=-0.5;
    while inc<0.5
        % Find a new y-point by incrementing through different slopes
        % Solve for the new y-point by finding the incrementing slope given
        % the new x-coordinate (nx)
        ny=round(-((m+inc)*(n2(1)-nx)-n2(2)));
        inc=inc+0.1;
        % Check if the new coordinates enter the invalid areas of the
        % binary occupancy grid
        mid=round([(n1(1)+nx)/2 (n1(2)+ny)/2]);
        q1=round([(mid(1)+nx)/2 (mid(2)+ny)/2]);
        q2=round([(n1(1)+mid(1))/2 (n1(2)+mid(2))/2]);
        if mid(1)>0 && mid(2)>0 && q1(1)>0 && q1(2)>0 && q2(1)>0 && q2(2)>0
            if I(nx,ny)~=0
                % Return the new splinter point for further node creation
                % and linkage in cprm
                % If no new point can be created, a connection crossing the
                % binary occupancy grid must be made
                if I(mid(1),mid(2))==1 || I(q1(1),q1(2))==1 || I(q2(1),q2(2))==1
                    inc=1;
                    fprintf('Splinter point created at (%d,%d)\n',nx,ny);
                    sp=[nx ny];
                elseif inc>0.5
                    sp=n1;
                    fprintf('Spline could not be created\n');
                end
            end
        end
    end
end