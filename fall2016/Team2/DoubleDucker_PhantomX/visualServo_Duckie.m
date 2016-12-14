% Sorts two different ARTag'd Duckies

% Initialize PhantomX Arm
run('phantomX_Inital.m')
pause(5)

% Initalize Camera for Visual Servoing
run('camera_Initial_Duckie.m')

dropOff_blue = [0.10;-0.20;0.20]; % drop off location for blue ducks
dropOff_red =  [0.10; 0.20;0.20]; % drop off location for red ducks

num_Tags = 3; % indicate the total number of ducks to be sorted
while num_Tags > 0 % continue to search for ducks until all have been sorted
    
    pause(2);
    
    % initalize homogenous transfrom of AR Tag to Camera
    T_red = I4;
    T_blue = I4;
    
    % search workspace until a duck has been identified
    while (T_red == I4) & (T_blue == I4)
        
        T_red = arDetect(IP, red);
        T_blue = arDetect(IP, blue);
    end
    
    % initialize the duckie
    T_duckie = I4;
    
    % set duck to appropriate color and drop off location
    if T_red(1:3,1:3) ~= I3
        T_duckie = T_red;
        dropOff = dropOff_blue;
    else
        T_duckie = T_blue;
        dropOff = dropOff_red;
    end
    
    % solve for position vector from inertial to task frame
    p0A = locPos(T_duckie);
    p0A_int = p0A+ .1*[0;0;1];
    
    % arm will go to intermediate position directly above duck before picking up
    phantomX.q = smoothMotion(phantomX, p0A_int);
    duckieDrop(phantomX)
    pause(2);
    phantomX.q = smoothMotion(phantomX, p0A);
    duckieGrab(phantomX)
    
    % move duck to dropoff location
    phantomX.q = smoothMotion(phantomX, p0A_int);
    phantomX.q = smoothMotion(phantomX, dropOff);
    duckieDrop(phantomX)
    pause(2)
    
    % adjust total number of ducks count and repeat
    num_Tags = num_Tags - 1;
    phantomX = Home(phantomX);
    phantomX = Home(phantomX);
    
end