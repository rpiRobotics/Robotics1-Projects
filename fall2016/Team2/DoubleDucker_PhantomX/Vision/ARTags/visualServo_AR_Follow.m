% Initialize PhantomX Arm
run('phantomX_Inital.m')
pause(5)

% Initalize Camera for Visual Servoing
run('camera_Initial.m')

while True
    pause(2);
    T = I4; % initalize homogenous transfrom of AR Tag to Camera
    while T == I4
        T = arDetect(IP, ID);
    end
    T1 = I4;
    while T1 == I4
        T1 = arDetect(IP, ID1);
    end
    p0A = locPos(T1);
    p0A_int = p0A+ .07*[0;0;1];
    
    phantomX.q = smoothMotion(phantomX, p0A_int);
end