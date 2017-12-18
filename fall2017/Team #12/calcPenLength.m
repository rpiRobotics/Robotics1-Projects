function [d]=calcPenLength()
    ex = [1; 0; 0];
    ey = [0; 1; 0];
    ez = [0; 0; 1];
    l1 = 103;
    l2 = 135;
    l3 = 160;
    Lg = 56; 
    robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
    start_angles = [0 0 30 0];
    gotoAngles(robot,start_angles);
    while 1
        c = menu('','Raise Pen .2 deg','Lower Pen .2 deg','Raise Pen 1 deg','Lower Pen 1 deg','GOOD!');
        switch c
            case 1
                start_angles = start_angles + [0 0 -.2 0] ;
            case 2
                start_angles = start_angles +  [0 0 .2 0] ;
            case 3
                start_angles = start_angles + [0 0 -1 0] ;
            case 4
                start_angles = start_angles +  [0 0 1 0] ;   
            case 5
                break
        end
        gotoAngles(robot,start_angles);
    end
    q1=double(start_angles(1))*pi/180;
    q2=double(start_angles(2))*pi/180;
    q3=double(start_angles(3))*pi/180;
    q4=double(start_angles(4))*pi/180;
    d = l1*ez + rotk(ez,q1)*rotk(ey,q2)*l2*ez + rotk(ez,q1)*rotk(ey,q3)*l3*ex+[Lg*cos(q1);Lg*sin(q1);0];
    d=d(3);
end