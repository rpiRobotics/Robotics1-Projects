function movePen(robot,x,y,z,d)
    [qv1,qv2]=invkinPEN([x,y,z]',d-2);
    angles = [(qv1(1)*180/pi) (qv1(2)*180/pi) (qv1(3)*180/pi) 0]
    gotoAngles(robot,angles);
    pause(.3);
end