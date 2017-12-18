function gotoAngles(robot,angles)
%     robot.setJointPositions(int16(fix(angles(1))),int16(fix(angles(2))),int16(fix(angles(3))),int16(fix(angles(4))));
%     pause(.1)
%     robot.setJointPositions(int16(1000*abs(angles(1)-fix(angles(1)))),int16(1000*abs(angles(2)-fix(angles(2)))),int16(1000*abs(angles(3)-fix(angles(3)))),int16(1000*abs(angles(4)-fix(angles(4)))));

    robot.setJointPositions(int16(100*angles(1)),int16(100*angles(2)),int16(100*angles(3)),int16(100*angles(4)));
end