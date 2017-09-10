function [  ] = CircleFollow( phantomX,radius,normvec,center )
%Command phantomX unit to follow a circle in 3-space defined by a radius, a
%vector normal to the circle plane, and the circle center. 
path=Circle3(radius,normvec,center(1),center(2),center(3));
for i=1:length(path)
    phantomX.p0T = path(:,1);
    phantomX.R0T = rotz(atan(phantomX.p0T(2)/phantomX.p0T(1)));
    [ q ] = phantomX_InverseKinematics_JS(phantomX);
    steps = rad2step(q);
    delay(0.3);
    phantomX.robot.getJointPositions()
    phantomX.robot.setJointPositions(int16([steps;300]))
    %smoothMotion(phantomX,path(:,i)); %Maysbe want to use different algo for this
end

end
