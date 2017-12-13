function [ axang ] = quat2axang( q )
%convert a unit quaternion to angle/axis representation
s = norm(q(2:4));
if s >= 10*eps(class(q))
    vector = q(2:4)/s;
    theta = 2*atan2(s,q(1));
else
    vector = [0,0,1];
    theta = 0;
end

axang = [vector,theta];

end

