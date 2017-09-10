function [ p0A] = locPos(T)
% locPos: Locate Position
%   Converts from AR to Camera transform into p0T for the arm.

p0C = [.26; 0; .775]; % camera position vector from inertia origin
pCA = T(1:3,4); % Camera position vector from AR Tag origin
% check notation and direction of this with jacob

p0A = p0C + rotx(pi)*rotz(pi/2)*pCA;

end