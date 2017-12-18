%Tim Chinenov 661295189
function [Arot] = rot(k,theta)
%     theta = double(theta);
  k=k/norm(k);
  Arot = eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
end