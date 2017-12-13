function [ rslt ] = quatmultiply( q1, q2 )
% quaternion multplication for two quaternions. 

% scalar = s1*s2 - dot(v1,v2)
scalar = q1(:,1).*q2(:,1) - q1(:,2).*q2(:,2) - ...
             q1(:,3).*q2(:,3) - q1(:,4).*q2(:,4);

% vector = s1*v2 + s2*v1 + cross(v1,v2)
vector = [q1(:,1).*q2(:,2) q1(:,1).*q2(:,3) q1(:,1).*q2(:,4)] + ...
         [q2(:,1).*q1(:,2) q2(:,1).*q1(:,3) q2(:,1).*q1(:,4)]+...
         [ q1(:,3).*q2(:,4)-q1(:,4).*q2(:,3) ...
           q1(:,4).*q2(:,2)-q1(:,2).*q2(:,4) ...
           q1(:,2).*q2(:,3)-q1(:,3).*q2(:,2)];

rslt = [scalar  vector];

end

