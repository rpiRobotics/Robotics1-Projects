function [ R ] = rot( k, q )
    R = eye(3,3) + sin(q)*hat(k) + (1-cos(q))*(hat(k))^2;
end

