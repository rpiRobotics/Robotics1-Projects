function R = rot(h,q)
%ROT Rotate along an axis h by q in radius.
%   Generate rotation matrix for such operation.
%   R = I + sin(q)*hat(h) + (1 - cos(q))*hat(h)^2
h=h/norm(h);
R = eye(3) + sin(q)*hat(h) + (1 - cos(q))*hat(h)^2;
end

