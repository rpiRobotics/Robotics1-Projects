%
% fwdkin.m
%
% [R,p]=fwdkin(theta,type,H,P,n)
%
% purpose: general forward kinematics for serial chain
% 
% input:
% theta: n-vector of rotational angle / translational displacement
% type: 0 = rotational  nonzero = prismatic
% H = [ h1 h2 ... hn ] axis of rotation or translation
% P = [p01 p12 p23 .. p_{n-1}n ] inter-link vectors
% n: # of links (>1)
% 
% output:
% R=R_{0n}, p=p_{0n}
%
function [R,p]=fwdkin(theta,type,H,P,n)

  if type(1) == 0
    R=(rot(H(1:3,1),theta(1)));
    p=P(1:3,1);
  else
    R=eye(3,3);
    p=P(1:3,1)+theta(1)*H(1:3,1);
  end
    
  for i = 2:n
    if type(i) == 0
      p=p+R*P(1:3,i);
      R=R*(rot(H(1:3,i),theta(i)));
    else
      p=p+R*(P(1:3,i)+theta(i)*H(1:3,i));
      R=R;
    end
%  disp(i);disp(p)
  end

  if size(P,2)>n
    p=p+R*P(1:3,n+1);
  end
   
