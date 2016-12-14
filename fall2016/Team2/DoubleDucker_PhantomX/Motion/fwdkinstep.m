% Robotics 1 - Code provided by John Wen
%
% iterative computation of forward kinematics 
% with Jacobian represented in end effector frame: ( (J_n)_n)
%
function [R,p,Jtr]=fwdkinstep(R,p,Jtr,theta,H,type,P,i)
  
  h = H(:,i);
  pvec = P(:,i);
  th = theta(i);
  if (type(i)==0)
    R_i_ip1 = (rot(h,th));%expm(hat(h)*th);
    p_i_ip1 = pvec;
    Hvec = [h;zeros(3,1)];
  else
    R_i_ip1 = eye(3,3);
    p_i_ip1 = pvec + h * th;
    Hvec = [zeros(3,1);h];    
  end

  p = p + R * p_i_ip1;
  R = R * R_i_ip1;
  
  phatR=hat(p_i_ip1)*R_i_ip1;
  Phitr= [R_i_ip1 phatR;zeros(3,3) R_i_ip1];
  
  if isempty(Jtr)
    Jtr=Hvec';
  else
    Jtr=[ Jtr*Phitr ; Hvec' ];
  end
  
