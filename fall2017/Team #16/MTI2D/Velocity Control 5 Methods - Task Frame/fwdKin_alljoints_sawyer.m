function [pp,RR]=fwdKin_alljoints_sawyer(q,type,H,P,n,R_EE)
%% initialize utility variables
R=eye(3);
p=zeros(3,1);
RR = zeros(3,3,n+1);
pp = zeros(3,n+1);
%% forward kinematics
for i = 1:n
    h_i = H(1:3,i);  
    if type(i) == 0 %rev
        pi = P(1:3,i);
        p = p+R*pi;
        Ri = rot(h_i,q(i));         
        R = R*Ri;        
    elseif type(i) == 1 %pris
        pi = P(1:3,i)+q(i)*h_i;
        p = p+R*pi;
	else %default pris
		pi = P(1:3,i)+q(i)*h_i; 
        p = p+R*pi;
    end
    pp(:,i) = p;
    RR(:,:,i) = R;
end
%% end effector T
p=p+R*R_EE*P(1:3,n+1);
pp(:,n+1) = p;
RR(:,:,n+1) = R*R_EE;
end