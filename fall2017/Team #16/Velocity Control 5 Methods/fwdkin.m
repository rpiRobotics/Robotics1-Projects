function [R,p]=fwdKin(q,type,H,P,n)
R=eye(3);
p=zeros(3,1);
for i = 1:n
    h_i = H(1:3,i);
    Ri=eye(3);

    if type(i) == 0 %rev
        pi = P(1:3,i);
        p = p+R*pi;
        Ri = rot(h_i,q(i));         
        R = R*Ri;        
    elseif type(i) == 1 %pris
        pi = P(1:3,i)+q(i)*h_i;
%         Ri = Ri; 
        p = p+R*pi;
%         R = R;
	else %default pris
		pi = P(1:3,i)+q(i)*h_i; 
        p = p+R*pi;
    end
end

%% End Effector T
p=p+R*P(1:3,n+1);
end