function [J_est] = callJacobian(q)
ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1]; zz = [0;0;0];
type = zeros(3,1);


n = 3;

R01 = rot(-ez,q(1));
R12 = rot(-ey,q(2));
R23 = rot(-ey,q(3));
% 
p01 = zz; p12 = zz; p23 = .15*ex; p3T = .17*ex + .02*-ez;
p1T = p12+p23+p3T;
p2T = p1T;
P = [p01 R01*p12 R01*R12*p23 R01*R12*R23*p3T];
% 
h1 = R01*-ez; h2 = R01*R12*-ey; h3 = R01*R12*R23*-ey;
H = [h1 h2 h3];
% 
% J = [cross(h1,R01*p1T) cross(h2,R01*R12*p2T) cross(h3,R01*R12*R23*p3T)];
ang = q;
dq = 1e-4;
for i = 1:1:n
    dq_v = zeros(n,1); dq_v(i) = dq;
    [R, P1] = fwdkin(ang-dq_v,type,H,P,n);
    [R, P2] = fwdkin(ang+dq_v,type,H,P,n);
    J_est(:,i) = (P2-P1)/(2*dq);
end


end

