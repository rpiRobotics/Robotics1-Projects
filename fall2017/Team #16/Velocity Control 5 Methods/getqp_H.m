function [ H ] = getqp_H(dq, J, vr, vp, er, ep)
n = length(dq);
H1 = [J,zeros(6,2)]'*[J,zeros(6,2)];

H2 = [zeros(3,n),vr,zeros(3,1);zeros(3,n),zeros(3,1),vp]'*[zeros(3,n),vr,zeros(3,1);zeros(3,n),zeros(3,1),vp];

H3 = -2*[J,zeros(6,2)]'*[zeros(3,n),vr,zeros(3,1);zeros(3,n),zeros(3,1),vp];
H3 = (H3+H3')/2;

H4 = [zeros(1,n),sqrt(er),0;zeros(1,n),0,sqrt(ep)]'*[zeros(1,n),sqrt(er),0;zeros(1,n),0,sqrt(ep)];

H = 2*(H1+H2+H3+H4);

end

