N=2000;

for i=1:N

k=rand(3,1);k=k/norm(k);
p=randn(3,1);q=randn(3,1);
p=p-p'*k*k;
q=q-q'*k*k;q=q/norm(q)*norm(p);

theta=subproblem0(p,q,k);
theta1=acos(p'*q/norm(p)^2);if k'*cross(p,q)<0;theta1=-theta1;end

err(i)=norm(rot(k,theta)*p-q);
err1(i)=norm(rot(k,theta1)*p-q);
%check 
%disp('[rot(k,theta)*p   q]');
disp([rot(k,theta)*p q])

if err(i)>eps*200;
    disp(sprintf('|| q-rot(k,theta)p || = %0.5g',err(i)));
    error('mismatch!');
    return;
end
if err1(i)>1e-12;
    disp(sprintf('theta = %0.5g, error = %0.5g',theta1,err1(i)));
    error('acos formula has larger error');
    return;
end

end

disp(['subproblem 0 checked out after ',num2str(N),' runs']);
disp(sprintf('atan2 error = %0.5g, acos error = %0.5g',...
    norm(err),norm(err1)));
figure(1);plot((1:N),err,'x',(1:N),err1,'o');
legend('atan2','acos');
