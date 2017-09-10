N=1000;

for i=1:N

k=rand(3,1);k=k/norm(k);
p=randn(3,1);theta=(rand-.5)*2*pi;
q=rot(k,theta)*p;

th=subproblem1(k,p,q);
err(i)=abs(th-theta);

if err(i)>eps*200;
    disp(sprintf('| theta-theta solution| = %0.5g',err(i)));
    error('mismatch!');
    return;
end

end

disp(['subproblem 1 checked out after ',num2str(N),' runs']);
disp(sprintf('error norm = %0.5g',norm(err)));
figure(1);plot((1:N),err,'x');

