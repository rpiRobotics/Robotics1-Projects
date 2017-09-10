clear err

N=1000;

for i=1:N

k=rand(3,1);k=k/norm(k);
theta=(rand-.5)*2*pi;
p=randn(3,1);
q=randn(3,1);

d=norm(q-rot(k,theta)*p);

th=subproblem3(k,p,q,d);

err(i)=min([abs(th(1)-theta) abs(th(2)-theta) ...
    abs(th(1)-theta+2*pi) abs(th(2)-theta+2*pi) ...    
    abs(th(1)-theta-2*pi) abs(th(2)-theta-2*pi) ...    
    ]);

if err(i)>eps*2000000;
    disp(sprintf('| theta error | = %0.5g',err(i)));
    error('mismatch!');
    return;
end

end

disp(['subproblem 3 checked out after ',num2str(N),' runs']);
disp(sprintf('theta error norm = %0.5g',norm(err)));
figure(1);plot((1:N),err,'x');


