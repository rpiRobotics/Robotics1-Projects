function [ y ] = Lowpass(x,alpha )
%Lowpass
%alpha=dt/(RC+dt);
y=zeros(1,length(x));
y(1)=x(1);
for i=2:length(x)
    y(i)=alpha*x(i)+(1-alpha)*y(i-1);
end

end

