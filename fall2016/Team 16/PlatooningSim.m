%Simulation for linear platooning

X0init = 3;
Y0init = 2;
V0 = .06;
pause('on');

%set start positions
X0 = X0init;
Y0 = Y0init;
X1 = X0 - .5;
X2 = X1 - .5;

p01d = .5;
p12d = .5;

plot(X0, Y0, 'ro', X1, Y0, 'bo', X2, Y0, 'ko');
axis([0 20 0 4]);
pause(.5);

for i = 1:300
    figure(1);
    plot(X0, Y0, 'ro', X1, Y0, 'bo', X2, Y0, 'ko');
    axis([0 20 0 4]);
    if i <= 200 
        X0 = X0 + V0;
    else 
        V0 = 0;
    end
    
    V1 = .06*((X0-X1)-(.5+V0));
    V1 = min(V1, 0.07);
    X1 = X1 + V1;
    V2 = .06*((X1-X2)-(.5+V1));
    V2 = min(V2, 0.07);
    X2 = X2 + V2;
    pause(.025)
end