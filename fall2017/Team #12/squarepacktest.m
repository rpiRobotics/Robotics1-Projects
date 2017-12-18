M=[5,4,3,1];
bin_x=[0,10];
bin_y=[0,10];

points=squarepack(M,bin_x,bin_y)
hold on
axis([min(bin_x) max(bin_x) min(bin_y) max(bin_y)])
for(i=1:length(points))
    plot(points(1,i),points(2,i),'r*');
end