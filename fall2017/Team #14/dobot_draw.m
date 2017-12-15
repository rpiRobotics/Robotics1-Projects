% Advanced edge detection algorithm using find_coords()

clc
clear

c=find_coords('t7.jpg');
x=c(:,1);
y=c(:,2);
% (value - curnt_min)*(new_max - new_min) / (curnt_max - curnt_min) + new_min
x=round(((x-min(x))*(260-200))/(max(x)-min(x))+200);
y=round(((y-min(y))*(30-(-30)))/(max(y)-min(y))-(-30));
figure
for n=1:5:length(c)
    plot(x(n,1),y(n,1),'*')
    hold on
end
