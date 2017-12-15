function coords = find_coords(jpg_file)
% Binarize image
I=imread(jpg_file);
I=rgb2gray(I);
I=imresize(I,0.15);
I=imbinarize(I,'adaptive','ForegroundPolarity','dark','Sensitivity',0.08);

% Find edge points with corner()
c=corner(I);
ordered=[c(1,1) c(1,2)];

% Order points based on proximity relative to one another relative to the 
% summation of x and y distances
while length(c) ~= 2
    d=[];
    m=0;
    for n=2:1:length(c)
        m=m+1;
        dx=abs(c(1,1)-c(n,1));
        dy=abs(c(1,2)-c(n,2));
        t=dx+dy;
        d(m,1)=t;
    end
    [x y]=min(d);
    a=[c(y+1,1) c(y+1,2)];
    c(1,:)=[];
    c(y,:)=[];
    c=[a;c];
    ordered=[ordered;a];
end
% Store points in ordered and plot final path to expect
coords=ordered;
x=ordered(:,1);
y=ordered(:,2);
figure
for n=1:1:length(ordered)
    plot(x,y,'LineStyle','-')
    hold on
end

figure
for n=1:1:length(ordered)
    plot(x,y,'*')
    hold on
end
end



