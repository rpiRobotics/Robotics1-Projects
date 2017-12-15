%% Function that generates the Hough line coordinate pairs and rescales them to actual drawing points

function LC=hough_coords(jpg_file,mix,maxx,miy,may)
x = imread(jpg_file);
x = imresize(x,0.6);%0.6 tri 0.6 cube 0.8 da
x = rgb2gray(x);
x = imbinarize(x,'adaptive','ForegroundPolarity','dark','Sensitivity',0.001); % cube 0.001 im 0.2 da 0.001
figure
imshow(x)
c =  imcomplement(x);
bw = bwmorph(c,'skel',3); %cube 5 im 1 da 3
i  = 1;
allLines = struct('point1',{},'point2',{},'theta',{},'rho',{});
while i < 5 %5
    if i > 1
        bw = imread('temp.jpg');
        bw = rgb2gray(bw);
    end
    [H,T,R] = hough(bw);

    P  = houghpeaks(H,5,'threshold',ceil(.75*max(H(:)))); %0.75
    x = T(P(:,2)); y = R(P(:,1));

    lines = houghlines(bw,T,R,P,'FillGap',80,'MinLength',7); % cube 50, im 70 da 80
    figure, imshow(bw), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',8,'Color','black'); % cube 12, im 50 da 8
       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'.','LineWidth',12,'Color','black'); % cube 12 da 12
       plot(xy(2,1),xy(2,2),'.','LineWidth',12,'Color','black'); % cube 12 da 12
       allLines = [allLines,lines];
    end
    img = frame2im(getframe(gca));
    imwrite(img,'temp.jpg');
    i=i+1;
end

% Organize the lines into a list to be read by robot
% This will create a matrix that contains all the lines starting and ending
% coordinates in the following form [x1,y1,x2,y2]
LineCoord = [];
for k = 1:length(allLines)
    newline = [allLines(k).point1(1),allLines(k).point1(2), allLines(k).point2(1),allLines(k).point2(2)];
    LineCoord = [LineCoord; newline];
end
LineCoord = unique(LineCoord,'rows');

% Isolating x and y coordinates for rescaling to Dobot task space
x=[LineCoord(:,1);LineCoord(:,3)];
y=[LineCoord(:,2);LineCoord(:,4)];
x1=LineCoord(:,1);
x2=LineCoord(:,3);
y1=LineCoord(:,2);
y2=LineCoord(:,4);
%270 190 40 -40
x1=round(((x1-min(x))*(maxx-mix))/(max(x)-min(x))+mix);
x2=round(((x2-min(x))*(maxx-mix))/(max(x)-min(x))+mix);
y1=round(((y1-min(y))*(may-miy))/(max(y)-min(y))+miy);
y2=round(((y2-min(y))*(may-miy))/(max(y)-min(y))+miy);

% Create final coordinate storage matrix
LineCoord=[x1 y1 x2 y2];

% Add parameter that stores line slopes (store in 5th column)
for i=1:1:size(LineCoord,1)
    LineCoord(i,5)=round((y2(i)-y1(i))/(x2(i)-x1(i)));
end

% Sort the lines based on their slopes
LineCoord=sortrows(LineCoord,5);

% Calculate the distance of the line (store in 6th column)
sum=0;
m=0;
for i=1:1:size(LineCoord,1)
    dist=sqrt((LineCoord(i,1)-LineCoord(i,3))^2+(LineCoord(i,2)-LineCoord(i,4))^2);
    if dist>m
        m=dist;
    end
    LineCoord(i,6)=dist;
    sum=sum+dist;
end

% Calculate the average line length
avg=sum/length(LineCoord);

% Check and store lines which are not longer than 1/3 of the average length
LC=[];
SC=[];
for i=1:1:size(LineCoord,1)
    if LineCoord(i,6)>avg/3
        LC=[LC;LineCoord(i,:)];
    else
        SC=[SC;LineCoord(i,:)];
    end
end

% Remove the lines which are not long enough (rem) and store lines for
% drawing (nL)
% First checking if the lines are close to other lines that are longer and
% have a similar slope
% If they are, an attempt to generate a similarly sloped longer line is
% made
nL=[];
rem=[];
for i=1:1:size(SC,1)
    for n=1:1:size(LC,1)
        if LC(n,5)==SC(i,5) && round((SC(i,2)-LC(n,2))/(SC(i,1)-LC(n,1)))==LC(n,5)
            rem=[rem;n];
            n1=[SC(i,1) SC(i,2) LC(n,1) LC(n,2) LC(n,5) sqrt((SC(i,1)-LC(n,1))^2+(SC(i,2)-LC(n,2))^2)];
            n2=[SC(i,1) SC(i,2) LC(n,3) LC(n,4) LC(n,5) sqrt((SC(i,1)-LC(n,3))^2+(SC(i,2)-LC(n,4))^2)];
            n3=[SC(i,3) SC(i,4) LC(n,1) LC(n,2) LC(n,5) sqrt((SC(i,3)-LC(n,1))^2+(SC(i,4)-LC(n,2))^2)];
            n4=[SC(i,3) SC(i,4) LC(n,3) LC(n,4) LC(n,5) sqrt((SC(i,3)-LC(n,3))^2+(SC(i,4)-LC(n,4))^2)];
            if n1(6)>n2(6) && n1(6)>n3(6) && n1(6)>n4(6) && n1(6)<m*0.8
                nL=[nL;n1];
            elseif n2(6)>n1(6) && n2(6)>n3(6) && n2(6)>n4(6) && n4(6)<avg*1.5%m*0.8
                nL=[nL;n2];
            elseif n3(6)>n1(6) && n3(6)>n2(6) && n3(6)>n4(6) && n4(6)<avg*1.5%m*0.8
                nL=[nL;n3];
            elseif n4(6)>n1(6) && n4(6)>n2(6) && n4(6)>n3(6) && n4(6)<avg*1.5%m*0.8
                nL=[nL;n4];
            end
        end
    end
end

% Remove duplicate lines
rem=unique(rem,'rows');
nLC=[];
for i=1:1:size(LC,1)
    if ~ismember(i,rem)
        nLC=[nLC;LC(i,:)];
    end
end

% Checking which lines to remove
rem=[];
for i=1:1:size(nLC,1)
    for n=1:1:size(nLC,1)
        if i~=n && nLC(i,5)==nLC(n,5) && (abs(nLC(i,1)-nLC(n,1))<=15 && abs(nLC(i,2)-nLC(n,2))<=15 && abs(nLC(i,3)-nLC(n,3))<=15 && abs(nLC(i,4)-nLC(n,4))<=15)
            if nLC(i,6)>nLC(n,6)
                rem=[rem;n];
            else
                rem=[rem;i];
            end
        end  
    end
end

rem=unique(rem,'rows');
enLC=[];
for i=1:1:size(nLC,1)
    if ~ismember(i,rem)
        enLC=[enLC;nLC(i,:)];
    end
end


enL=[];
for i=1:1:size(nL,1)
    for n=1:1:size(nL,1)
        if i~=n && nL(i,5)==nL(n,5) && (abs(nL(i,1)-nL(n,1))<=15 && abs(nL(i,2)-nL(n,2))<=15 && abs(nL(i,3)-nL(n,3))<=15 && abs(nL(i,4)-nL(n,4))<=15)
            if nL(i,6)>nL(n,6)
                enL=[enL;nL(i,:)];
            else
                enL=[enL;nL(n,:)];
            end
        end
    end    
end
enL=unique(enL,'rows');



% Plot of final lines to draw/return
LC=[enLC;enL];
LC=unique(LC,'rows');
LC=sortrows(LC,5);
length(LC);
figure
for i=1:1:size(LC,1)
    plot([LC(i,1) LC(i,3)],[LC(i,2) LC(i,4)],'LineWidth',3,'Color','blue');
%     pause(1)
    hold on 
end
end


