%% Function returns path points using PRM for inputted .jpg file 

function fc=cprm(image)
    clc
    clear
    % Reading and reprocessing the image 
    I=imread('t3.jpg');
    I=rgb2gray(I);
    I=imresize(I,0.1); 
    % Binarization and inverse for binary occupancy grid
    I=imbinarize(I,'adaptive','ForegroundPolarity','dark','Sensitivity',0.25);
    I=~I;

    %% Gathering corner points
    rc1=detectHarrisFeatures(I,'MinQuality',0.2);
    rc1=round(rc1.Location());

    %% Removing corner points that aren't in valid regions of binary occupancy grid 
    r=[];
    for i=1:1:size(rc1,1)
        if rc1(i,1)>size(I,1) || rc1(i,2)>size(I,2)
            r=[r;i];
        end
    end
    rc=[];
    for i=1:1:size(rc1,1)
        if ~ismember(i,r)
            rc=[rc;rc1(i,1) rc1(i,2)];
        end
    end
    % Algorithm to include only edge points centered in valid regions
%     r=[];
%     for i=1:1:size(rc,1)
%         if I(rc(i,1),rc(i,2))~=1 
%             if I(rc(i,1)+5,rc(i,2))==1
%                 rc(i,1)=rc(i,1)+5;
%             elseif I(rc(i,1)-5,rc(i,2))==1
%                 rc(i,1)=rc(i,1)-5;
%             elseif I(rc(i,1),rc(i,2)+5)==1
%                 rc(i,2)=rc(i,2)+5;
%             elseif I(rc(i,1),rc(i,2)-5)==1
%                 rc(i,2)=rc(i,2)-5;
%             else
%                 r=[r;i];
%             end
%         end
%     end

    c=[];
    for i=1:1:size(rc,1)
        if ~ismember(i,r)
            c=[c;rc(i,1) rc(i,2)];
        end
    end
    % Plotting all valid points
    figure
    imshow(I)
    hold on
    for i=1:1:size(c,1)
        plot(c(i,1),c(i,2),'*','LineWidth',5,'color','red')
        hold on
    end
    
    %% Trim points down to inputted number of nodes
    % Enter specific number of nodes separated by a specific minimum
    % distance
    fprintf('Current number of nodes: %d\n',size(c,1));
    % nodes=input('Enter number of desired nodes (int):  ');
    nodes=30;
    %nsep=input('Enter desired node separation (int):  ');
    nsep=30;
    counts=1;
    sz=size(c,1);
    % Attempt to create nodes using dist_rem to calculate next closest node
    while sz>nodes && counts<nsep
        if counts == 1
            nc=dist_rem(c,counts);
        else
            nc=dist_rem(nc,counts);
        end
        sz=size(nc,1);
        counts=counts+1;
    end
    fprintf('Number of nodes: %d\n',sz);
    % Plot nodes
    figure
    hold on
    imshow(I)
    hold on
    for i=1:1:size(nc,1)
        plot(nc(i,1),nc(i,2),'*','LineWidth',5,'color','green')
        hold on
    end

    %% Create initial path by ordering nodes by proximity
    d=[nc(1,1),nc(1,2)];
    used=[1];
    cond=0;
    count=2;
    % Order nodes based on proximity to one another
    % Iterate through all nodes and store next closest node in c
    while cond==0
        dist=1000;
        for i=2:1:size(nc,1)
            if ~ismember(i,used)
                s=sqrt((d(1,1)-nc(i,1))^2+(d(1,2)-nc(i,2))^2);
                if s<dist
                    dist=s;
                    used(count,1)=i;
                end
            end
        end
        d=[nc(used(size(used,1),1),1),nc(used(size(used,1),1),2)];
        count=count+1;
        if size(used,1)==size(nc,1) 
            cond=1;
        end
    end
    % Storing points in ordered matrix c
    c=[];
    for i=1:1:size(used,1)
        c=[c;nc(used(i,1),1) nc(used(i,1),2)];
    end
    c=[c;nc(1,1) nc(1,2)];
    % Plot ordered points with pause to view node order
    figure 
    imshow(I)
    hold on
    for i=1:1:size(c,1)
        plot(c(i,1), c(i,2), '*','LineWidth',5,'color','green')
        hold on
        pause(0.05)
    end
    %% Create path between nodes, implementing splinters when needed
    fc=[];
    for i=1:1:length(c)-1
        n1=[c(i,1) c(i,2)];
        n2=[c(i+1,1) c(i+1,2)];
        % Check path between initial point and next point for intersection
        % with invalid region of binary occupancy grid
        mid=round([(n2(1)+n1(1))/2 (n2(2)+n1(2))/2]);
        q1=round([(mid(1)+n1(1))/2 (mid(2)+n1(2))/2]);
        q2=round([(n2(1)+mid(1))/2 (n2(2)+mid(2))/2]);
        % No splinter node for splining needs to be created if a direct
        % connection can be made: 1
        % Splining needed if connection can't be made: connect = 0
        if I(mid(1),mid(2))~=1 || I(q1(1),q1(2))~=1 || I(q2(1),q2(2))~=1 || sqrt((n2(1)-n1(1))^2+(n2(2)-n1(2))^2)>10
            connect=0;
        else
            connect=1;
        end
        if connect==1
            fprintf('No spline needed between %d and %d\n',i, i+1);
            fc=[fc;n1];
        else
            fprintf('Spline made between %d and %d\n',i, i+1);
        end
        while connect==0
            % Call splinter function for form splining point, maintain
            % order by appending spline points sequentially
            sp=splinter(n1,n2,I);
            fc=[fc;n1];
            if n1~=sp
                fc=[fc;sp];
            end
            connect=1;
        end
        if i==length(c)-1
            fc=[fc;fc(1,1) fc(1,2);fc(2,1) fc(2,2)];
        end
    end
    
    % Rescale all points and spline points to Dobot task space
    x=fc(:,1);
    y=fc(:,2);
    x=round(((x-min(x))*(270-190))/(max(x)-min(x))+190);
    y=round(((y-min(y))*(40-(-40)))/(max(y)-min(y))+(-40));
    fc=[x y]
    
    % Plot ordered nodes and spline points
    figure
    for i=1:1:size(fc,1)
        plot(fc(i,1),fc(i,2),'*','LineWidth',5,'color','blue')
        hold on
        pause(0.5)
    end
end
