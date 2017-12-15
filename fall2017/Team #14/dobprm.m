%% Function using PRM toolbox to gather coordinates

function coords=dobprm(jpg_file)
% Read an binarize image for binary occupancy grid
I=imread(jpg_file);
I=rgb2gray(I);
I=imresize(I,1);
I=imbinarize(I,'adaptive','ForegroundPolarity','dark','Sensitivity',0.5);
map=robotics.BinaryOccupancyGrid(I,2);
% show(map);
robotRadius=1;
mapInflated=copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated)

% Run PRM toolbox to generate paths
prm=robotics.PRM
prm.Map=mapInflated;
prm.NumNodes=1;
prm.ConnectionDistance=10;
disp('Select end points from plot, click enter to finish');
pts=ginput()

% Iterate through nodes until a path can be created
for i=1:1:length(pts)-1
    startLocation=[pts(i,1) pts(i,2)];
    endLocation=[pts(i+1,1) pts(i+1,2)];
    prm.NumNodes=1;
    if i==1
        path=findpath(prm,startLocation,endLocation);
        while isempty(path)
            prm.NumNodes = prm.NumNodes+5;
            update(prm);
            path=findpath(prm,startLocation,endLocation);
        end
    else
        npath=findpath(prm,startLocation,endLocation);
        while isempty(npath)
            prm.NumNodes = prm.NumNodes+5;
            update(prm);
            npath=findpath(prm,startLocation,endLocation);
        end
        figure
        show(prm)
        path=[path;npath];
    end
    figure
    show(prm)
end

% Scale generated path to task space
p=round(path)
x=round(((p(:,1)-min(p(:,1)))*(270-190))/(max(p(:,1))-min(p(:,1)))+190);
y=round(((p(:,2)-min(p(:,2)))*(40-(-40)))/(max(p(:,2))-min(p(:,2)))+(-40));
coords=[x y]

% Show final paths
figure
for i=1:5:length(p)
    plot(x(i,1),y(i,1),'*')
    hold on
end

end