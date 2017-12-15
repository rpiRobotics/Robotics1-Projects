%% Code used for full scanning loop

clc
clear

%%

%image that is being searched for
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
Q = zeros(1,3,16);
[Q(:,:,1),~] = ikdobot(199,0,-10); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
start = fkdobot(0,0,0);
[Q(:,:,1),~] = ikdobot(start(1),start(2),start(3)); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(1);

searchedImage = imread('axis.jpg');
%Initialize and start webcam
cam = webcam('Logitech');
Prevcoor = [0 0];
preview(cam)
%wait until you have seen the image in your webcam.
while (true)
img = snapshot(cam);
mainImage = img;

imageFound = ImagePresent(mainImage,searchedImage);
if imageFound(1) > 0 && sameLocation([imageFound(2) imageFound(3)],Prevcoor)
    disp('I found the image');
    break;
end
Prevcoor = [imageFound(2) imageFound(3)];
pause(1);    
end    

plot(imageFound(2),imageFound(3),'X','LineWidth',2,'Color','red');
plot(102,360,'X','LineWidth',2,'Color','red');

hold off;
%Remove all items outside of the picture frame
drawing = imcrop(mainImage,[imageFound(2)-33.5 imageFound(3)-410 450 310]);

imshow(drawing)
savingimg = frame2im(getframe(gca));
% savedimg = imwrite(savingimg,'campic.jpg');
file=dir('C:\Users\evan_\Desktop\Dobot');
fd=fullfile('C:\Users\evan_\Desktop\Dobot','campic.jpg');
imwrite(savingimg,fd);

%Shimmy robot to show image was picked up

[Q(:,:,1),~] = ikdobot(216,0,90); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(.1);
start = fkdobot(0,0,0);
[Q(:,:,1),~] = ikdobot(start(1),start(2),start(3)); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(2);

hold off;
%Search for the drawing area
preview(cam)
pause(1);
Prevcoor = [0 0];
while (true)
    img = snapshot(cam);
    mainImage = img;
    
    points = findArea(mainImage);
    if (length(points) > 2)
        disp('Found drawing region')
        break;
    end
    pause(5);
end
disp(points)
hold on;
imshow(img)
plot(points(:,1),points(:,2),'x','LineWidth',5,'Color','green');
robotPoints = [];
for k=1:length(points(:,1))
    xyz = pinhole(points(k,:));
    xyz(1) = xyz(1)+40;
    xyz(2) = xyz(2);
    robotPoints = [robotPoints; xyz];
end


for k=1:length(robotPoints)
    [Q(:,:,1),~] = ikdobot(double(robotPoints(k,1)),double(robotPoints(k,2)),double(0)); 
    robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
    robot.getJointPositions()
    pause(1);
end

pause(3);
[Q(:,:,1),~] = ikdobot(199,0,-10); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
start = fkdobot(0,0,0);
[Q(:,:,1),~] = ikdobot(start(1),start(2),start(3)); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(1);

%Shimmy robot to show drawing area was picked up
[Q(:,:,1),~] = ikdobot(216,0,90); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(.1);
start = fkdobot(0,0,0);
[Q(:,:,1),~] = ikdobot(start(1),start(2),start(3)); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(2);

maxx=0;
may=-1000;
mix=1000;
miy=1000;
for i=1:size(robotPoints,1)
    if maxx<robotPoints(i,1)
        maxx=round(robotPoints(i,1));
    elseif mix>robotPoints(i,1)
        mix=round(robotPoints(i,1));
    end
    if may<robotPoints(i,2)
        may=round(robotPoints(i,2));
    elseif miy>robotPoints(i,2)
        miy=round(robotPoints(i,2));
    end
end


%
mix=190;
maxx=270;
miy=-40;
may=40;
mix
maxx
miy
may

coords=hough_coords('supertriangle.jpg',mix,maxx,miy,may)
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

% calib=[180 280 280 180 220; 100 100 -100 -100 0];
% length(calib);
% pause(1);
% for i=1:1:length(calib)
%    Q=zeros(1,3,16);
%    [Q(:,:,1),~]=ikdobot(calib(1,i),calib(2,i),30)
%    robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
%    robot.getJointPositions()
%    pause(1)
% end
% disp('Calibration completed')
% pause(2)
disp('Begining drawing')
p=0.5;
size(coords);
for n=1:size(coords,1)
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(coords(n,1)),double(coords(n,2)),20);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p)
   
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(coords(n,1)),double(coords(n,2)),-10);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p)
   
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(coords(n,3)),double(coords(n,4)),-10);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p)
   
   Q=zeros(1,3,16);
   [Q(:,:,1),~]=ikdobot(double(coords(n,3)),double(coords(n,4)),20);
   robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
   robot.getJointPositions()
   pause(p)
end
disp('Enjoy your scribble')
Q=zeros(1,3,16);
[Q(:,:,1),~]=ikdobot(270,100,30)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
pause(10)
Q=zeros(1,3,16);
[Q(:,:,1),~]=ikdobot(220,0,30)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(50));
robot.getJointPositions()
disp('At start')
