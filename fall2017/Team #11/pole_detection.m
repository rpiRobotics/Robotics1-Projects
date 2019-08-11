clear all 

load('C270_Params.mat');
magnification = 100; 

%% Acquire a picture
webcamlist; 
cam = webcam(1);
% preview(cam)

imOrig = snapshot(cam);

%% Read the image of object
% imOrig = imread(fullfile('E:\','Users', 'Sean Cai', 'Documents', 'MATLAB', ...
%         'Robotics_1', 'Project', 'Camera', 'pole_location_3.jpg'));
% figure; imshow(imOrig, 'InitialMagnification', magnification);
% title('Input Image');

%% Undistort the image
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
% figure; imshow(im, 'InitialMagnification', magnification);
% title('Undistorted Image');

%% Segment Objects
% Convert RGB image to chosen color space
I = rgb2ycbcr(im);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 117.000;
channel1Max = 156.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 43.000;
channel2Max = 78.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 135.000;
channel3Max = 205.000;

% Create mask based on chosen histogram thresholds
imCoin = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% figure; imshow(imCoin, 'InitialMagnification', magnification);
% title('Segmented Coins');

%% Detect Objects
% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 300, 'ExcludeBorderBlobs', true);
[areas, boxes] = step(blobAnalysis, imCoin);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the three largest components.
boxes = double(boxes(idx(1:3), :));

% Adjust for coordinate system shift caused by undistortImage
%boxes(:, 1:2) = bsxfun(@plus, boxes(:, 1:2), newOrigin);

% Reduce the size of the image for display.
scale = magnification / 100;
imDetectedCoins = imresize(im, scale); 

% Insert labels for the coins.
imDetectedCoins = insertObjectAnnotation(imDetectedCoins, 'rectangle', ...
    scale * boxes, 'pole');
figure; imshow(imDetectedCoins);
title('Detected Poles');

%% Compute Extrinsics
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

%% Measure the first coin
% Get the top-left and the top-right corners.
box1 = double(boxes(1, :));
imagePoints1 = [box1(1:2); ...
                box1(1) + box1(3), box1(2)];

% Get the world coordinates of the corners
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

% Compute the diameter of the coin in millimeters.
d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
% fprintf('Measured diameter of the first penny = %0.2f mm\n', diameterInMillimeters);

%% Measure the second coin
% Get the top-left and the top-right corners.
box2 = double(boxes(2, :));
imagePoints2 = [box2(1:2); ...
                box2(1) + box2(3), box2(2)];

% Apply the inverse transformation from image to world
worldPoints2 = pointsToWorld(cameraParams, R, t, imagePoints2);

% Compute the diameter of the coin in millimeters.
d = worldPoints2(2, :) - worldPoints2(1, :);
diameterInMillimeters = hypot(d(1), d(2));
% fprintf('Measured diameter of the second penny = %0.2f mm\n', diameterInMillimeters);

%% Measure the third coin
% Get the top-left and the top-right corners.
box3 = double(boxes(3, :));
imagePoints3 = [box3(1:2); ...
                box3(1) + box3(3), box3(2)];

% Apply the inverse transformation from image to world
worldPoints3 = pointsToWorld(cameraParams, R, t, imagePoints3);

% Compute the diameter of the coin in millimeters.
d = worldPoints3(2, :) - worldPoints3(1, :);
diameterInMillimeters = hypot(d(1), d(2));
% fprintf('Measured diameter of the third penny = %0.2f mm\n', diameterInMillimeters);

%% Measure the distance between the first and second coin
% Compute the center of the first coin in the image.
center1_image = box1(1:2) + box1(3:4)/2;
worldPoints1 = pointsToWorld(cameraParams, R, t, center1_image);

% Compute the center of the second coin in the image.
center2_image = box2(1:2) + box2(3:4)/2;
worldPoints2 = pointsToWorld(cameraParams, R, t, center2_image);

dx_12 = worldPoints1(1) - worldPoints2(1); 
dy_12 = worldPoints1(2) - worldPoints2(2); 
fprintf('Distance from the first to the second pole in x direction = %0.2f mm\n', dx_12);
fprintf('Distance from the first to the second pole in y direction = %0.2f mm\n', dy_12);

%% Measure the distance between the second and third coin
% Compute the center of the second coin in the image.
center2_image = box2(1:2) + box2(3:4)/2;
worldPoints2 = pointsToWorld(cameraParams, R, t, center2_image);

% Compute the center of the third coin in the image.
center3_image = box3(1:2) + box3(3:4)/2;
worldPoints3 = pointsToWorld(cameraParams, R, t, center3_image);

dx_23 = worldPoints2(1) - worldPoints3(1); 
dy_23 = worldPoints2(2) - worldPoints3(2); 
fprintf('Distance from the second to the third pole in x direction = %0.2f mm\n', dx_23);
fprintf('Distance from the second to the third pole in y direction = %0.2f mm\n', dy_23);