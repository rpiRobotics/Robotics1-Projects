function [worldPoints, cameraParams] = img_calibration()
clear all
%% Load checkboard pictures
numImages = 36;
files = cell(1, numImages);
for i = 1:numImages
    files{i} = fullfile('E:\','Users', 'Sean Cai', 'Documents', 'MATLAB', ...
        'Robotics_1', 'Project', 'Camera', 'Raw', sprintf('image_%d.jpg', i));
end

% Display one of the calibration images
magnification = 100;
% I = imread(files{1});
% figure; imshow(I, 'InitialMagnification', magnification);
% title('One of the Calibration Images');

%% Camera Calibration
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 17; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
cameraParams = estimateCameraParameters(imagePoints, worldPoints);

% Evaluate calibration accuracy.
% figure; showReprojectionErrors(cameraParams);
% title('Reprojection Errors');


