calib_offset_w = 0.67;
a = duck.april_tags;
duck.sendCmd(0.1,calib_offset_w);
while ~(isempty(a) == 0 && a{1,1}.id == 1 && norm(a{1,1}.pos(1))<=0.45)
    a = duck.april_tags;
end
duck.closeCamera();
duck.openCamera();
pause(3);
% get feature points from duckies
baseImage = imread('duckieontheroad9.jpg');
graybase = rgb2gray(baseImage);
basePoints = detectSURFFeatures(graybase);
[baseFeatures, basePoints] = extractFeatures(graybase, basePoints);

while 1
    duck.sendCmd(0, 0);
    n=921600;h=640;w=480;
    % tic;
    Image_ = duck.getImage();
    
    % get camera image and prepare to output
    Iheader = duck.getImageHeader();
    B = reshape(Image_.data(1:3:n), h, w)';
    G = reshape(Image_.data(2:3:n), h, w)';
    R = reshape(Image_.data(3:3:n), h, w)';
    C = zeros(480, 640, 3, 'uint8');
    C(:,:,1) = R;
    C(:,:,2) = G;
    C(:,:,3) = B;
    
    % get features from input images
    grayinput = rgb2gray(C);
    scenePoints = detectSURFFeatures(grayinput);
    [sceneFeatures, scenePoints] = extractFeatures(grayinput, scenePoints);
    
    % match the pairs
    basePairs = matchFeatures(baseFeatures, sceneFeatures);
    sizeofPairs = size(basePairs);
    if sizeofPairs(1) >= 7
        matchedBasePoints = basePoints(basePairs(:, 1), :);
        matchedScenePoints = scenePoints(basePairs(:, 2), :);
        % drop the noises
        % and try to estimate the angle
        [tform, inlierBasePoints, inlierScenePoints] = ...
            estimateGeometricTransform(matchedBasePoints, matchedScenePoints, 'affine');
        % draw a box for detection
        basePolygon = [1, 1;...                           % top-left
            size(baseImage, 2), 1;...                 % top-right
            size(baseImage, 2), size(baseImage, 1);... % bottom-right
            1, size(baseImage, 1);...                 % bottom-left
            1, 1];                   % top-left again to close the polygon
        newBasePolygon = transformPointsForward(tform, basePolygon);

        % toc;
        disp Detected!!
        duck.sendCmd(0, calib_offset_w);
        pause(1);
        figure(1);
        image(C);
        hold on;
        line(newBasePolygon(:, 1)+40, newBasePolygon(:, 2)+40, 'Color', 'y');
        line(basePolygon(:, 1)-40, basePolygon(:, 2)-40, 'Color', 'y');
        title('Detecting Duckie');
        hold off
    else
        lanefollowing
    end
    % colorbar
end