% calib_offset_w = 0.08;
duck.closeCamera();
duck.openCamera();
% get feature points from duckies
baseImage = imread('duckiesinthesky.jpg');
graybase = rgb2gray(baseImage);
basePoints = detectSURFFeatures(graybase);
[baseFeatures, basePoints] = extractFeatures(graybase, basePoints);

i = 0;
while i<1000
    i = i + 1;
    % duck.sendCmd(0, 0);
    n=921600;h=640;w=480;
    % tic;
    Image_ = duck.getImage();
    
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
    if sizeofPairs(1) >= 11
        matchedBasePoints = basePoints(basePairs(:, 1), :);
        matchedScenePoints = scenePoints(basePairs(:, 2), :);
        % drop the noises, estimate the geometric transform
        [tform, inlierBasePoints, inlierScenePoints] = ...
            estimateGeometricTransform(matchedBasePoints, matchedScenePoints, 'affine');
        % draw a box for detection
        basePolygon = [1, 1;...                           % top-left
            size(baseImage, 2), 1;...                     % top-right
            size(baseImage, 2), size(baseImage, 1);...    % bottom-right
            1, size(baseImage, 1);...                     % bottom-left
            1, 1];                                        % top-left again to close the polygon
        newBasePolygon = transformPointsForward(tform, basePolygon);

        % toc;
        disp Detected!!
        % duck.sendCmd(0, calib_offset_w);
        pause(0.2);
        figure(1);
        image(C);
        hold on;
        line(newBasePolygon(:, 1), newBasePolygon(:, 2), 'Color', 'y');
        line(basePolygon(:, 1), basePolygon(:, 2), 'Color', 'y');
        title('Detecting Duckie');
    else
        % duck.sendCmd(0, calib_offset_w);
        pause(0.01);
        figure(1);
        image(C);
        title('Detecting Duckie');
    end
    % colorbar
end
