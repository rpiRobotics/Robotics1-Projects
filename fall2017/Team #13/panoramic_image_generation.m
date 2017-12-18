n=921600; h = 640;w = 480;
duck.openCamera();
pause(1)
I1=duck.getImage();
C =zeros(480,640,3,'uint8');
C(:,:,3)=reshape(I1.data(1:3:n),h,w)';
C(:,:,2)=reshape(I1.data(2:3:n),h,w)';
C(:,:,1)=reshape(I1.data(3:3:n),h,w)';
imwrite(C,'1.jpg')
movefile('1.jpg','C:/Users/Tao/OneDrive/School/Robotics/project/pic');
for i = 2:12
    duck.sendCmd(0,-3);%rotate about itself
    pause(0.5)
    duck.sendCmd(0,0);
    pause(1)
    Ii=duck.getImage();
    C(:,:,3)=reshape(Ii.data(1:3:n),h,w)';
    C(:,:,2)=reshape(Ii.data(2:3:n),h,w)';
    C(:,:,1)=reshape(Ii.data(3:3:n),h,w)';
    %figure(i);
    chr = int2str(i);
    string=[chr,'.jpg'];
    imwrite(C,string);
    movefile(string,'C:/Users/Tao/OneDrive/School/Robotics/project/pic');
end
duck.closeCamera();
run('undistorted.m');
load('cam.mat');
images = imageDatastore(fullfile('C:/Users/Tao/OneDrive/School/Robotics/project/pic'));
for i = 1:12
    I = images.readimage(i);
    Ji = undistortImage(I,cameraParams);
    Ki = undistortImage(I,cameraParams,'OutputView','full');
    chr = int2str(i);
    string=[chr,'.jpg'];
    imwrite(Ji,string);
    movefile(string,'C:/Users/Tao/OneDrive/School/Robotics/project/undistorted');
    image(I);
end
pictures = imageDatastore(fullfile('C:/Users/Tao/OneDrive/School/Robotics/project/undistorted'));
 
I = readimage(pictures, 1);
 
grayImage = rgb2gray(I);
points = detectSURFFeatures(grayImage);
[features, points] = extractFeatures(grayImage, points);
 
numImages = numel(pictures.Files);
tforms(numImages) = projective2d(eye(3));
load tf.mat
trans = tf.T;
 
for n = 2:numImages
    tforms(n).T = trans * tforms(n-1).T;
end
 
imageSize = size(I);  % all the images are the same size
 
% Compute the output limits  for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);    
end
 
centerImageIdx = (numImages+1)/2;
 
 
% Apply the center image's inverse transform to all the others.
 
Tinv = invert(tforms(centerImageIdx));
 
for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end
 
 
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end
 
% Find the minimum and maximum output limits
xMin = min([1; xlim(:)]);
xMax = max([imageSize(2); xlim(:)]);
 
yMin = min([1; ylim(:)]);
yMax = max([imageSize(1); ylim(:)]);
 
% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);
 
% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', I);
 
blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);
 
% Create the panorama.
for i = 1:numImages
    
    I = readimage(pictures, i);   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end
 
figure
imshow(panorama)
imwrite(panorama,'pano1.jpg')
