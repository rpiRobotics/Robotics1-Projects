%% TEST IMAGE SIZE vs. COMPUTATIONAL LOAD
% Written By: Jordan Charest & Jack Mata

% This script is intended to test the computation time needed to filter
% images as a function of the image size, in an attempt to find an optimal
% image size for autonomous navigation

%% Setup

clear
clc

if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.13.215.114:1234/DuckiebotServer.jrcjpmbot/Duckiebot');
end

duck.openCamera();

% The connected percentage of the image that the color has to cover in
% order to not be filtered out
connected_percent = 0.02;

% Search for three colors
purple = [0.76 0.84];
light_blue = [0.09 0.105];
pink = [0.65 0.70];     % purple and pink are too close together, pick another color (green?)


%% Test
n = 921600;
tests = 200;

false_positives = 0;
false_negatives = 0;

base_height = 320;
base_width = 240;

hits = zeros(101,1);
misses = zeros(101,1);
captures = zeros(101,1);

image_size = zeros(101,1);
iterations = 0:1:tests;


for i = 0:tests
    clear RGB RGB_resized
    
    height = base_height + 20*i;
    width = height * 0.75; 
    
    % For data visualization
    image_size(i+1) = width * height;

    capture_time = tic();
    robot_view = duck.getImage();

    % process image into RGB image format
    RGB = zeros(480, 640, 3, 'uint8');
    RGB(:,:,1) = reshape(robot_view.data(1:3:n), 640, 480)';
    RGB(:,:,2)= reshape(robot_view.data(2:3:n), 640, 480)';
    RGB(:,:,3)= reshape(robot_view.data(3:3:n), 640, 480)';
    
    end_capture = toc(capture_time);
    captures(i+1) = end_capture;
    
    RGB_resized = imresize(RGB, [width, height]);
    
%     % display a couple captures
%     if i == 0 || i == 50 || i == 100
%         figure;
%         image(RGB_resized);
%         colorbar;
%     end
    
    % First compute a hit
    start = tic();
    purple_detected = filter_image(RGB_resized, connected_percent, purple);
    finish = toc(start);
    
    if purple_detected
        hits(i+1) = finish;
    else
        fprintf('False Negative, Iteration: %d', i);
        false_negatives = false_negatives + 1;
    end

    % Then compute a miss
    start = tic();
    blue_detected = filter_image(RGB_resized, connected_percent, light_blue);
    finish = toc(start);
    if ~blue_detected
        misses(i+1) = finish;
    else
        fprintf('False Positive, Iteration: %d', i);
        false_positives = false_positives + 1;
    end
    
    fprintf('\nLoop %d completed.', i);

end
figure;
hold on
 plot(image_size, hits, 'linewidth', 1.5);
 plot(image_size, misses, 'linewidth', 1.5);
 title('Color Filtering Performance');
 ylabel('Execution Time - s');
 xlabel('Image Size - # of pixels');
 legend('Hits', 'Misses');
 
 
figure;
 plot(iterations, captures);
 title('Camera Capture Performance');
 ylabel('Execution Time - s');
 xlabel('Iteration');
 
 
 fprintf('False Negatives: %d', false_negatives);
 fprintf('False Positives: %d', false_positives);