%% TESTING MULTIPLE COLOR FILTERS
% Written By: Jordan Charest & Jack Mata

% This script is intended to test the accuracy and speed of the
% filter_image() function, which invokes the color_filter() function to
% filter all colors out of an image except those that lie in a narrow color
% band

% USAGE: filter_image(arg1, arg2, arg3)
% Filter the image:
% arg1: Image to be filtered
% arg2: The connected percentage of the image that the color has to cover in
%        order to not be filtered out
% arg3: Color to be filtered out


%% Test Setup
clear
clc

% The connected percentage of the image that the color has to cover in
% order to not be filtered out
connected_percent = 0.02;

% Testing three colors
I_purple = imread('purple.tif');
purple = [0.76 0.84];

I_blue = imread('light_blue.tif');
light_blue = [0.09 0.105];

I_pink = imread('red_and_pink.tif');
pink = [0.65 0.70];     % purple and pink are too close together, pick another color (green?)


%%  Test if we can read purple
fprintf('Testing "Hits" - The following three tests should be successful\n\n')

purple_start = tic();
color_detected = filter_image(I_purple, connected_percent, purple);
purple_time = toc(purple_start);

if color_detected
    disp('SUCCESS: This image contains a purple sign.');
else
    disp('FAIL: Purple was not detected in this image.');
end


%% Test if we can read light blue

blue_start = tic();
color_detected = filter_image(I_blue, connected_percent, light_blue);
blue_time = toc(blue_start);

if color_detected
    disp('SUCCESS: This image contains a light blue sign.');
else
    disp('FAIL: Light blue not detected.');
end


%% Test if we can read pink - may need to print out a clearer image

pink_start = tic();
color_detected = filter_image(I_pink, connected_percent, pink);
pink_time = toc(pink_start);


if color_detected
    disp('SUCCESS: This image contains a pink sign.');
else
    disp('FAIL: Pink not detected.');
end


%% Test for misses
fprintf('\n\nTesting "Misses" - the following six tests should fail to detect a color\n');
% Test for failures to read purple

purple_start = tic();
color_detected = filter_image(I_blue, connected_percent, purple);
purple_miss_1 = toc(purple_start);

if color_detected
    disp('SUCCESS: This image contains a blue sign.');
else
    disp('FAIL: Blue was not detected in this image.');
end

purple_start = tic();
color_detected = filter_image(I_pink, connected_percent, purple);
purple_miss_2 = toc(purple_start);

if color_detected
    disp('SUCCESS: This image contains a pink sign.');
else
    disp('FAIL: Pink was not detected in this image.');
end


%% Test for failures to read light blue
blue_start = tic();
color_detected = filter_image(I_purple, connected_percent, light_blue);
blue_miss_1 = toc(blue_start);

if color_detected
    disp('SUCCESS: This image contains a purple sign.');
else
    disp('FAIL: Purple was not detected in this image.');
end

blue_start = tic();
color_detected = filter_image(I_pink, connected_percent, light_blue);
blue_miss_2 = toc(blue_start);

if color_detected
    disp('SUCCESS: This image contains a pink sign.');
else
    disp('FAIL: Pink was not detected in this image.');
end

%% Test for failure to read pink
pink_start = tic();
color_detected = filter_image(I_blue, connected_percent, pink);
pink_miss_1 = toc(pink_start);

if color_detected
    disp('SUCCESS: This image contains a blue sign.');
else
    disp('FAIL: Blue was not detected in this image.');
end


pink_start = tic();
color_detected = filter_image(I_purple, connected_percent, pink);
pink_miss_2 = toc(pink_start);

if color_detected
    disp('SUCCESS: This image contains a purple sign.');
else
    disp('FAIL: Purple was not detected in this image.');
end


%% Summary
fprintf('\n\nHIT STATISTICS:');
fprintf('\nPurple Test completed in: %8.5f seconds', purple_time);
fprintf('\nBlue Test completed in:   %8.5f seconds', blue_time);
fprintf('\nPink Test completed in:   %8.5f seconds', pink_time);

fprintf('\n\nMISS STATISTICS:');
fprintf('\nTwo purple miss tests completed in: %8.5f seconds and %8.5f seconds', purple_miss_1, purple_miss_2);
fprintf('\nTwo blue miss tests completed in:   %8.5f seconds and %8.5f seconds', blue_miss_1, blue_miss_2);
fprintf('\nTwo pink miss tests completed in:   %8.5f seconds and %8.5f seconds', pink_miss_1, pink_miss_2);



hit_average = (purple_time + blue_time + pink_time)/3;
miss_average = (purple_miss_1 + purple_miss_2 + blue_miss_1 + blue_miss_2 + pink_miss_1 + pink_miss_2) / 6;
fprintf('\n\nAverage Calculation Time for HITS:    %8.5f seconds', hit_average);
fprintf('\nAverage Calculation Time for MISSES:  %8.5f seconds\n', miss_average);


