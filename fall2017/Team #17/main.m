%% Jordan Charest and Jack Mata
% Robotics 1 Project - Color-controlled duckiebot

%% Setup

% establish the connection
if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.13.215.114:1234/DuckiebotServer.jrcjpmbot/Duckiebot');
end

duck.openCamera();


%% Get first image and display for sanity check before running autonomous control

% Image size
height = 640;
width = 480;
n = 921600;
capture = duck.getImage();

% process image into RGB image format
pixel_array = zeros(width, height, 3, 'uint8');
pixel_array(:,:,1) = reshape(capture.data(1:3:n), height, width)';
pixel_array(:,:,2)= reshape(capture.data(2:3:n), height, width)';
pixel_array(:,:,3)= reshape(capture.data(3:3:n), height, width)';


% % open the image
% figure(100);
% image(pixel_array);
% colorbar;

disp('If everything looks good, choose the mode you wish to use. Options:');
prompt = 'Teleoperation (t) or Autonomous Navigation (a) or Color Control (c): ';
mode = input(prompt, 's');

if mode == 't'
    while 1
        pause(1);
        teleop(duck);
        color_command(duck);
    end

elseif mode == 'a'
    auto_nav(duck);

elseif mode == 'c'
    color_command(duck);

else
    disp('ERROR: unrecognized command. Exiting.');
end

disp ('Program terminated.');
