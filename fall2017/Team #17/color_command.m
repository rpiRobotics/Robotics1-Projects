function color_command( duck )
% The connected percentage of the image that the color has to cover in
% order to not be filtered out
connected_percent = 0.01;

% Search for three colors
yellow = [0.48 0.5];
blue = [0.085 0.11];
green = [0.17 0.22];
purple = [0.88 1];



% Image size
height = 640;
width = 480;
n = height*width*3;


while 1
    robot_view = duck.getImage();

    % process image into RGB image format
    RGB = zeros(480, 640, 3, 'uint8');
    RGB(:,:,1) = reshape(robot_view.data(1:3:n), height, width)';
    RGB(:,:,2)= reshape(robot_view.data(2:3:n), height, width)';
    RGB(:,:,3)= reshape(robot_view.data(3:3:n), height, width)';

    % check for purple
    purple_detected = filter_image(RGB, connected_percent, purple);
    if purple_detected
        disp('Purple Detected. Right.');
        turn_left(duck);
        break;
    end

    % Then blue
    blue_detected = filter_image(RGB, connected_percent, blue);
    if blue_detected
        disp('Blue Detected. Left.');
        turn_right(duck);
        break;
    end
    
    % Then yellow
%     yellow_detected = filter_image(RGB, connected_percent, yellow);
%     if yellow_detected
%         disp('Yellow Detected. Turn Around.');
%         turn_around(duck);
%         break;
%     end
    
    % Then green
    green_detected = filter_image(RGB, connected_percent, green);
    if green_detected
        disp('Green Detected. Straight.');
        drive_straight(duck);
        pause(1);
        duck.sendCmd(0,0);
        break;
    end
       
end

% drive away from the intersection
drive_straight(duck);
pause(0.8);
stop(duck);

end

