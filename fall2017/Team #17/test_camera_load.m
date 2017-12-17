%% TEST CAMERA LOAD

% This script tests the response time of the camera based on different
% delay times


% establish the connection
if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.13.215.114:1234/DuckiebotServer.jrcjpmbot/Duckiebot');
end

duck.openCamera();


%% LOAD TESTS
n = 921600; % image size
tests = 200;
iterations = 1:1:tests;
no_delay = zeros(tests, 1);
delay_10ms = zeros(tests, 1);
delay_50ms = zeros(tests, 1);
delay_100ms = zeros(tests, 1);
delay_200ms = zeros(tests, 1);
delay_300ms = zeros(tests, 1);
delay_500ms = zeros(tests, 1);

for i = 1:7     % for 6 different pause times
    for j = 1:tests     % run a large number of tests
        
        if j == 1
            fprintf('\nStarting delay test number %d...', i);
        elseif j == tests/2;
            fprintf('halfway done...');
        end
    
        capture_time = tic();
        robot_view = duck.getImage();

        % process image into RGB image format
        RGB = zeros(480, 640, 3, 'uint8');
        RGB(:,:,1) = reshape(robot_view.data(1:3:n), 640, 480)';
        RGB(:,:,2)= reshape(robot_view.data(2:3:n), 640, 480)';
        RGB(:,:,3)= reshape(robot_view.data(3:3:n), 640, 480)';

        end_capture = toc(capture_time);
        
        switch i
            case 1
                no_delay(j) = end_capture;
            case 2
                delay_10ms(j) = end_capture;
                pause(0.01);
            case 3
                delay_50ms(j) = end_capture;
                pause(0.05);
            case 4
                delay_100ms(j) = end_capture;
                pause(0.1);
            case 5
                delay_200ms(j) = end_capture;
                pause(0.2);
            case 6
                delay_300ms(j) = end_capture;
                pause(0.3);
            case 7
                delay_500ms(j) = end_capture;
                pause(0.5);
                
        end
    end
    
    disp('finished.');
end

%% VISUALIZATION

figure;
hold on
 plot(iterations, no_delay, 'linewidth', 1.5);
 plot(iterations, delay_10ms, 'linewidth', 1.5);
 plot(iterations, delay_50ms, 'linewidth', 1.5);
 plot(iterations, delay_100ms, 'linewidth', 1.5);
 plot(iterations, delay_200ms, 'linewidth', 1.5);
 plot(iterations, delay_300ms, 'linewidth', 1.5);
 plot(iterations, delay_500ms, 'linewidth', 1.5);
 title('Camera Load Test');
 ylabel('Execution Time - s');
 xlabel('# of Iterations');
 legend('No Delay', '10ms Delay', '50ms Delay', '100ms Delay', '200ms Delay', '300ms Delay', '500ms Delay');
 

means = [mean(no_delay); mean(delay_10ms); mean(delay_50ms); mean(delay_100ms);
         mean(delay_200ms); mean(delay_300ms); mean(delay_500ms)];

delay_times = categorical({'1 - None','2 - 10 ms','3 - 50 ms', '4 - 100 ms', '5 - 200 ms', '6 - 300 ms', '7 - 500 ms'});


figure;
hold on
 bar(delay_times, means);
 ylabel('Capture Time - s');
 title('Image Capture Performance');
 

    