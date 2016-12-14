% Camera Inital

% Connect to RR Services
I4 =eye(4);
% Connect to Webcamera and load camera location
cam = RobotRaconteur.Connect('tcp://10.13.215.144:2355/WebcamServer/Webcam');
img = cam.SnapShot();

my_image = reshape(img.data,[480 640 3]);

% Connect to Image Processing Service and load/define parameters
IPH = RobotRaconteur.Connect('tcp://10.13.215.144:34567/ImageProcessingServer/ImageProcessingHost');
IP_ref = IPH.allocateImageProcessor();
IP = IPH.get_imageProcessor(IP_ref);

fc = [731 731]; % Calibration Values
cc = [317 250]; % Calibration Values
K = [fc(1) 0 cc(1) 0 fc(2) cc(2) 0 0 1]';
D = [0.03357   0.44069   -0.00653   -0.00108]';
IP.setALVARMarkerSize(0.1);

% Connect IP to Camera and Enable ALVAR Detection
IP.connectToRobotRaconteurCameraService('tcp://localhost:2355/WebcamServer/Webcam','SimpleWebcam_service.Webcam');
ALVARMultiMarkerBundle = struct('n_markers', int32(1), 'marker_ids', ...
    int32([56]), 'marker_translations', [0; 0; 0], ...
    'marker_orientations', [1; 0; 0; 0], 'marker_sizes', [0.02]);
ALVARMultiMarkerBundle1 = struct('n_markers', int32(1), 'marker_ids', ...
    int32([57]), 'marker_translations', [0; 0; 0], ...
    'marker_orientations', [1; 0; 0; 0], 'marker_sizes', [0.02]);
IP.connectToRobotRaconteurCameraService('tcp://localhost:2355/WebcamServer/Webcam','SimpleWebcam_service.Webcam');
ID = IP.addALVARMultiMarkerBundle(ALVARMultiMarkerBundle);
ID1 = IP.addALVARMultiMarkerBundle(ALVARMultiMarkerBundle1);
IP.setALVARMarkerSize(0.02);
IP.setCameraCalibrationData(K,D);

IP.enableALVARDetection();
