clear;
close all;

%% connect to robotraconteur
load('connInfo.mat');
mti2d = RobotRaconteur.ConnectService(Mti2dAddr);

% mti2d = RobotRaconteur.Connect(strcat('tcp://localhost:',int2str(PortNum),'/mti2D_RR_interface/MTI2D/'));
figure

Temperature = mti2d.getPropertyValue('Temperature')

setExposureTime(mti2d,25);
setFrequency(mti2d,200);
pause(0.5);
setSignalSelection(mti2d,0);
setIsDoubleSampling(mti2d,0);
% setLaserDeactivated(mti2d,0);


for i = 1:1000
A = mti2d.lineProfile;
len =A.length;
X = A.X_data;
Z = A.Z_data;
I = double(A.I_data);
I_scaled = 125-(I/1024*60); 

plot(X,Z,'.k','MarkerSize',0.1)
hold on
plot(X,I_scaled,'.b','MarkerSize',0.1)
hold off
axis([-35,35,55,130]);
axis ij
grid on
drawnow
pause(0.1);
end


