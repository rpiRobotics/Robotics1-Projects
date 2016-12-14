% Acquire Object Location from Color Thresholding
% Note: this was not implemented in the final design of the project and was
% used for demonstration purposes. Further work is needed to implement
% color thresholding. Not a finsihed project.

clc, clear, close all

cam = webcam;
cam.Resolution = '640x480';
r = 50;
%cam.Resolution = '160x120';
%r = 25;
cam.preview
pause(3)
pics=cell(0:0:0);
i=1;
T = 0;
while true %run infinitly
%while i < 20 % number of images take
  pause(.03); %time step
   RGB = cam.snapshot;
   gRGB = rgb2gray(RGB);
   canny = cannyTest(gRGB);
   
   % Choose Mask
   [BW_Red, maskRed] = createMaskRed(RGB); % red mask
   %[BW_Blue, maskBlue] = createMaskBlue(RGB); %b lue mask
   %[BW_Teal, maskTeal] = createMaskTeal(RGB); % teal-blue mask
   %[BW_Purp, maskPurp] = createPurp(RGB); % purple mask
   %[BW_Duck,maskDuck] = createMaskDuck(RGB); % mask Duck
   %[BW_Duck,maskApple] = createMaskApple(RGB); % mask Apple
   
   BW = mat2gray(BW_Red);
   BW = pixelFilter(BW);
   
   subplot(2,2,1), imshow(RGB)
   %subplot(2,2,2), imshow(gRGB)
   subplot(2,2,3), imshow(canny)
   subplot(2,2,4), imshow(BW)
   
   [c_CT, bb_CT] = BoundBox(BW);
   %[c_ED, bb_ED] = BoundBox(imfill(canny));
   
   if c_CT ~= 0
       
	   % Plot grey scale with object identified
	   subplot(2,2,2)
       [c] = duckDetect(c_CT, bb_CT);
       theta = 0:0.01:2*pi;
       
       imshow(gRGB)
       hold on
       plot(r*cos(theta)+c(1),r*sin(theta)+c(2),'r-',...
           c(1),c(2),'gx')
       
	   [u,v] = idBox(c,r);
       p = [u;v];
       
	   % Attempt to solve PnP problem. Needs work.
	   % If it is desired to test this you will need Peter Corke's RVC Tools
	   % See duckPnp for more info.
%        try
% 		   % Display PnP solution and verify
%            T = duckPnp(p)
%        catch
%            % Keep Running if Pnp cannot be solved
% 		   % Not good practive. Used for development purposes
%        end
   end
   
   %i = i+1;
end