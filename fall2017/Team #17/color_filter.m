function [ BW_raw, BW_filtered] = color_filter( RGB, percent, color )

% convert the RGB image to HSV (Hue-Saturation-Value)
HSV = rgb2hsv(RGB);

% Hue values are passed into the function in the form:
% HueMin = color(1)
% HueMax = color(2)

% Define Saturation thresholds
SaturationMin = 0.000;
SaturationMax = 1.000;

% Define Value thresholds
ValueMin = 0.000;
ValueMax = 1.000;

% filter out colors that don't fall within the defined ranges
BW_raw = ( (HSV(:,:,1) >= color(1)) & (HSV(:,:,1) <= color(2)) ) & ... %  | (HSV(:,:,1) < light_blue(2) )& ...
       (HSV(:,:,2) >= SaturationMin ) & (HSV(:,:,2) <= SaturationMax) & ...
       (HSV(:,:,3) >= ValueMin ) & (HSV(:,:,3) <= ValueMax);
 
% masked_RGB_image = RGB;
% masked_RGB_image(repmat(~BW_raw,[1 1 3])) = 0;
% figure;
% imshowpair(BW_raw, masked_RGB_image, 'montage')
% title ('White: Specified color detected');

BW_size = size(BW_raw);
image_size = BW_size(1) * BW_size(2);


% filter out colors that do not make up more than 2% of the image
BW_filtered = bwareafilt(BW_raw, [image_size*percent image_size]);

end

