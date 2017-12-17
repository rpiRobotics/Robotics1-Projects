%% test color filtering

color = [0.488 0.498];

I = duck.getImage();
% process image into RGB image format
pixel_array = zeros(480, 640, 3, 'uint8');
pixel_array(:,:,1) = reshape(I.data(1:3:n), height, width)';
pixel_array(:,:,2)= reshape(I.data(2:3:n), height, width)';
pixel_array(:,:,3)= reshape(I.data(3:3:n), height, width)';

percent = 0.02;
[BW, BW_filtered] = color_filter(pixel_array, percent, color);

imshowpair(BW,BW_filtered, 'montage');
