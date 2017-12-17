function [ color_detected ] = filter_image( image, connected_percent, color )
% Filter everything not light blue out of the image
% Returns 2 logical arrays: 1 where light blue exists, 0 elsewhere
[BW_raw, BW_refined] = color_filter(image, connected_percent, color);

%% VISUALIZATION: Uncomment for verification
%                 Comment out for speed test
% figure;
% imshowpair(BW_raw, BW_refined, 'montage')
% title ('White: Specified color detected');

%%
empty = true;
BW_size = size(BW_refined);

% If the array is empty, then the specified color was not detected in high
% enough concentration
for i = 1:BW_size(1)
    for j = 1:BW_size(2)
        if BW_refined(i,j) ~= 0
            empty = false;
        end
    end
end

color_detected = ~empty;

% VISUALIZATION
% figure;
% imshowpair(image, BW_raw, 'montage');
% title ('White: Specified color detected');
end

