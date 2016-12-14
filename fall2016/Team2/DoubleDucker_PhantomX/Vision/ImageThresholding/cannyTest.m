function [canny2] = cannyTest(z)

step_size = 0.01;
edge_threshold_low = 0.0;
edge_threshold_high = 0.2;
sens_low = edge_threshold_low + step_size;
sens_high = edge_threshold_high + step_size;
sen = [sens_low, sens_high];
sigma = 1;

canny2 = edge(z,'canny',sen, sigma);
% subplot(2,1,1)
% test = canny2;
% imshow(test)
% subplot(2,1,2)
% test2 = (imfill(canny2,'holes'));
% imshow(test2)


% imageRegionAnalyzer
% image porcessing apps