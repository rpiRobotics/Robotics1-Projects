function [B] = ImagePresent(mainImage,searchedImage)
%This function searches for the image the user submits. Specifically, it
%will look for the xy-axis and report these coordinates
B = [0 0 0];
%Convert the main image to grayscale
imS = rgb2gray(mainImage);
%Convert the searched image to grayscale
imL = rgb2gray(searchedImage);
%Display the image so that the location of image can be found
imshow(mainImage)
hold on
C = normxcorr2(imL,imS); %Find correlation between two images
C = abs(C);
C = C>=.8;
num=1;
itrs = 0;
[L1 L2] = size(imL);
while num==1
    [num index] = max(C(:));
    if num==0
        break;
    end
    [s,s2] = ind2sub(size(C),index);
    p = [s2-120 s-115 L2 L1];
    rectangle('Position',p,'EdgeColor','r')
    C(s,s2)=0;
    itrs = itrs + 1;
    disp('image found')
    %Find the center of the rectangle
    cx = p(1)+p(3)/2;
    cy = p(2)+p(4)/2;
    B = [itrs cx cy];
end

end
