function [Points] = findArea(mainImage)
%This function is similar to ImagePresent but it used instead to find the
%drawing region. The method to do this is the same, but the results need to
%be filtered. There are multiple regions of correlation but the knowledge
%of only three uniqe points is necessary
imshow(mainImage)
hold on
searchedImage = imread('oneplus.jpg');%read image, this image is default
%perform image conversion on both
imS = rgb2gray(mainImage);
imL = rgb2gray(searchedImage);
%perform cross correlation on images
C = normxcorr2(imL,imS);
C = abs(C);
C = C>=.8;
num=1;
%find size of correlation regions
[L1 L2] = size(imL)
allP = [];
while num==1
    [num index] = max(C(:));
    if num==0
        break;
    end
    [s,s2] = ind2sub(size(C),index);
    p = [s2-102 s-102 L2 L1];
    rectangle('Position',p,'EdgeColor','r')
    %Store each correlation point in a ultimate area
    allP = [allP; p];
    p = [];
    C(s,s2)=0;
end

%First remove any points that are exactly the same.
allP = unique(allP,'rows');
newArea = [];
%For each point check if the point +-20 is in the set newArea. If it is,
%skip it, if it isn't place it in the set.
for k=1:length(allP)
    curV = allP(k,:);
    found = 0; 
    for j=0:40
        valx = j - 20;
        for i = 0:40
            valy = i - 20;
            member = ismember([curV(1)+valx curV(2)+valy],newArea); 
            if (member(1) && member(2))
                found = 1;
                break;
            end
        end
        if found == 1
            break;
        end
    end
    if found == 0
        newArea = [newArea; [curV(1) curV(2)]]; 
    end
end

%shift the points to return the centers
for k=1:size(newArea,1)
    newArea(k,1) = newArea(k,1) + L1/2;
    newArea(k,2) = newArea(k,2) + L2/2;
end
%return unique points
Points = newArea;

end


