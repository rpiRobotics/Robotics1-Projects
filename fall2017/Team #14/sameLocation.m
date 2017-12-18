function [ B ] = sameLocation(newCoor,prevCoor)
%This function checks to make sure that the image that was previously taken
%is in the same position still. This allows the user time to move the image
%aroung before a screenshot is taken.
T = 5; 
B = 0;
if newCoor(1) <= prevCoor(1) + T && newCoor(1) >= prevCoor(1) - T
    if newCoor(2) <= prevCoor(2) + T && newCoor(2) >= prevCoor(2) - T
        B = 1;
    end
end

end

