function [ T ] = arDetect( IP,  ID )
% artagging
%   Generate homogeneous transform for AR Tag
T = IP.getCameraPoseFromMultiMarker(int32(ID));
T = [T(1:4)'; T(5:8)'; T(9:12)'; T(13:16)'];
end

