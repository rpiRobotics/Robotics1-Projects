function [BW_filter] = pixelFilter(BW) 
BW_filter = bwareaopen(BW,300);
%imshowpair(BW,BW_filter,'montage')