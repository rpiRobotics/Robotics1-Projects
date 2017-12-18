function [ points,M ] = squarepack( M,bin_x,bin_y )
    echo off;
    points=zeros(2,0);
    M=sort(M,2,'descend');
    disp(bin_x);
    disp(bin_y);
    i=1;
    flag = 0;
    while(i<=size(M,2) && flag == 0)
        if(M(i)<=(max(bin_x)-min(bin_x)) && M(i)<=(max(bin_y)-min(bin_y)))
            points=[min(bin_x),min(bin_x)+M(i),min(bin_x),min(bin_x)+M(i);
            min(bin_y),min(bin_y),min(bin_y)+M(i),min(bin_y)+M(i)];
            bin1_x=[min(bin_x)+M(i),max(bin_x)];
            bin1_y=[min(bin_y),min(bin_y)+M(i)];
            bin2_x=[min(bin_x),max(bin_x)];
            bin2_y=[min(bin_y)+M(i),max(bin_y)];
            M(i)=[];
            [points_new, M] = squarepack(M,bin1_x,bin1_y);
            points=[points points_new];
            [points_new, M] = squarepack(M,bin2_x,bin2_y);
            points=[points points_new];
            flag = 1;
        end
        i=i+1;
    end
    echo on;
end