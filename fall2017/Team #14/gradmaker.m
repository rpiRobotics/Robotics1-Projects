function gradpts=gradmaker(img,bsz)
%% Read and convert image to greyscale and b/w
I=imread(img);
I=imresize(I,1);
I=rgb2gray(I);
Ibin=imbinarize(I,'adaptive','ForegroundPolarity','dark','Sensitivity',0.1);
Ibin=~Ibin;
%% Gathering edge points
% Get all points that are in white regions on binary occupancy grid (Ibin)
ic=[];
for i=1:size(Ibin,1)
    for n=1:size(Ibin,2)
        if Ibin(i,n)==1
            cd=[i n]*[0 -1;1 0];
            ic=[ic;cd];
        end
    end
end
icn=[];
for i=1:1:size(ic,1)
    icn=[icn;ic(i,1) ic(i,2)];
end

%% Gradients
qd={};
gpts={};
grad={};
it=1;
% bsz determines bin sizes
for i=1:bsz:size(I,1)
    if i+bsz<size(I,1)
        n=1;
        % qd{} is a bin with width bsz and height bsz
        while n+bsz<size(I,2)
            qd{it}=I(i:(i+bsz),n:n+bsz);
            avg=favg(qd{it});
            % Find the average grayscale value and generate plot
            % coordinates
            grad{it}=gtpts(avg,i,i+bsz,n,n+bsz);
            n=n+bsz;
            it=it+1;
            % If the bottom of the image has been reached
            if n+bsz>=size(I,2)
                qd{it}=I(i:(i+bsz),n:size(I,2));
                avg=favg(qd{it});
                grad{it}=gtpts(avg,i,i+bsz,n,size(I,2));
                it=it+1;
            end
        end
    else
        n=1;
        % When the sides of the image have been reached
        while n+bsz<size(I,2)
            qd{it}=I(i:size(I,1),n:n+bsz);
            avg=favg(qd{it});
            grad{it}=gtpts(avg,i,size(I,1),n,n+bsz);
            n=n+bsz;
            it=it+1;
            % If the side and bottom have both been reached
            if n+bsz>=size(I,2)
                qd{it}=I(i:size(I,1),n:size(I,2));
                avg=favg(qd{it});
                grad{it}=gtpts(avg,i,size(I,1),n,size(I,2));
                it=it+1;
            end
        end
    end
end

% Rotate gathered points to be properly orientated and append to the matrix
% that will be returned
% figure
gradpts=[];
for i=1:size(grad,2)
    for n=1:size(grad{i},1)
        c=[grad{i}(n,1) grad{i}(n,2)]*[0 -1;1 0];
        if ~ismember(c,icn,'rows')
%             plot(c(1),c(2),'*')
            gradpts=[gradpts; c(1) c(2)];
%             hold on
        end
    end
end

end