function pth=path_coords(jpg_file)

% Read and convert
I=imread(jpg_file);
I=rgb2gray(I);

% Resize
prmpt=0;
while prmpt~=1
    imshow(I);
    nsize=input('\nInput scaling factor: ');
    I=imresize(I,nsize);
    imshow(I);
    prmpt=input('Good to proceed? (1 if good): ');
    if prmpt~=1
        I=a;
    end
end

% Binarize and rotate
prmpt=0;
while prmpt~=1
    a=I;
    bino=input('\nInput binarization factor (0.001-0.6): ');
    I=imbinarize(I,'adaptive','ForegroundPolarity','dark','Sensitivity',bino);
    imshow(I);
    prmpt=input('Good to proceed? (1 if good): ');
    if prmpt~=1
        I=a;
    end
end
I=imrotate(I,270);

% Find boundaries and make path
[B,L,N,A]=bwboundaries(I);
prmpt=0;
while prmpt~=1
    cut=input('\nInput plot cutoff value: ');
    ink=input('Incrementation per line: ');
    figure
    z=1;
    for n=2:1:length(B)
        x=B{n}(:,1);
        y=B{n}(:,2);
        x=round(((x-min(B{1}(:,1)))*(260-200))/(max(B{1}(:,1))-min(B{1}(:,1)))+200);
        y=round(((y-min(B{1}(:,2)))*(30-(-30)))/(max(B{1}(:,2))-min(B{1}(:,2)))+(-30));
        pth{:,:,z}=[x y];
        for i=1:ink:length(B{n})
            if length(B{n}) > cut
                [x(i,1) y(i,1)];
                plot(x(i,1),y(i,1),'*')
                hold on 
            end
        end
        z=z+1;
    end
    prmpt=input('Is path acceptable (1 if good): ');
end
end
