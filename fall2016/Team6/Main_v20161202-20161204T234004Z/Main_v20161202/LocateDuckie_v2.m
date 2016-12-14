function [XCenter, XRadii, Xsub,OriDuck] = LocateDuckie_v2(img,D)
%%% 11/07/2016
%%% INPUT=================
%%% img:         Input image
%%% D:             Resolution decimation
%%% OUTPUT=================
%%% XCenter:  Center of the duck
%%% XRadii:     Radius that circle the duck around the center
%%% XColor:    RGB average color in that circle



load('imgBG.mat');

[m,n,d] = size(img);

img_Sub = img -imgBG;
img_Sub = img_Sub(1:D:end,1:D:end,:);

% Applying Filter 
A1 = squeeze(img_Sub(:,:,1));
A2 = squeeze(img_Sub(:,:,2));
A3 = squeeze(img_Sub(:,:,3));

A1_tmp =A1;
A2_tmp =A2;
A3_tmp =A3;

[mD, nD] = size(A1_tmp);

for i = 3:mD-2
    for j=3:nD-2
        A1_tmp(i,j) = min(min(A1(i-2:i+2,j-2:j+2)));
        A2_tmp(i,j) = min(min(A2(i-2:i+2,j-2:j+2)));
        A3_tmp(i,j) = min(min(A3(i-2:i+2,j-2:j+2)));
    end  
end

% A1=A1_tmp;
% A2=A2_tmp;
% A3=A3_tmp;
% B(:,:,1) = A1_tmp;
% B(:,:,2) = A2_tmp;
% B(:,:,3) = A3_tmp;
% for i = 3:m-2
%     for j=3:n-2
%         A1_tmp(i,j) = max(max(A1(i-2:i+2,j-2:j+2)));
%         A2_tmp(i,j) = max(max(A2(i-2:i+2,j-2:j+2)));
%         A3_tmp(i,j) = max(max(A3(i-2:i+2,j-2:j+2)));
%         
%     end
% end

A(:,:,1) = A1_tmp;
A(:,:,2) = A2_tmp;
A(:,:,3) = A3_tmp;


% Group Labeling
[X, Xnum]=(bwlabel((A(:,:,1)>20).*(A(:,:,3)<5)));

Cnt=zeros(Xnum,1);
Xmass=zeros(Xnum,2);
X1=max(m,n)*ones(Xnum,2);
X2=zeros(Xnum,2);
XColor=zeros(Xnum, 3);


for i = 1:mD
    for j=1:nD
        if(X(i,j)~=0)
            
            Cnt(X(i,j)) = Cnt(X(i,j))+1;
            Xmass(X(i,j),1) = Xmass(X(i,j),1)+i;
            Xmass(X(i,j),2) = Xmass(X(i,j),2)+j;
            X1(X(i,j),1) = min(X1(X(i,j),1),i);
            X1(X(i,j),2) = min(X1(X(i,j),2),j);
            X2(X(i,j),1) = max(X2(X(i,j),1),i);
            X2(X(i,j),2) = max(X2(X(i,j),2),j);
            XColor(X(i,j),1) = XColor(X(i,j),1) + int32(img(D*i-1,D*j-1,1));
            XColor(X(i,j),2) = XColor(X(i,j),2) + int32(img(D*i-1,D*j-1,2));
            XColor(X(i,j),3) = XColor(X(i,j),3) + int32(img(D*i-1,D*j-1,3));
            
        end
    end
end

XColor(:,1) = XColor(:,1)./Cnt(:);
XColor(:,2) = XColor(:,2)./Cnt(:);
XColor(:,3) = XColor(:,3)./Cnt(:);

XCenter = [ Xmass(:,2)./Cnt(:), Xmass(:,1)./Cnt(:)];
XCenter = D*(XCenter-1);
XRadii = max(abs(X1(:,1)-X2(:,1)),abs(X1(:,2)-X2(:,2))/2 )+1;
XRadii = D*XRadii;
XArea = D*D*Cnt(:);

% i = 1;
% while(i<=length(XRadii))
%     if(XRadii(i)<10)
%         XCenter(i,:) = [];
%         XRadii(i) = [];
%         XArea(i) = [];     
%         XColor(i,:) = [];
%         i=i-1;
%     elseif(sum(XColor(i,:))<150)
%         XCenter(i,:) = [];
%         XRadii(i) = [];
%         XArea(i) = [];     
%         XColor(i,:) = [];
%         i=i-1;
%     end   
%     i=i+1;
% end

% sol =[    0.0098    0.0070   -0.1708]';
% b =   10.6381;
sol =[   -0.0200    0.0489   -0.0269]';
b =  -2.4496;

Xid = find((XColor*sol+b)==max((XColor*sol+b)));

% Xid = Xid(find(XRadii(Xid)>=5));

XCenter = XCenter(Xid,:);
% XRadii = XRadii(Xid);
XColor = XColor(Xid,:);

Xsearch_start = fix(XCenter-50);
Xsearch_stop = fix(XCenter+50);
if(Xsearch_start(1)<1) Xsearch_start(1)=1; end
if(Xsearch_start(2)<1) Xsearch_start(2)=1; end
if(Xsearch_stop(1)>n) Xsearch_stop(1)=n; end
if(Xsearch_stop(2)>m) Xsearch_stop(2)=m; end
Xsub = (img(Xsearch_start(2):Xsearch_stop(2), Xsearch_start(1):Xsearch_stop(1),3));
XRadii = sqrt(length(find(Xsub==0)));
% figure(100);
% imshow(Xsub);


%%% Duckie Orientation
edgeIm = sobel(double(img(Xsearch_start(2):Xsearch_stop(2), Xsearch_start(1):Xsearch_stop(1),3)), 0.3);
[Edge_x,Edge_y] = find(edgeIm(:,:)==0);
OriDuck = pca([Edge_x,Edge_y]');



end
