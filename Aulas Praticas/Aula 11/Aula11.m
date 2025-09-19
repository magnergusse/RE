%% Aula 11
% Magner Gusse 110180
% Robótica Espacial
%% Ex1
close all
clear
clc

cam=webcam("Logi C270 HD WebCam");
%preview(cam)


subplot(1,2,1)
img= snapshot(cam);
h=imshow(img);

subplot(1,2,2)
h2=imshow(im2gray(img));

XX=[];
YY=[];


while 1
    
img= snapshot(cam);
img= fliplr(img);
% h.CData = img;
subplot(1,2,1); imshow(img);


gray=im2gray(img);
T=0.85;
b=imbinarize(gray,T);
%h2.CData = b;

b=bwareaopen(b,5000);
b=bwmorph(b,'erode',2);
subplot(1,2,2), imshow(b)

[L,N]= bwlabel(b);
r=regionprops(L,'Centroid');

if numel(r)>0
r(1).Centroid;
x=r(1).Centroid(1);
y=r(1).Centroid(2);
subplot(1,2,1)
hold on
plot(x,y,'*b','MarkerSize',20);
XX=[XX x];
YY=[YY y];
plot(XX,YY,'-r')

end


end