%Exercicio 4
clear
close

ref=imread('imgref.jpeg');
obs=imread('imgobst.jpeg');

ref_b= imbinarize(im2gray(ref),0.8);
obs_b= imbinarize(im2gray(obs),0.8);

subplot(3,2,1), imshow(ref)
subplot(3,2,2), imshow(obs)
subplot(3,2,3), imshow(ref_b)
subplot(3,2,4), imshow(obs_b)

%% Ex5

ref_b_dil= imdilate(ref_b,ones(4,1));%Dilatar para cima
obsbsimple = obs_b;

obsbsimple(ref_b_dil)=0;
obsbsimple= bwareaopen(obsbsimple,10);%limpar


subplot(3,2,5), imshow(ref_b_dil)
subplot(3,2,6), imshow(obsbsimple)

%% Ex 6

 [L,N]=bwlabel(obsbsimple);
 r=regionprops(L,'Centroid');
 cc=cat(1,r.Centroid)';%gravar numa matriz

 hold on
 subplot(3,2,6), plot(cc(1,:),cc(2,:),'*r','MarkerSize',10)

 %% Ex7
 DD=zeros(1,N-1);

 for n=1:N-1
     DD(n)=norm(cc(:,n)-cc(:,n+1));
 end

 idx=find(DD==max(DD))