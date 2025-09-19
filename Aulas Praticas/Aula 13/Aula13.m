%% Aula 13
%MAgner Gusse 110180
%% Ex1
clc
close all
clear 

%medidas do mapa
WW=150; %Width (# of columns)
HH=100; %Height (# of rows)


%Posi¸c˜ao dos obst´aculos:
PP =[
40 55 65 %X
80 35 20 %Y
];

%Raio dos obst´aculos
r=4; %radius of obstacles

X = 1:WW; % span of X coordinate
Y = 1:HH; % span of Y coordinate
[XX, YY] = meshgrid(X, Y);


map = zeros(HH, WW); %initial empty map
% For every obstacle in the PP matrix,
% fill appropriately the map with values "1"
for k=1:size(PP,2)
XX0 = XX - PP(1,k);
YY0 = YY - PP(2,k);
map((XX0.^2 + YY0.^2) <= r^2) = 1;
end

imshow(map), axis xy
xlabel('X'), ylabel('Y')

%% Ex 2

Dmax = 20; %distancia maxima de influencia do obstaculo, mais do que isso nao afeta
krep = 30; %proportional repeller potential constant


[yobst,xobst]=find(map); %x and y coordinates of all obstacles (cells)
Urep=zeros(size(map)); %initial empty potential map


for k=1:numel(xobst) %check all obstacle cells one by one
            maskDmax = zeros(size(map)); %initial empty mask the same size of the map
            
            allDistsToObst=sqrt((XX-xobst(k)).^2 + (YY-yobst(k)).^2); % All distances to current obstacle
            
            maskDmax(allDistsToObst <= Dmax) = 1; % Mask to filter the points to be affected
            % tudo que for maior que dmax não sao consideradas
            
            Urep_parcial= 0.5*krep*(1./(allDistsToObst) - 1/(Dmax)).^2; %the rep potential for all points on map
            
            Urep_parcial(~maskDmax)=0; %Apply mask to cancel influence on distant points
            
            Urep=Urep+Urep_parcial;%Update repeller potential map for this obstacle
end

    Urep = min(Urep,50); % Limit the infinites to a reasonable value
    
    %Visualizar o campo repulsivo
    %surf(XX,YY,Urep); colormap("cool"); xlabel('X'), ylabel('Y'), axis equal

    %% Ex 3
    T = [120 50]';
    katt = 0.005;
    
    Uatt=0.5*katt*((XX-T(1)).^2+(YY-T(2)).^2);

    %Visualizar o campo atrativo
    %surf(XX,YY,Uatt); colormap("cool"); xlabel('X'), ylabel('Y'), axis equal

    Utot= Uatt+Urep;

    %Visualizar o campo de potenciais total
    surf(XX,YY,Utot); colormap("colorcube"); xlabel('X'), ylabel('Y'), axis equal

    %% Ex4 Gradientes descendentes no campo de potencial

    [Gx,Gy]= gradient(Utot);
    Gx=-Gx;
    Gy=-Gy;

    figure
    quiver(XX,YY, Gx, Gy,10)
    xlabel('X'), ylabel('Y'), axis equal;
    hold on
    plot(T(1),T(2),'.r','MarkerSize',25)


    %% EX 5 & 6
    figure
    contour(XX,YY, Utot, 100)% 100 numero de curvas a traçar
    hold on
    plot(T(1),T(2),'.r','MarkerSize',25)
    hold on
    sx=15;
    sy=40;
    streamline(stream2(X,Y,Gx,Gy,sx,sy),'Color','r');

    %% Ex 7

    R=110;
    theta=deg2rad(155:3:205);
    [xs,ys]=pol2cart(theta,R);
    hold on
    h= streamline(stream2(X,Y,Gx,Gy,xs+T(1),ys+T(2)),'Color','magenta');
     plot(xs+T(1),ys+T(2),'ro')
%% Ex 8

    B = ordfilt2(Utot, 1, ones(3,3));
    [ym,xm] = find(B == Utot);
    plot(xm,ym,'r*','MarkerSize',20)

%% EX 9
