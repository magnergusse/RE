%% Aula 14
%MAgner Gusse 110180

%% Ex1
clc
clear
close all


%a
load lidarScans.mat


s=lidarScans(250);

% b
% Representar o scan 250 usando as coordenadas
% cartesianas com eixos monom´etricos, uma grelha de
% fundo e nomes das coordenadas ‘X’ e ’Y’:

sCart=s.Cartesian;
plot(sCart(:,1), sCart(:,2),'.k');
axis equal; grid on
xlabel('X'); ylabel('Y')


% Verificar, por adi¸c˜ao no gr´afico anterior, que os
% dados que est˜ao nas vari´aveis polares s˜ao os
% mesmos:

[x,y]=pol2cart(s.Angles,s.Ranges);%angulos tem quee ser em radianos
hold on
plot(x, y,'or');

%c
% Uma forma alternativa de representar o gr´afico ´e a de usar o m´etodo plot de lidarScan:
figure
s.plot; % or plot(s)


%% Ex 2
%a & b
robot=[ 0 0.5 0
-0.2 0 0.2];


% Para facilitar a visualiza¸c˜ao do pol´ıgono de espa¸co livre detetado pelo LIDAR, sobrepor
% nos pontos de scan um pol´ıgono fechado cujo primeiro ponto ´e o centro do robˆo (0,0).

scan = lidarScans(1);
scanCart=scan.Cartesian;

figure
h=scan.plot; h.MarkerSize=20; %LiDAR scan points
hold on; axis equal; grid on; axis([-8 8 -8 8]);


hp=fill([0; scanCart(:,1)], [0; scanCart(:,2)], 'y'); %polygon
wall_e=fill(robot(1,:), robot(2,:),'r'); %The robot


for n=1:numel(lidarScans)
        scan = lidarScans(n);
        scanCart=scan.Cartesian;
        set(h, 'XData',scanCart(:,1),'YData',scanCart(:,2)) %update LiDAR plot
        set(hp,'XData',[0; scanCart(:,1)],'YData',[0; scanCart(:,2)]) %update polygon plot
        
        minRange = 0.15;
        maxRange = 8;
        scan = removeInvalidData(scan,...
        'RangeLimits',[minRange maxRange]);

        
        
        pause(0.04)
        
end


%% Ex 3
%a
clc
clear
close all

load lidarScans.mat

s=lidarScans(250);
robot=[ 0 0.5 0
-0.2 0 0.2];
scan = lidarScans(1);
scanCart=scan.Cartesian;


% Para facilitar a visualiza¸c˜ao do pol´ıgono de espa¸co livre detetado pelo LIDAR, sobrepor
% nos pontos de scan um pol´ıgono fechado cujo primeiro ponto ´e o centro do robˆo (0,0).

scan = lidarScans(1);
scanCart=scan.Cartesian;

figure
h2=scan.plot; h2.MarkerSize=10;h2.MarkerEdgeColor='b'; %LiDAR scan points
hold on; axis equal; grid on; axis([-8 8 -8 8]);

h1=scan.plot; h1.MarkerSize=10; h1.MarkerEdgeColor='g'; %LiDAR scan points
hold on
h3=scan.plot; h1.MarkerSize=10; h1.MarkerEdgeColor='k'; %LiDAR scan points

wall_e=fill(robot(1,:), robot(2,:),'r'); %The robot

step=1;
start=1; %first scan to analyse
numScans = numel(lidarScans);

        for i=step+start:numScans
                refScan = lidarScans(i-1);
                currScan = lidarScans(i);
                prevScan = lidarScans(i-step); % filter and update the 2 scans
                %now clear
            minRange = 0.15;
            maxRange = 8;
            refScan = removeInvalidData(refScan,'RangeLimits',[minRange maxRange]);
            currScan = removeInvalidData(currScan,'RangeLimits',[minRange maxRange]);


            scanCartref=refScan.Cartesian;
            set(h2, 'XData',scanCartref(:,1),'YData',scanCartref(:,2)) %update LiDAR plot

            scanCartcurr=currScan.Cartesian;
            set(h1, 'XData',scanCartcurr(:,1),'YData',scanCartcurr(:,2)) %update LiDAR plot
            
            pose = matchScans(currScan,prevScan);
            estScan=transformScan(prevScan,pose);
            estcart=estScan.Cartesian;
            set(h3,'XData',estcart(:,1),'YData',estcart(:,2))

                title("LiDAR Scan #"+i+" and #"+(i-step))
                pause(0.04)
        end


%% Ex 4

%a
clc
clear
close all

load lidarScans.mat

s=lidarScans(250);
robot=[ 0 0.5 0
-0.2 0 0.2];
scan = lidarScans(1);
scanCart=scan.Cartesian;


% Para facilitar a visualiza¸c˜ao do pol´ıgono de espa¸co livre detetado pelo LIDAR, sobrepor
% nos pontos de scan um pol´ıgono fechado cujo primeiro ponto ´e o centro do robˆo (0,0).

scan = lidarScans(1);
scanCart=scan.Cartesian;

figure
h2=scan.plot; h2.MarkerSize=10;h2.MarkerEdgeColor='b'; %LiDAR scan points
hold on; axis equal; grid on; axis([-8 8 -8 8]);

h1=scan.plot; h1.MarkerSize=10; h1.MarkerEdgeColor='g'; %LiDAR scan points
hold on
h3=scan.plot; h1.MarkerSize=10; h1.MarkerEdgeColor='k'; %LiDAR scan points

wall_e=fill(robot(1,:), robot(2,:),'r'); %The robot

step=1;
start=1; %first scan to analyse
numScans = numel(lidarScans);


initialPose = [0 0 0]; %initial estimate of pose
poseList = zeros(numScans,3); %empty list to calculate poses
poseList(1,:) = initialPose;



        for i=step+start:numScans
                refScan = lidarScans(i-1);
                currScan = lidarScans(i);
                prevScan = lidarScans(i-step); % filter and update the 2 scans
                %now clear
            minRange = 0.15;
            maxRange = 8;
            refScan = removeInvalidData(refScan,'RangeLimits',[minRange maxRange]);
            currScan = removeInvalidData(currScan,'RangeLimits',[minRange maxRange]);


            scanCartref=refScan.Cartesian;
            set(h2, 'XData',scanCartref(:,1),'YData',scanCartref(:,2)) %update LiDAR plot

            scanCartcurr=currScan.Cartesian;
            set(h1, 'XData',scanCartcurr(:,1),'YData',scanCartcurr(:,2)) %update LiDAR plot
            
            pose = matchScans(currScan,prevScan);
            estScan=transformScan(prevScan,pose);
            estcart=estScan.Cartesian;
            set(h3,'XData',estcart(:,1),'YData',estcart(:,2))


            pose = matchScans(currScan,refScan);
            poseList(i,:)= pose;
                title("LiDAR Scan #"+i+" and #"+(i-step))
                pause(0.04)
        end
[allT,allPoses]=accumulatedPoses(poseList);

%Pontos percorridos
figure
hold on, grid on, axis([-8 8 -8 8]);

for n=1:10:size(allT,3)
    robotn=allT(:,:,n)*[robot;1,1,1];
    fill(robotn(1,:), robotn(2,:),'c'); %The robot

end


%% Ex 5-mpode ajudar no tp2

map = occupancyMap(15,15,20);
map.GridLocationInWorld = [-7.5 -7.5];


for n=2:numScans
currentScan = lidarScans(n);
absolutePose=allPoses(n,:);
insertRay(map, absolutePose, currentScan, 10);

end

figure
show(map)
hold on
plot(allPoses(:,1),allPoses(:,2),'bo','DisplayName','Estimatedposition');