%% Aula 7 Dinˆamica de Manipuladores 
%Trajetorias no espa¸co operacional

% MAGNER GUSSE 110180
%% Ex1 RRR Planar
clear
close all
clc

L1=1; L2=0.8; L3=0.6;%Comprimento dos 3 elos

DH=[0 0 L1 0
    0 0 L2 0
    0 0 L3 0]; %Matriz denavide
robot= SerialLink(DH, 'name', 'RRR robot');

m1=2; m2=1.5; m3=1.0;
masses = [m1,m2,m3];


I(:,:,1)=diag([0.1, 0.1, 0.2 ]);
I(:,:,2)=diag([0.05,0.05,0.1 ]);
I(:,:,3)=diag([0.02,0.02,0.05]);

g=[0 -10 0]';% (m{s2)

comLocal = [L1/2 0 0;
    L2/2 0 0;
    L3/2 0 0]';%Centros de massa locais

for i = 1:robot.n
        robot.links(i).m = masses(i);
        robot.links(i).r = comLocal(:,i);
        robot.links(i).I = I(:,:,i);
end
robot.gravity=g;

% Robo completo, verificar os parametros a partir de robot.dyn na command
% window

%% Ex 2 Dinamica Direta

tdur=6;%Tempo de duração
q_init=[0 0 0];%Posição inicial das juntas
qd_init=[0 0 0];% Velocidade inicial
load('Aula7Ex1e2.mat')
% [t,qtraj,qdtraj]=robot.fdyn(tdur,[],q_init,qd_init);

figure; hold on; grid on; axis equal;
axis([-4 4 -4 4 -1 2])
subplot(1,3,1),plot(t,q_traj), legend({'\theta_1','\theta_2','theta_3',}),grid on;
subplot(1,3,2),plot(t,qd_traj),legend({'\theta_1','\theta_2','theta_3',}),grid on;
subplot(1,3,3) 
robot.plot(q_init,'nobase', ...
    'nowrist', 'noname', 'notiles', ...
    'noshadow', 'linkcolor', 'c', 'delay', 0.05)
pause
robot.animate(q_traj)

%% Ex 3  Funcao para a energia potencial total de um robo
%funcao potential_energy

%% Exercicio 4 Energia potencial

P=zeros(1,size(q_traj,1));

for n=1:size(t)

P(n)=potential_energy(robot,q_traj(n,:));
end

figure
plot(t,P)
xlabel('Time(s)')
ylabel('Potential Energy (J)')

%% Exercicio 5 Energia Cinética
robot.inertia([0 0 0])
