%% Aula 2 Cinematica Direta

% MAGNER GUSSE
%% Exercicio 1
clear all;
close all;
clc;

% Comprimentos dos elos
L1=2;
L2=1;

%Definir os elos com juntas rotacionais, 'theta' é variavel então nao se poe nada
L(1) = Link('a', L1, 'd', 0, 'alpha', 0);
L(2) = Link('a', L2, 'd', 0, 'alpha', 0);

% Criar Robo planar 
RR_robot = SerialLink(L, 'name', 'Robo RR');

% [0,0] é a posição inicial, depois limites do plot e o delay para ver
% melhor os frames 
figure('WindowStyle','docked')
RR_robot.plot([0 0], 'workspace', [-4 4 -4 4 -1 1],'delay',0)
hold on
%% Exercicio 2 definir caminho

% posiçoes inicial e final em rad das juntas
    qi=[0,0];
    qf=[pi/4, pi/2];

nsteps = 100;
qtraj = jtraj(qi, qf, nsteps); %calcula a trajetoria polinomial

% % segunda figura
% figure('WindowStyle','normal')
% subplot(2,1,1)
% plot(qtraj)
% legend( 'q_1', 'q_2')
% grid on
% ylabel('rad')
% xlabel('iteration/time' )
% title('polynomial traj')
% 
% 
% 
% % traj linear
% qtrajL=zeros(nsteps, numel(qi));% inicializar com zeros
% for n=1:numel(qi)
% 
%     qtrajL(:,n)=linspace(qi(n), qf(n), nsteps );
% end
% 
% subplot(2,1,2)
% plot(qtraj)
% legend( 'q_1', 'q_2')
% grid on
% ylabel('rad')
% xlabel('iteration/time' )
% title('polynomial linear')
% hold on
%% Exercicio 3 Animação do movimento

qtraj2   = flipud(qtraj);
qtrajTot = [qtraj; qtraj2];

% animação
%while 1 % animação infinita
for n=1:size(qtrajTot)
    qq = qtrajTot(n,:);
    RR_robot.animate(qq)
    %drawnow% alternativamente pause (t)
    pause(0.01)

end
%end % end do while infinito


%% Exercicio 4 gripper na ponta

% gripper definition

GripperDef= [0.25    0      0      0.25
            0.2      0.2 -0.2    -0.2
            0        0     0     0];

T = RR_robot.fkine(qi);
Gripper = T * GripperDef; 

gr=plotp(Gripper,'r');
gr.LineWidth=3;


% animação
%while 1 % animação infinita
for n=1:size(qtrajTot)
    qq = qtrajTot(n,:);
    RR_robot.animate(qq)

    T = RR_robot.fkine(qq);
    Gripper = T * GripperDef; 

    gr.XData=Gripper(1,:);
    gr.YData=Gripper(2,:);

    %drawnow% alternativamente pause (t)
    pause(0.01)

end
%end % fim do while infinito


%% Alternativas ao comando Link


%% Exercicio 5 Robo RRR Antropomotfico 

%comprimentos de elos
LA=2; LB= 2; LC=1;

L(1)= Revolute ('a', 0, 'd', LA, 'alpha', pi/2);
L(2)= Revolute ('a', LB, 'd', 0, 'alpha', 0);
L(3)= Revolute ('a', LC, 'd', 0, 'alpha', 0);

% Criar Robo  
RRR_robot = SerialLink(L, 'name', 'Robo RR');

% [0,0] é a posição inicial, depois limites do plot e o delay para ver
% melhor os frames 
figure('WindowStyle','docked')
RRR_robot.plot([0 0 0],'workspace', [-6 6 -6 6 -1 5],'delay',0)
hold on

%% Exercicio 6&7  caminho das juntas e animação

qi=[0 0 0];
qA=[pi/2 pi/4 -pi/4];
qf=[-pi/2 -pi/4 pi/4];

nsteps = 100;

%calcula a trajetoria polinomial
RRRtraj1 = jtraj(qi, qA, nsteps); %calcula a trajetoria polinomial
RRRtraj2 = jtraj(qA, qf, nsteps);
RRRtraj3 = flipud(RRRtraj2);
RRRtraj4 = flipud(RRRtraj1);

RRRtrajT = [RRRtraj1; RRRtraj2; RRRtraj3; RRRtraj4];


for n=1:size(RRRtrajT)
    q = RRRtrajT(n,:);
    RRR_robot.animate(q)
    %drawnow% alternativamente pause (t)
    pause(0.01)

end

%% Exercicio 8

% só funciona com link ou prismatic
% junta prismatica o d é variavel e nao o theta

% L3= prismatic depois pode se concatenar, mas nao pode ser usado array
% como os outrops exercicios

Ls(1)= Link (    'a', LA, 'd', LB, 'alpha', 0);
Ls(2)= Link (    'a', LC, 'd', 0, 'alpha', 0);
Ls(3)= Prismatic ('theta',0,'a', 0, 'alpha', 0, 'offset', -1.5);
Ls(4)= Link('a', 0, 'd', 0, 'alpha', 0);

% Criar Robo  
Scara_robot = SerialLink(Ls, 'name', 'Robo SCARA');

figure('WindowStyle','docked')
Scara_robot.plot([0 0 0 0],'workspace', [-6 6 -6 6 -1 5],'delay',0)
hold on

%% exercicio 9 Caminho

qi=[0 0 0 0];
qA=[pi/2 pi/4 0 0];
qD=[0 0 1 0];
qf=[-pi/2 -pi/4 0 0];

nsteps = 100;

%calcula a trajetoria polinomial
SCARAtraj1 = jtraj(qi, qA, nsteps); %calcula a trajetoria polinomial
SCARAtraj2 = jtraj(qA, qD, nsteps);
SCARAtraj3 = jtraj(qD, qf, nsteps);
SCARAtraj4 = flipud(SCARAtraj3);
SCARAtraj5 = flipud(SCARAtraj2);
SCARAtraj6 = flipud(SCARAtraj1);

SCARAtrajT = [SCARAtraj1; SCARAtraj2; SCARAtraj3; SCARAtraj4; SCARAtraj5; SCARAtraj6];


for n=1:size(SCARAtrajT)
    q = SCARAtrajT(n,:);
    Scara_robot.animate(q)
    %drawnow% alternativamente pause (t)
    pause(0.01)

end