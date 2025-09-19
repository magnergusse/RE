%% Aula 6 Estatica de Manipuladores 
%Trajetorias no espa¸co operacional

% MAGNER GUSSE 110180
%%
close all
clear
clc

L1=1.5;
L2=1;

DH=[0 0 L1 0
    0 0 L2 0];

Robot= SerialLink(DH, 'name', 'Robot');

q=[-pi/8,pi/3];%Posição inicial
F=[5;10;0; 0; 0; 0];%Força a aplicar na ponta generalizada (Forças e Momentos)


figure('WindowStyle','docked')
Robot.plot(q,'nobase','nojoints','noname');
hold on
axis([-2 4 -3 3 -3 3])
J = Robot.jacob0(q); % Compute Jacobian

tau=J'*F;
T=Robot.fkine(q);
P=T.t;

h=drawFTvector(P,F(1:3),0.15,'r','F');

T0=SE3(eye(4));
P1=T0.t;
ax1=T0.a;


T1=Robot.A(1,q);
P2=T1.t;
ax2=T1.a;

H=drawFTvector(P1,tau(1)*ax1,0.15,'m','tau');
hold on
Ha=drawFTvector(P2,tau(2)*ax2,0.15,'m','tau_2');
hold on



%% Exercicio 2
NN=50;
orientacao=[linspace(1.25*pi, -0.75*pi, NN),linspace(-0.75*pi, 0.5*pi, NN)];
Trajectory=zeros(size(orientacao),2);

for t=orientacao

    Fx=10*cos(t); F(1)=Fx;
    Fy=10*sin(t); F(2)=Fy;
    
    tau=J'*F;

    drawFTvector(P,F(1:3),0.15,'r','F',h);
    hold on
    drawFTvector(P1,tau(1)*ax1,0.15,'m','tau',H);
    hold on
    drawFTvector(P2,tau(2)*ax2,0.15,'m','tau_2',Ha);
    hold on
pause(0.05)
end

%% Exercicio 3
LA=1.5;LB=1;

DH=[0 0 LA 0
    0 0 LB 0];
robot=SerialLink(DH,'name','"DOF With spring');

Qi=[pi/3 -pi/4]; 
QA=[pi/2 pi/4];

traj=jtraj(Qi,QA,100);


P1 = [4, 0, 0];%Ponta fixa da mola

% view(120,90)
figure('WindowStyle','docked')
robot.plot(traj(1,:), 'nobase','nojoints', ...
    'nowrist', 'noname', 'notiles', ...
    'noshadow', 'linkcolor', 'c', 'delay', 0.00);
hold on
animate_robot_with_spring_inc(robot, traj, P1, 15, 0.08);