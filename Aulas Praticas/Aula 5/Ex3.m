%% Aula 5 Cinematica diferencial inversa
%Trajetorias no espa¸co operacional

% MAGNER GUSSE 110180

% Exercicio 1: Função RRJacobianInv

%% Exercicio 2 Trajetoria circular do RR
clc
clear 
close all
syms q1 q2 LA LB real
LA=1;LB=1;
q1=0; q2=0;

DH1 = [q1 0 LA 0
    q2 0 LB 0];

RR = SerialLink(DH1, 'name', 'RR Planar Robot');
figure('WindowStyle','docked')
% RR.plot([0 0], 'workspace', [-4 4 -4 4 -1 1],'delay',0)
% hold on


% A=[-1;1.5];%Ponto inicial
% B=[1.75;0];%Ponto final
% determinar primeiro o caminho

C=[1;0];%Centro da circunferencia
r=0.5;%Raio da circunferencia
w=2*pi; %Nr de voltas 

NN=100;
time=linspace(0,1,NN);%Tempo

path= C+r*[cos(w*time); sin(w*time)];%Caminho obedece a euqaçao parametrica da reta
% plot(path(1,:),path(2,:),'*b')

dr=gradient(path);

q = zeros(2, NN); %initialize the array for the joints' values.
% Initial inverse kinematics to get starting position of robot
qi=[-1 1];

q(:, 1) = RR.ikine(transl(path(1,1),path(2,1), 0), 'q0',qi,'mask', [1 1 0 0 0 0]);
for i = 2:NN
J = RR.jacob0(q(:,i-1)); % Compute Jacobian
Ji = pinv(J(1:2, :)); % ... and its inverse

%Ji = RRJacobianInv(RR,q(:,i-1));


dq = Ji * dr(:,i); % Compute joint increments
q(:, i) = q(:, i-1) + dq; % update the joint values valor anterior + o incremento atual


alldq(:,i)=dq;% Velocidades das juntas
end


RR.plot(q','workspace', [-4 4 -4 4 -1 1], 'delay', 0.01, 'trail', {'b', 'LineWidth', 1.5});

figure
plot(time,alldq)