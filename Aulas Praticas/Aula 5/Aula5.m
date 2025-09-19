%% Aula 5 Cinematica diferencial inversa
%Trajetorias no espa¸co operacional

% MAGNER GUSSE 110180

% Exercicio 1: Função RRJacobianInv

%% Exercicio 2 Trajetoria Linear do RR
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


A=[-1;1.5];%Ponto inicial
B=[1.75;0];%Ponto final
% determinar primeiro o caminho

NN=100;

time=linspace(0,1,NN);%Tempo

path=A+(B-A)*time;%Caminho obedece a euqaçao parametrica da reta
% plot(path(1,:),path(2,:),'*b')

dr=gradient(path);

q = zeros(2, NN); %initialize the array for the joints' values.
% Initial inverse kinematics to get starting position of robot
q(:, 1) = RR.ikine(transl(A(1), A(2), 0), 'mask', [1 1 0 0 0 0]);
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
%No tp1 olhar para o jacobiano antes de ir, ou seja, ver se é ou não
%alcançavel

%% Exercicio 4, ex3 noutro ficheiro
clc
clear 

LA=1; LB=1; LC=0.5; LD=0.3;
DH=[ 0 LA 0 pi/2
0 0 LB 0
0 0 LC 0
0 0 0 pi/2
0 LD 0 0];

R5DOF = SerialLink(DH, 'name', '5DOF Robot');
figure('WindowStyle','docked')
 R5DOF.plot([0 0 0 0 0])

hold on;
axis square
wspace=[-1 2 -1.5 1 0.5 2];
axis(wspace)
A=[0 -1.25 0]'; B=[1.5 -0.75 1.5]';

NN=100;

time=linspace(0,1,NN);%Tempo

path=A+(B-A)*time;%Caminho obedece a euqaçao parametrica da reta
% plot(path(1,:),path(2,:),'*b')

dr=gradient(path);

qi=[0 -1 1 1 0];
q = zeros(5, NN); %initialize the array for the joints' values.
% Initial inverse kinematics to get starting position of robot
q(:, 1) = R5DOF.ikine(transl(A(1), A(2), A(3)), 'q0',qi, 'mask', [1 1 1 0 0 0]);



for i = 2:NN
J = R5DOF.jacob0(q(:,i-1)); % Compute Jacobian
Ji = pinv(J(1:3, :)); % ... and its inverse

%Ji = RRJacobianInv(RR,q(:,i-1));


dq = Ji * dr(:,i); % Compute joint increments
q(:, i) = q(:, i-1) + dq; % update the joint values valor anterior + o incremento atual


alldq(:,i)=dq;% Velocidades das juntas
end

figure('WindowStyle','docked')
R5DOF.plot(q', ...
'notiles',...
'noname',...
'nobase', ...
'noshadow', ...
'scale', 0.5, ...
'trail', {'b', 'LineWidth', 1.5}, ...
'delay', 0.001 ...
); % Plot full motion with trail

hold on
%% Exercicio 5
%Definition of vertices
V=0.25*[
1 -1 0 %point 1
1 1 0 %point 2
-1 1 0 %point 3
-1 -1 0 %point 4
0 0 2 %point 5
];
%definition of faces
F = [
1 2 5 5 %face1
2 3 5 5 %face2
3 4 5 5 %face3
4 1 5 5 %face4
1 2 3 4 %face5
];
%simple color index to paint the faces
fColor= [ 1 2 3 4 5 ]';


figure('WindowStyle','docked')
R5DOF.plot([0 0 0 0 0])
R5DOF.animate(qi)
% hold on
%pause (0.25)
h=patch('Vertices',V, 'Faces', F,'FaceVertexCData', fColor,'FaceColor','flat');
hold on
%h=scale(h,0.5);
% T=R5DOF.fkine(q(:,1));
% Vn=T*V';
% h.Vertices=Vn';

%% Exercicio 6

% 
% 
for i = 2:NN
J = R5DOF.jacob0(q(:,i-1)); % Compute Jacobian
Ji = pinv(J(1:3, :)); % ... and its inverse

%Ji = RRJacobianInv(RR,q(:,i-1));

dq = Ji * dr(:,i); % Compute joint increments
q(:, i) = q(:, i-1) + dq; % update the joint values valor anterior + o incremento atual

R5DOF.animate(q(:, i)')

T=R5DOF.fkine(q(:,i));
Vn=T*V';
h.Vertices=Vn';
drawnow;

%alldq(:,i)=dq;% Velocidades das juntas
end
% 


%% exercicio 7

LA=1; LB=1; LC=0.5; LD=0.3;
DH=[ 0 LA 0 pi/2
0 0 LB 0
0 0 LC 0
0 0 0 pi/2
0 LD 0 0];

R_5DOF = SerialLink(DH, 'name', '5DOF Robot');
figure('WindowStyle','docked')
R_5DOF.plot([0 0 0 0 0])
hold on
C=[0.75 0 1]';
R=0.5;
w=2*pi; %Nr de voltas 

NN=100;
time=linspace(0,1,NN);%Tempo

path= C+R*[(cos(w*time)); (sin(w*time));-0.25*time];

dr=gradient(path);

q = zeros(5, NN); %initialize the array for the joints' values.
% Initial inverse kinematics to get starting position of robot



Pi=[0 1.5 0.5];
T=transl(Pi);%Matriz de transformação
qi=R_5DOF.ikine(T,'mask', [1 1 1 0 0 0]);

R_5DOF.plot(qi);
hold on


q(:, 1) = R_5DOF.ikine(transl(path(1,1),path(2,1), path(3,1)), 'q0',qi,'mask', [1 1 1 0 0 0]);
for i = 2:NN
J = R_5DOF.jacob0(q(:,i-1)); % Compute Jacobian
Ji = pinv(J(1:3, :)); % ... and its inverse

%Ji = RRJacobianInv(RR,q(:,i-1));

dq = Ji * dr(:,i); % Compute joint increments
q(:, i) = q(:, i-1) + dq; % update the joint values valor anterior + o incremento atual


alldq(:,i)=dq;% Velocidades das juntas
end


R_5DOF.plot(q','workspace', [-3 3 -3 3 -2 3], 'trail', {'b', 'LineWidth', 1.5});
