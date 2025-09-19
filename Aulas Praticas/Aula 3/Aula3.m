%% Aula 3 Cinematica Inversa

% MAGNER GUSSE 110180

%% Exercicio 1
clc
clear
%Matriz DH na ordem tetha_i, d_i, a_i, alpha_i
LA=2; LB=1;
DH=[0 0 LA 0
    0 0 LB 0];

RR_Robot= SerialLink(DH, 'name', 'RR_(Planar)');% criar o robo

RR_Robot2= SerialLink(DH, 'name', 'RR_(Planar2)');% criar o robo
% alinea B

target=[2.5 0.5 0];% posiçao de destino


T=transl(target);%Matriz de transformação

q=RR_Robot.ikine(T,'mask', [1 1 0 0 0 0]);% metodo de inverese kinematics, mas não dá redundancias

% [0,0] é a posição inicial, depois limites do plot e o delay para ver
% melhor os frames 
figure('WindowStyle','docked')
subplot(1,2,1)
RR_Robot.plot(q, 'workspace', [-4 4 -4 4 -1 1],'delay',0, 'notiles','nobase','noshadow','noname')
% sem o xadrez, sem a base, sem aparecer o nome
hold on


%% Exercicio 2
% função invKinRRplanar

%% Exercicio 3

Q=invKinRRplanar(RR_Robot,target);

subplot(1,2,2)
view(-20,33)
NN=20;
traj1=jtraj(Q(1,:),Q(2,:),NN);%Trajetoria de redundancia Q(1alinha;todas colunas) a 2
traj2=jtraj(Q(2,:),Q(1,:),NN);

trajT=[traj1;traj2];

RR_Robot2.plot(trajT,'fps',30);

%% Exercicio 4 Canadarm2

DH1 = [ 0 0.380  0    -pi/2;
      0 0.635  0     pi/2;
      0 0.504  6.85  0;
      0 0.8    6.85  0;
      0 0.504  0    -pi/2;
      0 0.635  0     pi/2;
      0 0.380  0     0 ];
ISS_Robot= SerialLink(DH1, 'name', 'Canadarm2');% criar o robo

q0=zeros(1,7);% 7 juntas do robo, portanto matriz de inicializaçao tem de ter 7 zeros


figure('WindowStyle','docked')
    ISS_Robot.plot(q0,'scale',0.4,'workspace', [-2 18 -14 14 -10 10],'delay',0, 'notiles','nobase','noshadow','noname','nojoints','jointdiam',0.5,'linkcolor','c')
    % sem o xadrez, sem a base, sem aparecer o nome
hold on
ISS_Robot.teach()

%% Exercicio 5 por os satelites

hold on

T1(:,:,1)=transl([10 5 5 ])* trotx(-45)*troty(30);
T1(:,:,2)=transl([8 4 -6 ])* trotx(-45)*troty(30);
T1(:,:,3)=transl([5 -8 0 ])* trotx(-45)*troty(30);

% Fazer plot dos satelites
for n=1:size(T1,3)

drawSatellite(T1(:,:,n))
end

%% Exercicio 6
% Criar Q1, Q2, Q3 e criar as trajetorias e dar plot
hold on
Nsteps=50;

Q1=ISS_Robot.ikine(T1(:,:,1));
Q2=ISS_Robot.ikine(T1(:,:,2));
Q3=ISS_Robot.ikine(T1(:,:,3));

trajeto1=jtraj(q0,Q1,Nsteps);%0 a 1
trajeto2=jtraj(Q1,Q2,Nsteps);%1 a 2
trajeto3=jtraj(Q2,Q3,Nsteps);%2 a 3
trajeto4=jtraj(Q3,q0,Nsteps);%3 a 0
Trajeto=[trajeto1;trajeto2;trajeto3;trajeto4];

ISS_Robot.plot(Trajeto,'fps',30);