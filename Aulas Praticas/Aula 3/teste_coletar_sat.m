
clc; clear; close all;

%% Exercicio 4 Canadarm2

DH1 = [ 0 0.380  0    -pi/2;
      0 0.635  0     pi/2;
      0 0.504  6.85  0;
      0 0.8    6.85  0;
      0 0.504  0    -pi/2;
      0 0.635  0     pi/2;
      0 0.380  0     0 ];
ISS_Robot= SerialLink(DH1, 'name', 'Canadarm2');% criar o robo
q0=zeros(1,7);
figure('WindowStyle','docked')
ISS_Robot.plot(q0,'scale',0.4,'workspace', [-2 18 -14 14 -10 10],'delay',0, 'notiles','nobase','noshadow','noname','nojoints','jointdiam',0.5,'linkcolor','c')
% sem o xadrez, sem a base, sem aparecer o nome
hold on
ISS_Robot.teach()

%% Exercicio 5 por os satelites

hold on

T1(:,:,1)=transl([5 5 5 ])* trotx(-45)*troty(30);
T1(:,:,2)=transl([8 4 -6 ])* trotx(-45)*troty(30);
T1(:,:,3)=transl([5 -8 0 ])* trotx(-45)*troty(30);

% % Fazer plot dos satelites
% for n=1:size(T1,3)
% 
% drawSatellite(T1(:,:,n))
% end

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

%while 1 % animação infinita
for n=1:size(Trajeto)
    qq = Trajeto(n,:);
    ISS_Robot.animate(qq)

    T = ISS_Robot.fkine(qq);
    
    %drawnow% alternativamente pause (t)
    drawSatellite(T)
    pause(0.01)

end
%end % fim do while infinito
