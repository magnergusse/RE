%% Aula 1

% MAGNER GUSSE
%% Exercicio 1- Poligono simples

%Pontos do triangulo
P1=[-1 0]';
P2=[1 0]';
P3=[0 2]';
A1=[P1 P2 P3];

%triangulo
h = fill(A1(1,:), A1(2,:),'y');

%eixos e manter Aspect ratio
axis([-10 10 -10 10]);
axis equal
hold on
grid on

% Coordenadas do segundo triangulo
v=[5 0]';
A2=A1+v;

% desenhar o segundo triangulo
h2 = fill(A2(1,:), A2(2,:),'r');

%% Exercicio 2

% Angulo de rotacao 50º em radians
a=50*pi/180;
% Matriz de rotacao
T= [cos(a) -sin(a)
    sin(a) cos(a)]

A3=T*A2;
h3 = fill(A3(1,:), A3(2,:),'b');


% segunda parte, varios angulos

NN=50; %nr de steps
angls=linspace(60,3500,NN);


for b=angls

        T=rot2(b,'deg');% fazer a matriz de rotaçao a cada step
        A4=T*A2;
       % fill(A4(1,:), A4(2,:),'c');% desenhar, se comentado nao desenhar
        h2.XData = A4(1,:);
        h2.YData = A4(2,:);
        pause(0.05)

end

%% Exercicio 3 Transformaçoes homogeneas

close all
clear all
clc

%Pontos do triangulo
P1=[-1 0]';
P2=[1 0]';
P3=[0 2]';
A1=[P1 P2 P3];

A1=e2h(A1);% Acrescentar linha de 1s em baixo

plot_poly(A1, 'fillcolor','y');% plot do poligono
%eixos e manter Aspect ratio
axis([-10 10 -10 10]);
axis equal
hold on
grid on
T = transl2(5,1); % Dá a matriz de translação homogenea
R = trot2(pi/4); % Dá a matriz de rotação homogenea


%% Exerecicio 4 Representar a pre e pos multiplicacao com o R e o T

pre_mult_result = T * R * A1;% primeiro rotação depois translaçao
pos_mult_result = R * T * A1;% Primeiro translação depois rotação 

plot_poly(pre_mult_result, 'fillcolor','b');% plot do poligono pre mult
hold on
plot_poly(pos_mult_result, 'fillcolor','r');% plot do poligono pos mult



%% Exercicio 5 Transformações homogeneas a 3d- Seta
close all
clear all
clc


P = [0.5 0.5 1 0 -1 -0.5 -0.5;
     0 2 2 3 2 2 0;
     0 0 0 0 0 0 0];% Original

Ph = e2h(P); % Homogeneo


Rx = trotx(90,'deg'); % Rotação em torno de x, 90 graus
Ry = troty(90,'deg'); % Rotação em torno de y, 90 graus
Rz = trotz(90,'deg'); % Rotação em torno de z, 90 graus

Px= Ry * Rx *Ph;
Pz= Rx * Ry * Ph;



Ph= h2e(Ph);
Px= h2e(Px);
Pz = h2e(Pz);


roxa = plot_poly(Ph, 'fillcolor','m');% plot da seta roxa
hold on
view(3);
%eixos e manter Aspect ratio
axis([-10 10 -10 10 -10 10]);
axis equal
hold on
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
view(120,25)
trplot(eye(4), 'length', 5)
vermelha = plot_poly(Px, 'fillcolor','r');% plot do poligono do eixo X
hold on
amarela = plot_poly(Pz, 'fillcolor','y');% plot do poligono do eixo X
hold on

%% Exercicio 6 


% segunda parte, varios angulos
a=1;
NN=50; %nr de steps
angls=linspace(0,360,NN);

Ph = e2h(Ph); % Homogeneo
Px = e2h(Px);
Pz = e2h(Pz);


while a<=10

for b=angls

       

        Rx = trotx(b,'deg'); % Rotação em torno de x, 90 graus
        Ry = troty(3*b,'deg'); % Rotação em torno de y, 90 graus
        Rz = trotz(2*b,'deg'); % Rotação em torno de z, 90 graus


        Ph1 = Rx * Ph; % Roxo
        Px1 = Rz * Px; % Vermelho
        Pz1 = Ry * Pz; %amarelo
        


        roxa.XData = Ph1(1,:);  
        roxa.YData = Ph1(2,:);
        roxa.ZData = Ph1(3,:);

        amarela.XData = Pz1(1,:);  
        amarela.YData = Pz1(2,:);
        amarela.ZData = Pz1(3,:);

        vermelha.XData = Px1(1,:);  
        vermelha.YData = Px1(2,:);
        vermelha.ZData = Px1(3,:);
        
        pause(0.05)
end
a=a+1;
end
%% Exercicio 7 Ur3
close all
clc
clearvars

ur3 = loadrobot('universalUR3', 'DataFormat', 'row');
show(ur3);

juntas=[0 0 0 0 0 0];
juntas(1)=0;
juntas(2)=-135*pi/180;
juntas(3)=90*pi/180;

show(ur3,juntas)
angle = linspace(0,-135,50);

for b=angle
    juntas(1)=0;
    juntas(2)=b*pi/180;
    juntas(3)=pi/4;

    show(ur3,juntas,"FastUpdate",false,"PreservePlot",true);
    pause(0.05)
end
