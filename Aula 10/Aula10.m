%% Aula 10
%Magner Gusse 110180
%
clc; close all; clear;
%% Exercicio 1: Modelo simples de Rover
rover_shape = robotShape();

% Preparation and initial representation
figure;
axis equal; grid on; hold on
% Indicative initial limits
xlim([-1 13]); ylim([-1 7]);
xlabel('X [m]'); ylabel('Y [m]');
title('Mars Rover Odometry');
% Create graphic handle for the rover
h_true = fill(rover_shape(1, :), rover_shape(2, :), 'g', 'FaceAlpha', 0.6);
h_odom = fill(rover_shape(1, :), rover_shape(2, :), 'r', 'FaceAlpha', 0.6);
% General simulation parameters
dt = 0.1; % Time step [s]
T = 20; % Total time [s]
N = round(T/dt); % Number of steps


%% Exercicio 2: Movimento e odometria sem erros
% Exercicio 3 Movimento e odometria com erros

%
%erros tipo derrapagem, uma roda mais gasta que outra, etc

bias_v = 0.015; % Systematic bias in linear velocity (e.g., worn wheel)
bias_w = -0.005; % Systematic bias in angular velocity (id.)
sigma_v = 0.02; % Random noise std dev for linear velocity (emulate slippage)
sigma_w = 0.01; % Random noise std dev for angular velocity (idem)

% copiar o exercicio anterior e adicionar os erros




%Primeira parte

% Assume initial constant velocities
v_cmd0 = 0.4; % linear [m/s]
w_cmd0 = pi/40; % angular [rad/s]

% Matrix to store the states [x; y; theta]
true_pose = zeros(3, N);
% Create the empty trajectory as an animatedline
traj_true = animatedline('Color','g','LineWidth',1.5);
% Create the empty trajectory as an animatedline
traj_true2 = animatedline('Color','r','LineWidth',0.5,'LineStyle','--');

% Ciclo de simulaçao/segunda parte


% Assume initial constant velocities
v_cmd1 = 0.4; % linear [m/s]
w_cmd1 = pi/40; % angular [rad/s]
% Matrix to store the states [x; y; theta]

odom_pose = zeros(3, N);

for k = 2:N % Why starting in 2? porque a cinematica depende da posiçao anterior
w_cmd = w_cmd0;

if k >= N/2
w_cmd = -5*w_cmd0;
end


v_cmd = v_cmd0;
% Calculate current position after previous
true_pose(3,k)=true_pose(3,k-1)+w_cmd*dt;
theta=true_pose(3,k);
true_pose(1,k)=true_pose(1,k-1)+v_cmd*cos(theta)*dt;
true_pose(2,k)=true_pose(2,k-1)+v_cmd*sin(theta)*dt;

% Update visuals
R_true = [cos(theta) -sin(theta)
sin(theta) cos(theta)];
shape_true = R_true * rover_shape + true_pose(1:2,k);
h_true.XData=shape_true(1,:);
h_true.YData=shape_true(2,:);


% Upate trajectory
addpoints(traj_true, true_pose(1,k), true_pose(2,k));
pause(0.01)


%Primeira parte


% Ciclo de simulaçao/segunda parte

w_cmd = w_cmd1;

if k >= N/2
w_cmd = -5*w_cmd1;
end

v_cmd = v_cmd1;

v_meas = v_cmd + bias_v + sigma_v*randn();
w_meas = w_cmd + bias_w + sigma_w*randn();

% Calculate current position after previous
odom_pose(3,k)=odom_pose(3,k-1)+w_cmd*dt;
theta2=odom_pose(3,k);
odom_pose(1,k)=odom_pose(1,k-1)+v_meas*cos(theta)*dt;
odom_pose(2,k)=odom_pose(2,k-1)+v_meas*sin(theta)*dt;

% Update visuals
R_odom = [cos(theta2) -sin(theta2)
sin(theta2) cos(theta2)];
shape_odom = R_odom * rover_shape + odom_pose(1:2,k);
h_odom.XData=shape_odom(1,:);
h_odom.YData=shape_odom(2,:);


% Upate trajectory
addpoints(traj_true2, odom_pose(1,k), odom_pose(2,k));
pause(0.01)
end


%% Exercicio 4




close
m=mobiledev
h=figure;
m.OrientationSensorEnabled=1;
m.Logging = 1; %start logging
ang1=animatedline('Color', 'b', MaximumNumPoints ,500);
ang2=animatedline('Color', 'r', MaximumNumPoints ,500);
ang3= animatedline('Color', 'g', MaximumNumPoints ,500);
t=0; dt=0.1;
grid on; ylim=[-180 180];
legend('azimuth', 'pitch', 'roll');
while isvalid(h)
t=t+dt;
if numel(m.Orientation) > 2
addpoints(ang1,t,m.Orientation(1));
addpoints(ang1,t,m.Orientation(2));
addpoints(ang1,t,m.Orientation(3));
end
pause(dt)
end
m.Logging = 0; %stop logging