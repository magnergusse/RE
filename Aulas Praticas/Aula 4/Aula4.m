%% Aula 4 Cinematica diferencial

% MAGNER GUSSE 110180

%% Exercicio 1
LA=0.7; LB=0.4; LC=0.3; LD=0.3; LE=0.25; % Robot main measurements
% q d a alpha
DH=[
0, LA, 0, pi/2
0, 0, LB, 0
0, 0, LC, -pi/2
0, 0, LD, pi/2
0, 0, LE, 0
];
robot = SerialLink(DH, 'name', 'Robot5DOF'); % Create robot kinematics chain
q = zeros(1, 5); % Initial joint configuration
wSpace=[-1 2 -2 2 -0.05 2]; % Suggested workspace
fig=figure('Name', 'Euler Angles and Quaternions', 'NumberTitle', 'off'); % Create figure
% Display robot
robot.plot(q, 'workspace', wSpace, 'scale',0.75, 'notiles', 'nobase', 'noshadow',...
'linkcolor','c', 'noname');
hold on % to allow multiple graphics and plots in the sme figure
% Plot fixed reference frame at the base
trplot(eye(4), 'frame', 'B', 'length', 2, 'thick', 1, 'color', 'k');
% use in teach mode for interactive modification of joints
robot.teach(q);

%% EWxercicio 2

%Text for quaternion info -- Initial dummy text to update later
hQuatText=text(1,0,2,'initial text','HorizontalAlignment', 'left','FontSize',12);
% Create a line for quaternion axis representation. Use a red line with a thickness of 2
axisLength=1.5; %length of the red line that represents the quaternion rotation axis
quaternionAxisLine = plot3([0, axisLength],[0, 0],[0, 0],'r-','LineWidth',2);
% Create a handle for the quaternion axis frame
hAxisFrame=trplot(eye(4),'frame','Q','length',0.2,'thick',2,'color','m');
% Create a handle for a replica of the base frame to later animate
hBaseFrame=trplot(eye(4),'frame','BB','arrow','width',0.5,'length',0.4,'thick',1,'color','k');

%% Exercicio 3

% Continuously update Euler angles when joints change
while isvalid(robot)
q = robot.getpos(); % Get current joint angles
T = robot.fkine(q); % Compute forward kinematics
quat = UnitQuaternion(T); % Create corresponding UnitQuaternion from T
quatAngle=2*acos(quat.s); % Calculate rotation angle after the real part of quaternion
%Update the string with the complete information about the quaternion
hQuatText.String = sprintf('w = %.3f , ang=%.1f deg \nx = %.3f, y = %.3f,z = %.3f', ...
quat.s, 180/pi*quatAngle, quat.v(1), quat.v(2), quat.v(3));
%pause(0.25); % Small delay to reduce CPU usage
% end
% 
 % Exercicio 4
% 
% while isvalid(robot)
%... code from previous exercises
qaxis = quat.v; % extract the quaternion vector part (x, y, z)
if any(quat.v) % normalize only if non null vector (in case no rotation exists)
qaxis = qaxis/norm(qaxis); % Normalize to unit vector
end
% Update the axis line representing the quaternion axis of rotation
set(quaternionAxisLine, 'XData', [0, axisLength*qaxis(1)], ...
'YData', [0, axisLength*qaxis(2)], ...
'ZData', [0, axisLength*qaxis(3)]);
axisOffset = axisLength * qaxis;
quatAxisMatrix = transl(axisOffset) * quat.T;
% Update the quaternion axis frame location and orientation
set(hAxisFrame, 'Matrix', quatAxisMatrix);
% Update the base reference frame location to haf way along the quaternion axis
set(hBaseFrame, 'Matrix', transl(axisOffset/2));
pause(0.25); % Small delay to reduce CPU usage
end


%% Exercicio 6
clear 
close all
syms q1 q2 LA LB real

DH1 = [q1 0 LA 0
    q2 0 LB 0];

RR = SerialLink(DH1, 'name', 'RR Planar Symbolic');

T = RR.fkine([q1 q2]);


%% Exercicio 7

function J=RRjacobian(robot,Q)

LA=robot.a(1);
LB=robot.a(2);

q1=Q(1);
q2=Q(2);


J = [-LB*sin(q1 + q2) - LA*sin(q1), -LB*sin(q1 + q2)
      LB*cos(q1 + q2) + LA*cos(q1),  LB*cos(q1 + q2)];
end

%% Exercicio 8

clear 
close all
syms q1 q2 LA LB real
LA=2;LB=1;
q1=0; q2=0;

DH1 = [q1 0 LA 0
    q2 0 LB 0];

RR = SerialLink(DH1, 'name', 'RR Planar Symbolic');

Q=[pi/4 pi/3];


jj= RRjacobian(RR,Q)
jac=jacob0(RR,Q)

%% Exercicio 9

close all
Qi=[0 0];
Qf=[pi/3 pi/2];
Dt=1;


NN=100;
qtraj=jtraj(Qi,Qf,NN);
tempo = linspace(0, 1, 99);

vq=diff(qtraj)/(Dt/NN);
figure('WindowStyle','docked')

plot(tempo, vq(:,1));
hold on
plot(tempo, vq(:,2));
hold 
legend( 'v_1', 'v_2')
grid on
ylabel('rad/s')
xlabel('time')

figure('WindowStyle','docked')
RR.plot(qtraj,'fps',30);

%% Exercicio 10
vq=vq';
vp = zeros(2, size(vq, 2)); % %velocidade
pp=zeros(2,100);%posiçao

figure('WindowStyle','normal')


for n = 1:size(vq, 2)

    qq = qtraj(n,:);
    RR.animate(qq);

    J=jacob0(RR,qq);

    %drawnow% alternativamente pause (t)

    vp(:,n)=J(1:2,:)*vq(:,n);
    pause(0.01)

end
hold on
% Plot the end-effector positions

plot(tempo,vp(1,:));
hold on
plot(tempo,vp(2,:));
xlabel('X'); ylabel('Y'); 
title('End-Effector velocity');
grid on;