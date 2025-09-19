%% Trabalho Pratico 1
% Magner Gusse 110180
% Robótica Espacial


%%
clc
clear
close all


% Satelites=readmatrix('tp1.txt');
% Sat=cell(2, 1);
% Sat{1}=transl([Satelites(1,1) Satelites(1,2) Satelites(1,3)])* trotx(Satelites(1,4))*troty(Satelites(1,5))*trotz(Satelites(1,6));
% Sat{2}=transl([Satelites(2,1) Satelites(2,2) Satelites(2,3)])* trotx(Satelites(2,4))*troty(Satelites(2,5))*trotz(Satelites(2,6));
% 

% Verifica se o arquivo 'tp1.txt' existe
if exist('tp1.txt', 'file') == 2
    % Lê os dados do arquivo
    Satelites = readmatrix('tp1.txt');
    
    % Verifica se a matriz possui pelo menos 2 linhas e 6 colunas
    if size(Satelites, 1) < 2 || size(Satelites, 2) < 6
        warning('Arquivo tp1.txt encontrado, mas os dados são insuficientes. Valores padrão serão utilizados.');
        % Valores padrão para dois satélites
        Satelites = [8000, -5000, -4000, 0, 0, 0;
                     -8000,  5000,  4000, 0, 0, 0];
    end
else
    warning('Arquivo tp1.txt não foi encontrado. Valores padrão serão utilizados.');
    % Valores padrão para dois satélites
    Satelites = [8000, -5000, -4000, 0, 0, 0;
                 -8000,  5000,  4000, 0, 0, 0];
end

% Cria um cell array para armazenar as transformações dos satélites
Sat = cell(2, 1);
Sat{1} = transl([Satelites(1,1), Satelites(1,2), Satelites(1,3)]) * ...
         trotx(Satelites(1,4)) * troty(Satelites(1,5)) * trotz(Satelites(1,6));
Sat{2} = transl([Satelites(2,1), Satelites(2,2), Satelites(2,3)]) * ...
         trotx(Satelites(2,4)) * troty(Satelites(2,5)) * trotz(Satelites(2,6));



figure(1)
[Docking_positions, docking_targets, sat_park]=DrawISS;
Docks=Docking_positions([1, 2, 5, 6]);
hold on
% Define approximation points for docking ports
approximation_points_ISS = [
    docking_targets{1}(1:3,4)' + [0, 0, 500];   % Dock 1: +Z offset
    docking_targets{2}(1:3,4)' + [0, 0, 500];   % Dock 2: +Z offset
    docking_targets{3}(1:3,4)' + [0, 0, -500];  % Dock 3: -Z offset
    docking_targets{4}(1:3,4)' + [0, 0, -500];  % Dock 4: -Z offset
    docking_targets{5}(1:3,4)' + [500, 0, 0];   % Dock 5: +X offset
    docking_targets{6}(1:3,4)' + [-500, 0, 0];  % Dock 6: -X offset
];


% Create a 3D scatter plot
scatter3(approximation_points_ISS(:, 1), approximation_points_ISS(:, 2), approximation_points_ISS(:, 3),50,'filled');


hold on
[points_sat{1}, normals_sat{1},Tmatrix{1},approx_T{1},drawings(1,:)] = DrawSat(Sat{1});
[points_sat{2}, normals_sat{2},Tmatrix{2},approx_T{2},drawings(2,:)] = DrawSat(Sat{2});

Sat_pos=Tmatrix;
final_pos=Tmatrix;

view(-130,50)
axis square
axis equal
axis([-20000 20000 -20000 20000 -20000 20000])

%
DH = [ 0 1030  0    -pi/2;
      0 850  0     pi/2;
      0 700  6850  0;
      0 400   6850  0;
      0 700  0    -pi/2;
      0 850  0     pi/2;
      0 1030  0     0];

     T_base = transl([0, -5500, 1500]);  % deslocamento da base
     T_tip = transl([0 5500 1500])*trotz(-pi/2) * troty(0) * trotx(-pi/2);
     %T_tip=SE3(T_tip);
     Robot= SerialLink(DH, 'name', 'Canadarm2');% criar o robo
     start_index=1;
     Robot.base = Docking_positions{start_index};
       Robot.tool = trotz(-pi/2);

        
     q0=[-2.984513, 1.5729, 2.4305, -1.658063, -0.7627, 1.5921,0];
           

     Robot.plot(q0,'scale',0.4,'delay',0, 'notiles','nobase','noshadow','noname','nojoints','jointdiam',0.5,'linkcolor','c')
        hold on
        q_current = Robot.getpos;
        
%Corrigir melhor a posição inicial
     % Robot.teach

hold on
%index=6;
pause
fprintf('Press Enter');

%[q_current, Robot] = MoveToNewBase(Robot,  docking_targets{index}, Docking_positions{index},approximation_points_ISS(2,:), approximation_points_ISS(index,:),q0,index);

satellite_pairs = computeSatelliteDockingPairs(Docks, {points_sat{1}, points_sat{2}});

% Decide which satellite to handle first by comparing the precomputed distances.
% Identify which satellite is paired with this base
if satellite_pairs{1}.docking_index == start_index
    first_sat = 1;
    second_sat = 2;
else
    first_sat = 2;
    second_sat = 1;
end
fprintf('First satellite to be serviced: Satellite %d (assigned to base %d)\n', first_sat, start_index);


%% First movements(Go to Sat1)
Steps=150;
Steps_couple =10;
time=linspace(0,1,Steps);%Tempo
time_couple=linspace(0,1,Steps_couple);%Tempo


%go to approx point
Start= Robot.fkine(Robot.getpos).T;
End = approx_T{first_sat}{satellite_pairs{first_sat}.sat_face_index};

[q,path] = Calculatetraj(Robot, Start, End, Steps);
Robot.plot(q)
delete(path)

%couple
Start= Robot.fkine(Robot.getpos).T;
End = Tmatrix{first_sat}{satellite_pairs{first_sat}.sat_face_index}.T;

[q,path]= Correct_orientation(Robot,Start,End,Steps_couple);
 Robot.plot(q)
delete(path)

%%
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{1};
End(3,4)=End(3,4)+2000;

%q = Calculatetraj(Robot, Start, End, 150);
[q,path] = Correct_orientation(Robot,Start,End,Steps);

Tcentro=(Start)\Sat{first_sat};

%Robot.plot(q')
    for i=1:size(q,1)
        delete(drawings(first_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;
        [points_sat{first_sat}, normals_sat{first_sat},Tmatrix{first_sat},approx_T{first_sat},drawings(first_sat,:),Sat{first_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
delete(path)


Start= Robot.fkine(Robot.getpos).T;
End = sat_park{1};

 [q,path] = Correct_orientation(Robot,Start,End,Steps_couple);

% Tcentro=(Start)\Sat{first_sat};
 %Robot.plot(q)
    for i=1:size(q,1)
        delete(drawings(first_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;
        [points_sat{first_sat}, normals_sat{first_sat},Tmatrix{first_sat},approx_T{first_sat},drawings(first_sat,:),Sat{first_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
delete(path)
%% troca de base
if satellite_pairs{second_sat}.docking_index == 3
index=2;
elseif satellite_pairs{second_sat}.docking_index == 4
index=6;
elseif satellite_pairs{second_sat}.docking_index == 5 || satellite_pairs{second_sat}.docking_index == 6
index=2;
else
    index=satellite_pairs{second_sat}.docking_index;
end
 % index=satellite_pairs{second_sat}.docking_index;

 [q_current, Robot] = MoveToNewBase(Robot,  docking_targets{index}, Docking_positions{index},approx_T{first_sat}{satellite_pairs{first_sat}.sat_face_index}, approximation_points_ISS(index,:),Robot.getpos,index);
% [q_current, Robot] = MoveToNewBase(Robot,  docking_targets{index}, Docking_positions{index},End(1:3,4), approximation_points_ISS(index,:),Robot.getpos,index);

%% Go to second

    Start= Robot.fkine(Robot.getpos).T;
    End = Start;
    End(3,4) = End(3,4) + 5500; 
    
    [q,path] = Calculatetraj(Robot, Start, End, Steps);
    Robot.plot(q)
    delete(path)


    %go to approx point
    Start= Robot.fkine(Robot.getpos).T;
    End = approx_T{second_sat}{satellite_pairs{second_sat}.sat_face_index};
    
    [q,path] = Correct_orientation(Robot, Start, End, Steps);
    Robot.plot(q)
    delete(path)


    Start= Robot.fkine(Robot.getpos).T;
    End = Tmatrix{second_sat}{satellite_pairs{second_sat}.sat_face_index}.T;
    
    [q,path] = Correct_orientation(Robot,Start,End,Steps_couple);
    Robot.plot(q)
    delete(path)

%% Trazer ate a estação

%Up primeiro
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{2};
End(3,4) = End(3,4) +3500;
[q,path] = Calculatetraj(Robot, Start, End, Steps);
Tcentro=(Start)\Sat{second_sat};

 %Robot.plot(q)
    for i=1:size(q,1)
        delete(drawings(second_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;

        [points_sat{second_sat}, normals_sat{second_sat},Tmatrix{second_sat},approx_T{second_sat},drawings(second_sat,:),Sat{second_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
    delete(path)
    
    % Now down
% second go down
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{2};
[q,path] = Correct_orientation(Robot, Start, End, Steps);
    for i=1:size(q,1)
        delete(drawings(second_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;

        [points_sat{second_sat}, normals_sat{second_sat},Tmatrix{second_sat},approx_T{second_sat},drawings(second_sat,:),Sat{second_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
delete(path)

%% Ir ao primeiro sat
%primeiro pra cima
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{2};
End(3,4) = End(3,4) +2500;
[q,path] = Calculatetraj(Robot, Start, End, Steps);
Robot.plot(q)
delete(path)



%Para o lado
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{1};
End(3,4) = End(3,4) +5000;
[q,path] = Calculatetraj(Robot, Start, End, Steps);
Robot.plot(q)
delete(path)


% para o ponto de aprox
Start= Robot.fkine(Robot.getpos).T;
End = approx_T{first_sat}{satellite_pairs{first_sat}.sat_face_index};

[q,path] = Calculatetraj(Robot, Start, End, Steps);
Robot.plot(q)
delete(path)


%couple
Start= Robot.fkine(Robot.getpos).T;
End = Tmatrix{first_sat}{satellite_pairs{first_sat}.sat_face_index}.T;

 [q,path] = Correct_orientation(Robot,Start,End,Steps_couple);
 Robot.plot(q)
 delete(path)
%% take first sat to second sat first position
    
    %First up
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{1};
End(3,4) = End(3,4) +5000;
[q,path] = Calculatetraj(Robot, Start, End, Steps);
Tcentro=(Start)\Sat{first_sat};

    for i=1:size(q,1)
        delete(drawings(first_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;
        [points_sat{first_sat}, normals_sat{first_sat},Tmatrix{first_sat},approx_T{first_sat},drawings(first_sat,:),Sat{first_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
delete(path)

    %Now to position
    Start= Robot.fkine(Robot.getpos).T;
    End = final_pos{second_sat}{satellite_pairs{second_sat}.sat_face_index}.T;
    
    [q,path] = Correct_orientation(Robot,Start,End,Steps);

    for i=1:size(q,1)
        delete(drawings(first_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;
        [points_sat{first_sat}, normals_sat{first_sat},Tmatrix{first_sat},approx_T{first_sat},drawings(first_sat,:),Sat{first_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
    delete(path)
%%
% troca de base
index=1;

%[q_current, Robot] = MoveToNewBase(Robot,  docking_targets{index}, Docking_positions{index},approx_T{first_sat}{satellite_pairs{first_sat}.sat_face_index}(1:3,4), approximation_points_ISS(index,:),Robot.getpos,index);
 [q_current, Robot] = MoveToNewBase(Robot,  docking_targets{index}, Docking_positions{index},approx_T{first_sat}{satellite_pairs{first_sat}.sat_face_index}, approximation_points_ISS(index,:),Robot.getpos,index);

%% Go to second sat again
    Start= Robot.fkine(Robot.getpos).T;
    End = Start;
    End(3,4) = End(3,4) + 5500; 
    
    [q,path] = Calculatetraj(Robot, Start, End, Steps);
    Robot.plot(q)
    delete(path)
    %go to approx point
    Start= Robot.fkine(Robot.getpos).T;
    End = approx_T{second_sat}{satellite_pairs{second_sat}.sat_face_index};
    
    [q,path] = Correct_orientation(Robot, Start, End, Steps);
    Robot.plot(q)
    delete(path)


    Start= Robot.fkine(Robot.getpos).T;
    End = Tmatrix{second_sat}{satellite_pairs{second_sat}.sat_face_index}.T;
    
    [q,path] = Correct_orientation(Robot,Start,End,Steps_couple);
    Robot.plot(q)
    delete(path)
%% Take second to first position

    %First up
Start= Robot.fkine(Robot.getpos).T;
End  = sat_park{2};
End(3,4) = End(3,4) +5000;
[q,path] = Correct_orientation(Robot, Start, End, Steps);
Tcentro=(Start)\Sat{second_sat};

    for i=1:size(q,1)
        delete(drawings(second_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;
        [points_sat{second_sat}, normals_sat{second_sat},Tmatrix{second_sat},approx_T{second_sat},drawings(second_sat,:),Sat{second_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
delete(path)

    %Now to position
    Start= Robot.fkine(Robot.getpos).T;
    End = final_pos{first_sat}{satellite_pairs{first_sat}.sat_face_index}.T;
    
    [q,path] = Correct_orientation(Robot,Start,End,Steps);

    for i=1:size(q,1)
        delete(drawings(second_sat,:));
        T=Robot.fkine(q(i,:));
        T=T.T;
        T=T*Tcentro;

        [points_sat{second_sat}, normals_sat{second_sat},Tmatrix{second_sat},approx_T{second_sat},drawings(second_sat,:),Sat{second_sat}] = DrawSat(T);

        Robot.plot(q(i,:))
        drawnow
    end
delete(path)

%% End
Start = Robot.fkine(Robot.getpos).T;
End   = docking_targets{2};
 [final_traj,path] = Correct_orientation(Robot,Start,End,Steps);

Robot.plot(final_traj)


%% Funções

function [q_out, Robot] = MoveToNewBase(Robot, new_base_T, targetpos,approx_point_1, approx_point_2, q_current,index)

    %Step 1: Go to approx point of current state aka uncouple
    T_start = Robot.fkine(Robot.getpos); % Current pose (approx point)
    T_end = T_start; % Target docking pose
   % T_end.t = approx_point_1(1:3,4);
    T_end.t = T_end.t+[0 0 1500]';
    T_end = SE3(T_end);
    steps = 20; % Number of steps for linear motion
    % Generate Cartesian trajectory (linear in SE(3))
    trajectory_1 = ctraj((T_start), T_end, steps);

    %Plot trajectory
        positions = zeros(steps, 3);
    for i = 1:steps
        % Assuming each element in traj_SE3 is an SE3 object with a property .t,
        % which contains the translation (3x1 vector).
        positions(i, :) = trajectory_1(i).t(:)';
        % Alternative: positions(i,:) = transl(traj_SE3(i)); 
    end

    % Plot the desired end-effector trajectory before executing the motion.
   path= plot3(positions(:,1), positions(:,2), positions(:,3), 'b');


    % Solve IK for each Cartesian pose (use previous q as initial guess)
    q_traj = zeros(steps, Robot.n);
    q_traj(1, :) = Robot.getpos;
    for i = 2:steps
        q_traj(i, :) = Robot.ikcon(trajectory_1(i), q_traj(i-1, :));
    end

    % Animate linear motion
    Robot.plot(q_traj);
    q_current = q_traj(size(q_traj,1),:);
    delete(path)


%Step 2 using jacobian
        Start = Robot.fkine(Robot.getpos).t; % Current pose (approx point)
        
        if index == 6
        End = [-5000,0,5000]';
        elseif index == 5
        End = [5000,0,5000]';
        elseif index == 2
        End = [0,-8500,7000]';
        elseif index == 1
        End = [0,5000,6000]';
        end
        NN=100;

    time=linspace(0,1,NN);%Tempo

    path=Start+(End-Start)*time;%Caminho obedece a euqaçao parametrica da reta
    traj= plot3(path(1,:),path(2,:),path(3,:),'b');

    dr=gradient(path);

    % q = zeros(7, NN); %initialize the array for the joints' values.
    % Initial inverse kinematics to get starting position of robot
    q(:, 1) = Robot.ikine(fkine(Robot,Robot.getpos),'q0',Robot.getpos);
    for i = 2:NN
    J = Robot.jacob0(q(:,i-1)); % Compute Jacobian
    Ji = pinv(J(1:3, :)); % ... and its inverse

    %Ji = RRJacobianInv(RR,q(:,i-1));


    dq = Ji * dr(:,i); % Compute joint increments
    q(:, i) = q(:, i-1) + dq; % update the joint values valor anterior + o incremento atual


    % alldq(:,i)=dq;% Velocidades das juntas
    end

    q_current = q(:,size(q,2))';
    Robot.plot(q')
    delete(traj)


%   %Step 3 using jacobian

        Start = Robot.fkine(Robot.getpos).t; % Current pose (approx point)
        End = approx_point_2';
        NN=100;

    time=linspace(0,1,NN);%Tempo

    path=Start+(End-Start)*time;%Caminho obedece a euqaçao parametrica da reta
    % plot(path(1,:),path(2,:),'*b')

    dr=gradient(path);

    % q = zeros(7, NN); %initialize the array for the joints' values.
    % Initial inverse kinematics to get starting position of robot
    q(:, 1) = Robot.ikine(fkine(Robot,Robot.getpos),'q0',Robot.getpos);
    for i = 2:NN
    J = Robot.jacob0(q(:,i-1)); % Compute Jacobian
    Ji = pinv(J(1:3, :)); % ... and its inverse

    %Ji = RRJacobianInv(RR,q(:,i-1));


    dq = Ji * dr(:,i); % Compute joint increments
    q(:, i) = q(:, i-1) + dq; % update the joint values valor anterior + o incremento atual


    % alldq(:,i)=dq;% Velocidades das juntas
    end

    q_current = q(:,size(q,2))';
    Robot.plot(q')




steps = 15; % Number of steps for linear motion
    % Step 4: Generate linear Cartesian trajectory from approx to dock
%pause(1)
    T_start = Robot.fkine(Robot.getpos); % Current pose (approx point)
    T_end = new_base_T; % Target docking pose
    T_end = SE3(T_end);

    % Generate Cartesian trajectory (linear in SE(3))
    trajectory_cart = ctraj(T_start, T_end, steps);


        positions = zeros(steps, 3);
    for i = 1:steps
        % Assuming each element in traj_SE3 is an SE3 object with a property .t,
        % which contains the translation (3x1 vector).
        positions(i, :) = trajectory_cart(i).t(:)';
        % Alternative: positions(i,:) = transl(traj_SE3(i)); 
    end

    % Plot the desired end-effector trajectory before executing the motion.
   path4= plot3(positions(:,1), positions(:,2), positions(:,3), 'b');

    % Solve IK for each Cartesian pose (use previous q as initial guess)
    q_traj = zeros(steps, Robot.n);
    q_traj(1, :) = Robot.getpos;
    
    for i = 2:steps
        q_traj(i, :) = Robot.ikcon(trajectory_cart(i), q_traj(i-1, :));
    end

    % Animate linear motion
    Robot.plot(q_traj, 'nobase', 'noshadow');
     q_out = q_traj(size(q_traj,1),:);

delete(path4)




    % Step 5: Update the robot's base
    Robot.base = targetpos;
    

    if index == 6
            q_home = [pi, -0.8727, 1.8326, 2.0071, 0.7854, pi/2, 0];
    elseif index == 5
            q_home = [0, -2.88, 1.83, 2.01, 0.817, 1.384, 0];
    elseif index == 1%return home
        q_home=q_out;
        q_home(1)= q_home(1)+pi;
    elseif index == 2
        q_home =[0.1571, 1.5729, 2.4305, -1.658063, -0.7627, 1.5921,0];
    else
            q_home = fliplr(q_current);
            q_home(6)= q_home(6)+pi;
            q_home(2)= q_home(2)+pi;
            q_home(1)= q_home(1)+pi;
            %q_home(3)= q_home(3)+pi;
    end 
    
    Robot.plot(q_home);

    
end

function [facecenters, facenormals,T_faces,T_approx,drawings,T] = DrawSat(T)
%Definition of vertices


apotherm=519.61;
Y_length=900;
side_length=600;
Poli_Vertices = [
    side_length  Y_length/2  0;  % point 1
    side_length/2  Y_length/2 -apotherm;  % point 2
   -side_length/2  Y_length/2 -apotherm;  % point 3
   -side_length  Y_length/2  0;  % point 4
   -side_length/2  Y_length/2  apotherm;  % point 5
    side_length/2  Y_length/2  apotherm;  % point 6
    side_length -Y_length/2  0;  % point 1_2
    side_length/2 -Y_length/2 -apotherm;  % point 2_2
   -side_length/2 -Y_length/2 -apotherm;  % point 3_2
   -side_length -Y_length/2  0;  % point 4_2
   -side_length/2 -Y_length/2  apotherm;  % point 5_2
    side_length/2 -Y_length/2  apotherm   % point 6_2
];

%definition of faces
F = [
1 2 3 4   5   6 %face1
7 8 9 10 11 12 %face2
5 6 12 11 11 11 %face3
6 1 7 12 12 12 %face4
1 2 8 7 7 7%face5
2 3 9 8 8 8
3 4 10 9 9 9
4 5 11 10 10 10
];


% Step 1: Transpose and then convert to homogenous
Poli_Vertices_hom = e2h(Poli_Vertices');

% Step 2: Apply the transformation T (which is 4 x 4)
Vn_hom = T * Poli_Vertices_hom;

% Step 3: Convert back to regular 3D coordinates (n x 3)
Vn = h2e(Vn_hom);

% The new position of the vertices
Poli_Vertices = Vn';



%simple color index to paint the faces
fColor = repmat([0.5, 0.5, 0.5], size(F,1), 1);


% Panel Parameters
panel_size = [2400, 1800]; % [Width, Height]
link_length = 200;
panel_offset = Y_length + link_length + panel_size(2) / 4; % Distance from sphere center

% Panel vertices in local coordinates (homogeneous format)

panel_vertices = [...
    -0.5,  0.5,  0.5, -0.5;
    -0.5, -0.5,  0.5,  0.5;
     0,  0,  0,  0;
     1,  1,  1,  1]; % Unit square in homogeneous coordinates

% Scale the panel to the correct size
S_panel = makehgtform('scale', [panel_size, 1]); 
panel_vertices = S_panel * panel_vertices; 


% Transformation Matrices
% Rotate Panels by 90 degrees around the X-axis to align them with the hemisphere plane
R_y45 = makehgtform('yrotate', pi/4);
% Left Panel: Translated along -X axis
T_left = makehgtform('translate', [0, -panel_offset, 0]) * R_y45;
% Right Panel: Translated along +X axis
T_right = makehgtform('translate', [0, panel_offset, 0]) * R_y45;
% % Left Link: Line from sphere to panel (from -r to -r-link_length along X)
link_left = [0 0
    -Y_length/2  -Y_length/2-link_length; 
              0 0
              1 1];
% Right Link: Line from sphere to panel (from r to r+link_length along X)
link_right = [0 0
    Y_length/2 Y_length/2+link_length
              0 0
              1 1];
% Left Panel
panel_left = T_left * panel_vertices;
% Right Panel
panel_right = T_right * panel_vertices;


%Full transformation for the others
link_right = T*link_right;
link_right=link_right(1:3,:);
link_left = T*link_left;
link_left=link_left(1:3,:);
panel_right = T*panel_right;
panel_left = T*panel_left;


link_color = [0.3 0.3 0.3];
panel_color = [0, 0, 1];



view(3)
% Drawing
%prisma hexagonal
prisma=patch('Vertices',Poli_Vertices, 'Faces', F,'FaceVertexCData', fColor,'FaceColor','flat');
% Configurações finais do plot
axis equal;
grid on;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
%title('Satélite com Centros e Normais das Faces Laterais');



%paineis
panel1 = patch(panel_right(1,:), panel_right(2,:), panel_right(3,:), panel_color);
panel2 = patch(panel_left(1,:), panel_left(2,:), panel_left(3,:), panel_color);

%Linhas de ligação
line1 = plot3(link_left(1,:), link_left(2,:), link_left(3,:), 'Color', link_color, 'LineWidth', 5);
line2 = plot3(link_right(1,:), link_right(2,:), link_right(3,:), 'Color', link_color, 'LineWidth', 5);


% --- Calcular e Desenhar Centros e Normais das Faces Laterais ---
num_faces = size(F, 1);
num_side_faces = 6; % Only the rectangular side faces
normal_length = 350; % Aumentei um pouco para testar visibilidade


facecenters = zeros(6,3);
facenormals = zeros(6,3);


normal_lines={};
approxis={};
central_draws={};
for i = 3:(num_side_faces + 2) % Iterate through side faces (indices 3 to 8)
    
    face_vertex_indices = F(i, 1:4);


    face_vertices = Poli_Vertices(face_vertex_indices, :);
    center_point = mean(face_vertices, 1);

    % Calcular o vetor normal
    v1 = face_vertices(1, :);
    v2 = face_vertices(2, :);
    v4 = face_vertices(4, :);
    vecA = v2 - v1;
    vecB = v4 - v1;
    normal_vec = cross(vecA, vecB);
    norm_val = norm(normal_vec);


    if norm_val > eps % Check if magnitude is significant
        normal_vec = normal_vec / norm_val; % Normalize
    else
        warning('Face %d has zero area or collinear vertices. Cannot calculate normal.', i);
        normal_vec = [0 0 0]; % Assign a default zero vector, quiver won't draw this
    end

         % Salvar os dados
    facecenters(i-2, :) = center_point;
    facenormals(i-2, :) = normal_vec;




    % --- Approximation points calculation ---
    approx_points = zeros(size(facecenters)); % Initialize 6x3 matrix
    
    for n = 1:size(facecenters, 1)
        % Offset along the normal direction (1000 units from face center)
        approx_points(n,:) = facecenters(n,:) - facenormals(n,:) * 1000;
        
        % Optional: Visualize approximation points
        % aprox=plot3(approx_points(n,1), approx_points(n,2), approx_points(n,3),...
        %      'o', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'r', 'MarkerSize', 4);
%    approxis{n}=aprox;
    end




    

    % Desenhar o círculo (marcador) no

    % Calcula o vetor final já com o comprimento desejado
    scaled_normal = normal_vec * normal_length;

    % Verifica se o vetor escalado não é zero ou NaN antes de desenhar
    if any(isnan(scaled_normal)) || norm(scaled_normal) < eps
         disp('  Vetor escalado é NaN ou zero. Não desenhando quiver.');
    else
        % normal_line=quiver3(center_point(1), center_point(2), center_point(3), ...
        %        scaled_normal(1), scaled_normal(2), scaled_normal(3), ...
        %         0, 'Color', 'green', 'LineWidth', 2, 'MaxHeadSize', 1); % S=0, Cor Verde, MaxHeadSize=1 (padrão relativo)
    end

%normal_lines{1,i}=normal_line;

%central_draws{1,i}=center_pointdraw;
end


T_faces = cell(6, 1);  % Store transformation matrices for each face

for i = 1:6
    p = facecenters(i, :);     % The center point of the face
    n = facenormals(i, :);    % The normal vector of the face

    % Normalize the normal vector to define z-axis
    z_axis = n / norm(n);

    % Choose arbitrary x_temp not parallel to z
    if abs(dot(z_axis, [1 0 0])) < 0.9
        x_temp = [1 0 0];
    else
        x_temp = [0 1 0];
    end

    % Generate orthogonal coordinate system
    x_axis = cross(x_temp, z_axis);
    x_axis = x_axis / norm(x_axis);
    y_axis = cross(z_axis, x_axis);

    % Construct rotation and full transformation
    R = [x_axis(:), y_axis(:), z_axis(:)];
    T_face = [R, p(:); 0 0 0 1];

    T_faces{i} = SE3(T_face);
end


T_approx = cell(6, 1);  % Store transformation matrices for each approximation point

for i=1:size(approx_points,1)
    Rot = T_faces{i}.R;
    Loc = approx_points(i,:)';

    T_approx{i} = [Rot, Loc(:); 0 0 0 1];

end






drawings = [prisma, line1, line2, panel1, panel2];
drawings1 =[central_draws, normal_lines, approxis];



end



function [transformation_matrices_base, transformation_matrices_tip, transformation_matrices_sat]= DrawISS
%DRAWISS Summary of this function goes here
%   Detailed explanation goes here

T=transl([0 0 0 ])* trotx(0)*troty(0)*trotz(0);


ISS_length=20000;%direçao Y
ISS_width=3000;%direçao Z
ISS_depth=5000;%direçao X
ISS_Vertices = [
 ISS_depth/2 -ISS_length/2 -ISS_width/2
 ISS_depth/2 -ISS_length/2  ISS_width/2
 ISS_depth/2  ISS_length/2  ISS_width/2
 ISS_depth/2  ISS_length/2 -ISS_width/2
-ISS_depth/2 -ISS_length/2 -ISS_width/2
-ISS_depth/2 -ISS_length/2  ISS_width/2
-ISS_depth/2  ISS_length/2  ISS_width/2
-ISS_depth/2  ISS_length/2 -ISS_width/2
];

%definition of faces
Faces = [
1 2 3 4
5 6 7 8
1 2 6 5
3 7 8 4
2 6 7 3
1 5 8 4
];

% Step 1: Transpose and then convert to homogenous
ISS_Vertices_hom = e2h(ISS_Vertices');

% Step 2: Apply the transformation T (which is 4 x 4)
Vn_hom = T * ISS_Vertices_hom;

% Step 3: Convert back to regular 3D coordinates (n x 3)
Vn = h2e(Vn_hom);

% The new position of the vertices
ISS_Vertices = Vn';




%simple color index to paint the faces
fColor = repmat([0.5, 0.5, 0.5], size(Faces,1), 1);


% Panel Parameters
panel_size = [10000, 2000]; % [Width, Height]

link_length = 10000;

panel_offset = ISS_length/2 + 3400 + panel_size(2) / 4; % Distance from sphere center

% Panel vertices in local coordinates (homogeneous format)

panel_vertices = [...
    -0.5,  0.5,  0.5, -0.5;
    -0.5, -0.5,  0.5,  0.5;
     0,  0,  0,  0;
     1,  1,  1,  1]; % Unit square in homogeneous coordinates

% Scale the panel to the correct size
S_panel = makehgtform('scale', [panel_size, 1]); 
panel_vertices = S_panel * panel_vertices; 


% Transformation Matrices
% Rotate Panels by 90 degrees around the X-axis to align them with the hemisphere plane
R_y30 = makehgtform('yrotate', pi/3);
% Left Panel: Translated along -X axis
T_left = makehgtform('translate', [0, -panel_offset, 0]) * R_y30;
% Right Panel: Translated along +X axis
T_right = makehgtform('translate', [0, panel_offset, 0]) * R_y30;


% % Left Link: Line from sphere to panel (from -r to -r-link_length along X)
link_left = [ 0, 0
    -ISS_length/2,  -ISS_length/2-link_length; 
              0, 0
              1, 1];
% Right Link: Line from sphere to panel (from r to r+link_length along X)
link_right = [0, 0
    ISS_length/2, ISS_length/2+link_length
              0, 0
              1, 1];
% Left Panels
panel_left = T_left * panel_vertices;
panel_left2=panel_left-[0 2300 0 0]';
panel_left3=panel_left2-[0 2300 0 0]';

% Right Panels
panel_right = T_right * panel_vertices;
panel_right2=panel_right+[0 2300 0 0]';
panel_right3=panel_right2+[0 2300 0 0]';


%Full transformation for the others
link_right = T*link_right;
link_right=link_right(1:3,:);
link_left = T*link_left;
link_left=link_left(1:3,:);
panel_right = T*panel_right;
panel_right2=T*panel_right2;
panel_right3=T*panel_right3;
panel_left = T*panel_left;
panel_left2= T*panel_left2;
panel_left3= T*panel_left3;


link_color = [0.3 0.3 0.3];
panel_color = [0, 0, 1];



%Circulos braço
% Circle parameters
r_i = 200;                   % Internal radius
r_e = 300;
theta = linspace(0, 2*pi, 100);  % angle values


% Coordinates of the center point (top)
x_top = 0;
y_top1 = 5500;

y_top2 = -5500;

% Furo 1
Furo1_x_e = x_top + r_e * cos(theta);
Furo1_y_e= y_top1 + r_e * sin(theta);

Furo1_x_i = x_top + r_i * cos(theta);
Furo1_y_i= y_top1 + r_i * sin(theta);


% Furo 2
Furo2_x_e = x_top + r_e * cos(theta);
Furo2_y_e= y_top2 + r_e * sin(theta);

Furo2_x_i = x_top + r_i * cos(theta);
Furo2_y_i= y_top2 + r_i * sin(theta);


% Frente e tras
y_front=0;
z_front=0;

% Furo 1
front_y_e = y_front + r_e * cos(theta);
front_z_e= z_front + r_e * sin(theta);

front_y_i = y_front + r_i * cos(theta);
front_z_i= z_front + r_i * sin(theta);


% Define local circle centers and normals (before applying T)
local_centers = [
    0, 5500, 1500;   % Top Furo1
    0, -5500, 1500;  % Top Furo2
    0, 5500, -1500;  % Bottom Furo1
    0, -5500, -1500; % Bottom Furo2
    2500, 0, 0;      % Back
    -2500, 0, 0;     % Front
];

local_normals = [
    0, 0, 1;         % Top normals
    0, 0, 1;
    0, 0, -1;        % Bottom normals
    0, 0, -1;
    1, 0, 0;         % Front/Back normals
    -1, 0, 0;
];

% Initialize arrays for results
global_centers = zeros(size(local_centers));
global_normals = zeros(size(local_normals));
transformation_matrices_tip = cell(size(local_centers, 1), 1);


for i = 1:size(local_centers, 1)
    % Transform center point
    local_center_h = [local_centers(i, :)'; 1];  % Homogeneous
    global_center_h = T * local_center_h;
    global_center = global_center_h(1:3)';
    global_centers(i, :) = global_center;
    
    % Transform normal
    R = T(1:3, 1:3);  % Rotation part
    local_normal = local_normals(i, :)';
    global_normal = R * local_normal;

    % Flip normal to point inward
    inward_normal = -global_normal / norm(global_normal);

    % Choose a stable reference vector not colinear with normal
    if abs(dot(inward_normal, [0; 1; 0])) < 0.9
        ref = [1; 0; 0];
    else
        ref = [0; 1; 0];
    end

    % Construct orthonormal frame
    x_axis = cross(ref, inward_normal);  % X is perpendicular to Z and ref
    x_axis = x_axis / norm(x_axis);
    y_axis = cross(inward_normal, x_axis);  % Y completes right-handed frame

    % Assemble transformation matrix
    T_matrix = eye(4);
    T_matrix(1:3, 1:3) = [x_axis, y_axis, inward_normal];  % X Y Z as cols
    T_matrix(1:3, 4) = global_center';
    transformation_matrices_tip{i} = (T_matrix);  % or T_matrix if not SE3
end


% Initialize arrays for results
global_centers = zeros(size(local_centers));
global_normals = zeros(size(local_normals));
transformation_matrices_base = cell(size(local_centers, 1), 1);


for i = 1:size(local_centers, 1)
    % Transform center
    local_center_h = [local_centers(i, :)'; 1]; % Homogeneous coordinates
    global_center_h = T * local_center_h;
    global_center = global_center_h(1:3)';
    global_centers(i, :) = global_center;

    % Transform normal (rotation only)
    R = T(1:3, 1:3); % Rotation part of T
    local_normal = local_normals(i, :)';
    global_normal = R * local_normal;
    global_normals(i, :) = global_normal';

% Construct transformation matrix for the circle
z_axis = global_normal / norm(global_normal);

% Choose a non-aligned axis for cross product
if abs(z_axis(1)) < 0.9
    temp = [1; 0; 0]; % Use X-axis
else
    temp = [0; 1; 0]; % Use Y-axis if z_axis is near X
end

% Compute orthogonal axes
y_axis = cross(z_axis, temp);
y_axis = y_axis / norm(y_axis);
x_axis = cross(y_axis, z_axis);

% Build transformation matrix
trans_matrix = eye(4);
trans_matrix(1:3, 1:3) = [x_axis, y_axis, z_axis];
trans_matrix(1:3, 4) = global_center';
transformation_matrices_base{i} = trans_matrix;
end


% Drawing
%Paralelepipedo
%view(3)
view(130,50)
paralelepipedo=patch('Vertices',ISS_Vertices, 'Faces', Faces,'FaceVertexCData', fColor,'FaceColor','flat');
axis equal
hold on
xlabel('X')
ylabel('Y')
zlabel('Z')
axis square
axis([-20000 20000 -20000 20000 -20000 20000])


%paineis
patch(panel_right(1,:), panel_right(2,:), panel_right(3,:), panel_color);
patch(panel_left(1,:), panel_left(2,:), panel_left(3,:), panel_color);
patch(panel_right2(1,:), panel_right2(2,:), panel_right2(3,:), panel_color);
patch(panel_right3(1,:), panel_right3(2,:), panel_right3(3,:), panel_color);
patch(panel_left2(1,:), panel_left2(2,:), panel_left2(3,:), panel_color);
patch(panel_left3(1,:), panel_left3(2,:), panel_left3(3,:), panel_color);


%Linhas de ligação
plot3(link_left(1,:), link_left(2,:), link_left(3,:), 'Color', link_color, 'LineWidth', 5);
plot3(link_right(1,:), link_right(2,:), link_right(3,:), 'Color', link_color, 'LineWidth', 5);
hold on

%Furos de cima e baixo
for z_furo = [-ISS_width/2, ISS_width/2]
    z_furos = z_furo * ones(size(theta));  % Furo 1
    patch(Furo1_x_e, Furo1_y_e, z_furos,'r');  % Furo 1
    patch(Furo1_x_i, Furo1_y_i, z_furos,'y');  % Furo interno 1
    patch(Furo2_x_e, Furo2_y_e, z_furos,'r');  % Furo 2
    patch(Furo2_x_i, Furo2_y_i, z_furos,'y');  % Furo interno 2
end

for x_furo = [-ISS_depth/2,ISS_depth/2]
    x_furos = x_furo * ones(size(theta));  % Furo frente e tras

    patch(x_furos,front_y_e,front_z_e,'r')
    patch(x_furos,front_y_i,front_z_i,'y')
end


% Define local circle centers and normals (before applying T)
local_centers_1 = [
    0, 1500, 4000;   % Top Furo1
    0, -1500, 4000;  % Top Furo2
    0, 1500, -4000;  % Bottom Furo1
    0, -1500, -4000; % Bottom Furo2
    % Front
];

local_normals_1 = [
    0, 0, 1;         % Top normals
    0, 0, 1;
    0, 0,  -1;       % Bottom normals
    0, 0,  -1;
];

% Initialize arrays for results
global_centers_1 = zeros(size(local_centers_1));
global_normals_1 = zeros(size(local_normals_1));
transformation_matrices_sat = cell(size(local_centers_1, 1), 1);


for i = 1:size(local_centers_1, 1)
    % Transform center point
    local_center_h_1 = [local_centers_1(i, :)'; 1];  % Homogeneous
    global_center_h_1 = T * local_center_h_1;
    global_center_1 = global_center_h_1(1:3)';
    global_centers_1(i, :) = global_center_1;
    
    % Transform normal
    R = T(1:3, 1:3);  % Rotation part
    local_normal_1 = local_normals_1(i, :)';
    global_normal_1 = R * local_normal_1;


    % Construct transformation matrix for the circle
    z_axis = -global_normal_1 / norm(global_normal_1);
    
    % Choose a non-aligned axis for cross product
    %if abs(dot(z_axis,[0;1;0])) < 0.9
    if abs(z_axis(1)) < 0.9
        reff = [0; 1; 0]; % Use X-axis
    else
        reff = [1; 0; 0]; % Use Y-axis if z_axis is near X
    end
    
    % Compute orthogonal axes
    x_axis = cross(reff,z_axis);
    x_axis = x_axis / norm(x_axis);
    y_axis = cross( z_axis,x_axis);
    
    % Build transformation matrix
    trans_matrix_sat = eye(4);
    trans_matrix_sat(1:3, 1:3) = [x_axis, y_axis, z_axis];
    trans_matrix_sat(1:3, 4) = global_center_1';
    transformation_matrices_sat{i} = trans_matrix_sat;
end
end

function [q,traj] = Calculatetraj(Robot, Start, End, Steps)
%helped by Copilot
% computeTrajectory computes a joint-space trajectory for the robot using 
% differential inverse kinematics to follow a linear Cartesian path.
%
% Inputs:
%   Robot - a robot object with the methods fkine, jacob0, ikine, and getpos.
%   Start - a 4x4 homogeneous transformation matrix representing the start pose.
%   End   - a 4x4 homogeneous transformation matrix representing the target pose.
%   Steps - (optional) number of steps in the trajectory (default: 150).
%
% Output:
%   q     - a 7xSteps matrix containing the computed joint-space trajectory.
%
% Example:
%   q = computeTrajectory(Robot, Start, End, 150);

    if nargin < 4
        Steps = 150;
    end

    % Generate a normalized time vector from 0 to 1.
    time = linspace(0, 1, Steps);

    % Create a linear interpolation between the start and the target positions.
    % Only the translational part (fourth column) is used.
    path = Start(1:3,4) + (End(1:3,4) - Start(1:3,4)) * time;
    traj=plot3(path(1,:),path(2,:),path(3,:),'b');
    % Compute the translational velocity using a numerical gradient.
    dr = gradient(path);

    % Preallocate the joint-space trajectory matrix.
    q = zeros(7, Steps);

    % Determine the starting joint configuration that achieves the Start pose.
    q(:, 1) = Robot.ikine(Start, 'q0', Robot.getpos);

    % Differential Inverse Kinematics Loop:  
    % Only the translational part of the Jacobian (first 3 rows) is used here.
    for i = 2:Steps
        % Compute the Jacobian for the current joint configuration.
        J = Robot.jacob0(q(:, i-1));
        
        % Compute the pseudo-inverse of the translational part.
        Ji = pinv(J(1:3, :));
        
        % Determine the joint increments required to follow the Cartesian path.
        dq = Ji * dr(:, i);
        
        % Update the joint configuration.
        q(:, i) = q(:, i-1) + dq;
    end
    q=q';
end
function [q,path] = Correct_orientation(Robot,Start,End,steps)


    %Step 1: Go to approx point of current state aka uncouple

    %steps = 15; % Number of steps for linear motion
    % Generate Cartesian trajectory (linear in SE(3))
    trajectory_1 = ctraj(SE3(Start), SE3(End), steps);
 
        positions = zeros(steps, 3);
    for i = 1:steps
        % Assuming each element in traj_SE3 is an SE3 object with a property .t,
        % which contains the translation (3x1 vector).
        positions(i, :) = trajectory_1(i).t(:)';
        % Alternative: positions(i,:) = transl(traj_SE3(i)); 
    end

    % Plot the desired end-effector trajectory before executing the motion.
   path= plot3(positions(:,1), positions(:,2), positions(:,3), 'b');
    
 
 
 % Solve IK for each Cartesian pose (use previous q as initial guess)
    q_traj = zeros(steps, Robot.n);
    q_traj(1, :) = Robot.getpos;
    for i = 2:steps
        q_traj(i, :) = Robot.ikcon(trajectory_1(i), q_traj(i-1, :));
    end

    q=q_traj;


end

function satellite_docking_pairs = computeSatelliteDockingPairs(docking_positions, satellites_faces)
%Created by Copilot
% computeSatelliteDockingPairs computes the closest docking port pairing for each satellite.
%
% Inputs:
%   docking_positions - A cell array of 4x4 transformation matrices, one per docking port.
%                       The docking position is taken from the 4th column of each matrix.
%   satellites_faces  - A cell array where each cell contains an Nx3 matrix of face points
%                       for a given satellite.
%
% Output:
%   satellite_docking_pairs - A cell array of structures. Each structure has the following fields:
%       satellite_index   - Index of the satellite.
%       docking_index     - Index of the docking port (from docking_positions) closest to the satellite.
%       docking_position  - A 1x3 vector with the docking port position.
%       sat_face          - The satellite face point (1x3 vector) that is closest to the docking port.
%       sat_face_index    - The index (row) of the satellite face point in the corresponding matrix.
%       distance          - The Euclidean distance between the docking port and the satellite face.
%
% Example usage:
%   satellite_docking_pairs = computeSatelliteDockingPairs(Docking_positions, {points_sat1, points_sat2});

    num_satellites = length(satellites_faces);
    satellite_docking_pairs = cell(num_satellites, 1);

    % Loop over each satellite
    for sat_idx = 1:num_satellites
        current_faces = satellites_faces{sat_idx};
        best_distance = inf;
        best_dock_idx = -1;
        best_face_point = [];
        best_face_index = -1;
        
        % Loop through every docking port to find the closest face point for this satellite
        for dock_idx = 1:length(docking_positions)
            % Extract the docking position (1x3 row vector) from the transformation matrix
            current_docking = docking_positions{dock_idx}(1:3, 4)';
            
            % Compute the Euclidean distances from the docking position to each face point
            distances = sqrt(sum((current_faces - current_docking).^2, 2));
            [min_dist, min_face_idx] = min(distances);
            
            % Update if this docking port gives a closer pair than previously recorded
            if min_dist < best_distance
                best_distance = min_dist;
                best_dock_idx = dock_idx;
                best_face_point = current_faces(min_face_idx, :);
                best_face_index = min_face_idx;
            end
        end
        
        % Store the pairing info in a structure
        satellite_docking_pairs{sat_idx} = struct(...
            'satellite_index', sat_idx, ...
            'docking_index', best_dock_idx, ...
            'docking_position', docking_positions{best_dock_idx}(1:3,4)', ...
            'sat_face', best_face_point, ...
            'sat_face_index', best_face_index, ...
            'distance', best_distance);
    end
end
